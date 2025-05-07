#!/home/esoc/anaconda3/envs/megapose/bin/python3
# coding :utf-8
# ROS1
import rospy
from visp_megapose.srv import Init, Track, Render
import transforms3d
from geometry_msgs.msg import Transform as RosTransform, Vector3, Quaternion
from sensor_msgs.msg import Image
import os
import json
import sys
import megapose_server
# Set megapose environment variables
sys.path.append('/home/esoc/anaconda3/envs/megapose/lib/python3.8/site-packages')
megapose_server_install_dir = os.path.dirname(megapose_server.__file__)
variables_file = '/home/esoc/ros_ws/src/vision_visp/visp_megapose/scripts/megapose_variables_final.json'
with open(variables_file, 'r') as f:
    json_vars = json.load(f)
    print('Loaded megapose variables', json_vars)
    os.environ['MEGAPOSE_DIR'] = json_vars['megapose_dir']
    os.environ['MEGAPOSE_DATA_DIR'] = json_vars['megapose_data_dir']

if 'HOME' not in os.environ: # Home is always required by megapose but is not always set
    if os.name == 'nt':
      if 'HOMEPATH' in os.environ:
        os.environ['HOME'] = os.environ['HOMEPATH']
      elif 'HOMEDIR' in os.environ:
        os.environ['HOME'] = os.environ['HOMEDIR']
      else:
        os.environ['HOME'] = '.'
    else:
      os.environ['HOME'] = '.'

# 3rd party
import numpy as np
import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
from cv_bridge import CvBridge, CvBridgeError
import pandas as pd
import torch
import torch.fx as fx
import torch.nn as nn
from torch.fx.experimental.optimization import optimize_for_inference, fuse
import time
# MegaPose
from megapose.datasets.object_dataset import RigidObject, RigidObjectDataset
from megapose.datasets.scene_dataset import CameraData, ObjectData
from megapose.inference.types import (
    DetectionsType,
    ObservationTensor,
    PoseEstimatesType,
)
from megapose.inference.utils import make_detections_from_object_data
from megapose.lib3d.transform import Transform
from megapose.panda3d_renderer import Panda3dLightData
from megapose.panda3d_renderer.panda3d_scene_renderer import Panda3dSceneRenderer
from megapose.utils.conversion import convert_scene_observation_to_panda3d
from megapose.utils.load_model import NAMED_MODELS, load_named_model

# Megapose server
from megapose_server.network_utils import *
from megapose_server.server_operations import ServerMessage

megapose_models = {
        'RGB': ('megapose-1.0-RGB', False),
        'RGBD': ('megapose-1.0-RGBD', True),
        'RGB-multi-hypothesis': ('megapose-1.0-RGB-multi-hypothesis', False),
        'RGBD-multi-hypothesis': ('megapose-1.0-RGB-multi-hypothesis-icp', True),
    }
camera_data = {
        'K': np.asarray([
            [700, 0.0, 320],
            [0.0, 700, 240],
            [0.0, 0.0, 1.0]
        ]),
        'h': 480,
        'w': 640
}

def make_object_dataset(meshes_dir: Path) -> RigidObjectDataset:
    rigid_objects = []
    mesh_units = "m"
    object_dirs = meshes_dir.iterdir()
    for object_dir in object_dirs:
        label = object_dir.name
        mesh_path = None
        for fn in object_dir.glob("*"):
            if fn.suffix in {".obj", ".ply", ".glb", ".gltf"}:
                assert not mesh_path, f"there are multiple meshes in the {label} directory"
                mesh_path = fn
        assert mesh_path, f"couldn't find the mesh for {label}"
        rigid_objects.append(RigidObject(label=label, mesh_path=mesh_path, mesh_units=mesh_units))
    rigid_object_dataset = RigidObjectDataset(rigid_objects)
    return rigid_object_dataset

class MegaPoseServer:
    """
    ROS service 版本的 MegaPose 伺服器，提供下列服務：
      1. initial_pose (Init) : 根據輸入影像、相機資訊、物體名稱與檢測框，估計初始位姿與信心分數。
      2. track_pose (Track) : 根據輸入影像與先前位姿，更新物體位姿與信心分數。
      3. render_object (Render) : 根據物體名稱與位姿，渲染物體圖像並回傳 ROS Image。
    """
    def __init__(self, image_batch_size=256, warmup=True):
        rospy.init_node('MegaPoseServer')

        mesh_dir = rospy.get_param('~mesh_dir', 'visp_megapose/data/models')
        mesh_dir = Path(mesh_dir).absolute()
        assert mesh_dir.exists(), 'Mesh directory does not exist, cannot start server'

        model_name = rospy.get_param('~megapose_models', 'RGB')
        model_use_depth = megapose_models[model_name][1]
        model_name = megapose_models[model_name][0]

        num_workers = rospy.get_param('~num_workers', 4)
        optimize = rospy.get_param('~optimize', False)

        self.init_service = rospy.Service('initial_pose', Init, self.InitPoseCallback)
        self.track_service = rospy.Service('track_pose', Track, self.TrackPoseCallback)
        self.render_service = rospy.Service('render_object', Render, self.RenderObjectCallback)
       
        

        self.num_workers = num_workers
        self.object_dataset: RigidObjectDataset = make_object_dataset(mesh_dir)
        model_tuple = self._load_model(model_name)
        self.model_info = model_tuple[0]
        self.model = model_tuple[1]
        self.model.eval()
        self.model.bsz_images = image_batch_size
        self.camera_data = self._make_camera_data(camera_data)
        self.renderer = Panda3dSceneRenderer(self.object_dataset)
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = False
        self.optimize = optimize
        self.warmup = warmup
        self.model_use_depth = model_use_depth

        rospy.logwarn("MegaPose server initialized")
        rospy.logwarn("Model: %s", model_name)
        rospy.logwarn("Model requires depth: %s", self.model_use_depth)
        rospy.logwarn("Mesh directory: %s", mesh_dir)
        rospy.logwarn("Number of workers: %d", num_workers)
        rospy.logwarn("Optimize: %s", optimize)
        rospy.logwarn("Warmup: %s", warmup)

        if self.optimize:
            print('Optimizing Pytorch models...')
            class Optimized(nn.Module):
                def __init__(self, m: nn.Module, inp):
                    super().__init__()
                    self.m = m.eval()
                    self.m = fuse(self.m, inplace=False)
                    self.m = torch.jit.trace(self.m, torch.rand(inp).cuda())
                    self.m = torch.jit.freeze(self.m)

                def forward(self, x):
                    return self.m(x).float()

            h, w = self.camera_data.resolution
            self.model.coarse_model.backbone = Optimized(self.model.coarse_model.backbone, (1, 9, h, w))
            self.model.refiner_model.backbone = Optimized(self.model.refiner_model.backbone, (1, 32 if self.model_info['requires_depth'] else 27, h, w))

        if self.warmup:
            rospy.loginfo('Warming up models...')
            h, w = self.camera_data.resolution
            labels = self.object_dataset.label_to_objects.keys()
            observation = self._make_observation_tensor(np.random.randint(0, 255, (h, w, 3), dtype=np.uint8),
                                                        np.random.rand(h, w).astype(np.float32) if self.model_info['requires_depth'] else None).cuda()
            detections = self._make_detections(labels, np.asarray([[0, 0, w//2, h//2] for _ in range(len(labels))], dtype=np.float32)).cuda()
            self.model.run_inference_pipeline(observation, detections, **self.model_info['inference_parameters'])
        rospy.loginfo('waiting for requests...')

    def _load_model(self, model_name):
        return NAMED_MODELS[model_name], load_named_model(model_name, self.object_dataset, n_workers=self.num_workers).cuda()

    def _make_camera_data(self, camera_data: Dict) -> CameraData:
        '''
        Create a camera representation that is understandable by megapose.
        camera_data: A dict containing the keys K, h, w
        K is the 3x3 intrinsics matrix
        h and w are the input image resolution.

        Returns a CameraData object, to be given to megapose.
        '''
        c = CameraData()
        c.K = camera_data['K']
        c.resolution = (camera_data['h'], camera_data['w'])
        print(c)
        c.z_near = 0.001
        c.z_far = 100000
        return c

    def _make_observation_tensor(self, image: np.ndarray, depth: Optional[np.ndarray] = None) -> ObservationTensor:
        '''
        Create an observation tensor from an image and a potential depth image
        '''
        return ObservationTensor.from_numpy(image, depth, self.camera_data.K)

    def _make_detections(self, labels, detections):
        result = []
        for label, detection in zip(labels, detections):
            o = ObjectData(label)
            o.bbox_modal = detection
            result.append(o)
        return make_detections_from_object_data(result)

    def InitPoseCallback(self, req):
        rospy.loginfo("InitPose service request received")
        bridge = CvBridge()
        pose = RosTransform()
        confidence = 0.0
        try:
            img = bridge.imgmsg_to_cv2(req.image, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            return pose, confidence
        # 使用 depth 資訊（如果 req.use_depth 為 True）
        depth = None
        if self.model_use_depth:
            try:
                # depth_uint16 = bridge.imgmsg_to_cv2(req.depth, desired_encoding="passthrough")
                # depth = depth_uint16.astype(np.float32) / 1.0
                depth_uint16 = bridge.imgmsg_to_cv2(req.depth, desired_encoding="passthrough")
                depth = depth_uint16.astype(np.float32) / 1000.0
            except CvBridgeError as e:
                rospy.logerr("CvBridge error (depth): %s", e)
                return pose, confidence
        # 更新相機資訊
        camera_data = {
            'K': np.asarray([
                [req.camera_info.K[0], req.camera_info.K[1], req.camera_info.K[2]],
                [req.camera_info.K[3], req.camera_info.K[4], req.camera_info.K[5]],
                [req.camera_info.K[6], req.camera_info.K[7], req.camera_info.K[8]]
            ]),
            'h': req.camera_info.height,
            'w': req.camera_info.width
        }
        self.camera_data = self._make_camera_data(camera_data)
        # 物件偵測：利用請求中的 bounding box
        object_name = [req.object_name]
        detections = [[req.topleft_j, req.topleft_i, req.bottomright_j, req.bottomright_i]]
        detections = self._make_detections(object_name, detections).cuda()
        observation = self._make_observation_tensor(img, depth).cuda()
        inference_params = self.model_info['inference_parameters'].copy()
        output, extra_data = self.model.run_inference_pipeline(
            observation, detections=detections, **inference_params, coarse_estimates=None
        )
        poses = output.poses.cpu().numpy().reshape(-1,4,4)
        conf = output.infos['pose_score'].to_numpy()
        pose.translation.x = float(poses[0][0,3])
        pose.translation.y = float(poses[0][1,3])
        pose.translation.z = float(poses[0][2,3])
        rotation = poses[0][0:3,0:3]
        rotation = transforms3d.quaternions.mat2quat(rotation)
        pose.rotation.w = rotation[0]
        pose.rotation.x = rotation[1]
        pose.rotation.y = rotation[2]
        pose.rotation.z = rotation[3]
        confidence = float(conf[0])
        return pose, confidence

    def TrackPoseCallback(self, req):
        # rospy.loginfo("TrackPose service request received")
        bridge = CvBridge()
        pose = RosTransform()
        confidence = 0.0
        try:
            img = bridge.imgmsg_to_cv2(req.image, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            return pose, confidence
        depth = None
        if self.model_use_depth:
            try:
                depth_uint16 = bridge.imgmsg_to_cv2(req.depth, desired_encoding="passthrough")
                depth = depth_uint16.astype(np.float32) / 1000.0
            except CvBridgeError as e:
                rospy.logerr("CvBridge error (depth): %s", e)
                return pose, confidence
        object_name = [req.object_name]
        cTos = np.eye(4)
        cTos[0:3,3] = [req.init_pose.translation.x, req.init_pose.translation.y, req.init_pose.translation.z]
        cTos[0:3,0:3] = transforms3d.quaternions.quat2mat([
            req.init_pose.rotation.w, req.init_pose.rotation.x,
            req.init_pose.rotation.y, req.init_pose.rotation.z])
        cTos = cTos.reshape(1,4,4)
        tensor = torch.from_numpy(cTos).float().cuda()
        infos = pd.DataFrame.from_dict({
            'label': object_name,
            'batch_im_id': [0 for _ in range(len(cTos))],
            'instance_id': [i for i in range(len(cTos))]
        })
        coarse_estimates = PoseEstimatesType(infos, poses=tensor)
        observation = self._make_observation_tensor(img, depth).cuda()
        inference_params = self.model_info['inference_parameters'].copy()
        inference_params['n_refiner_iterations'] = req.refiner_iterations
        output, extra_data = self.model.run_inference_pipeline(
            observation, detections=None, **inference_params, coarse_estimates=coarse_estimates
        )
        poses = output.poses.cpu().numpy()
        poses = poses.reshape(len(poses), 4, 4)
        conf = output.infos['pose_score'].to_numpy()
        bounding_boxes = extra_data['scoring']['preds'].tensors['boxes_rend'].cpu().numpy().reshape(-1, 4)
        bounding_boxes = bounding_boxes.tolist()

        pose.translation.x = float(poses[0][0,3])
        pose.translation.y = float(poses[0][1,3])
        pose.translation.z = float(poses[0][2,3])
        rotation = poses[0][0:3,0:3]
        rotation = transforms3d.quaternions.mat2quat(rotation)
        pose.rotation.w = rotation[0]
        pose.rotation.x = rotation[1]
        pose.rotation.y = rotation[2]
        pose.rotation.z = rotation[3]
        confidence = float(conf[0])
        return pose, confidence

    def RenderObjectCallback(self, request):
        labels = [request.object_name]
        poses = np.eye(4)
        poses[0:3, 3] = [request.pose.translation.x, request.pose.translation.y, request.pose.translation.z]
        poses[0:3, 0:3] = transforms3d.quaternions.quat2mat([request.pose.rotation.w, request.pose.rotation.x, request.pose.rotation.y, request.pose.rotation.z])
        poses = poses.reshape(1, 4, 4)

        camera_data = CameraData()
        camera_data.K = self.camera_data.K
        camera_data.resolution = self.camera_data.resolution
        camera_data.TWC = Transform(np.eye(4))

        object_datas = []
        for label, pose in zip(labels, poses):
            object_datas.append(ObjectData(label=label, TWO=Transform(pose)))
        camera_data, object_datas = convert_scene_observation_to_panda3d(camera_data, object_datas)
        light_datas = [
            Panda3dLightData(
                light_type="ambient",
                color=((1.0, 1.0, 1.0, 1)),
            ),
        ]
        renderings = self.renderer.render_scene(
            object_datas,
            [camera_data],
            light_datas,
            render_depth=False,
            render_binary_mask=False,
            render_normals=False,
            copy_arrays=True,
        )[0]

        img = renderings.rgb
        img = np.uint8(img).reshape(1, -1).tolist()[0]

        image = Image()
        image.header.stamp = rospy.Time.now()
        image.height = renderings.rgb.shape[0]
        image.width = renderings.rgb.shape[1]
        image.encoding = 'rgb8'
        image.is_bigendian = 0
        image.step = 3 * renderings.rgb.shape[1]
        image.data = img
        return image

    def _set_intrinsics(self, req):
        K = np.asarray([[req.px, 0.0, req.u0],
                        [0.0, req.py, req.v0],
                        [0.0, 0.0, 1.0]])
        cam_data = {'K': K, 'h': req.h, 'w': req.w}
        self.camera_data = self._make_camera_data(cam_data)
        # 若成功則回傳 True
        from visp_megapose.srv import SetIntrinsicsResponse
        return SetIntrinsicsResponse(True)

    def _score(self, req):
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(req.image, desired_encoding="rgb8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            from visp_megapose.srv import GetScoreResponse
            return GetScoreResponse([])
        labels = req.labels
        poses = req.cTos
        pose_estimates = None
        if poses:
            cTos_np = np.array(poses).reshape(-1,4,4)
            tensor = torch.from_numpy(cTos_np).float().cuda()
            infos = pd.DataFrame({
                'label': labels,
                'batch_im_id': [0] * len(cTos_np),
                'instance_id': list(range(len(cTos_np)))
            })
            pose_estimates = PoseEstimatesType(infos, poses=tensor)
        observation = self._make_observation_tensor(img).cuda()
        result = self.model.forward_scoring_model(observation, pose_estimates)
        scores = result[0].infos['pose_score'].tolist()
        from visp_megapose.srv import GetScoreResponse
        return GetScoreResponse(scores)

    def _set_SO3_grid_size(self, req):
        def random_quaternion(rand=None):
            if rand is None:
                rand = np.random.rand(3)
            else:
                assert len(rand) == 3
            r1 = np.sqrt(1.0 - rand[0])
            r2 = np.sqrt(rand[0])
            pi2 = np.pi * 2.0
            t1 = pi2 * rand[1]
            t2 = pi2 * rand[2]
            return np.array([np.sin(t1)*r1, np.cos(t1)*r1, np.sin(t2)*r2, np.cos(t2)*r2])
        value = req.so3_grid_size
        if value in [72, 512, 576, 4608]:
            self.model.load_SO3_grid(value)
        else:
            rospy.logwarn("Non-standard SO(3) grid size, generating random orientations.")
            import roma
            Rs = roma.random_rotmat(value)
            self.model._SO3_grid = Rs.cuda()
        from visp_megapose.srv import SetSO3GridSizeResponse
        return SetSO3GridSizeResponse(True)

    def _list_objects(self, req):
        from visp_megapose.srv import ListObjectsResponse
        objects = list(self.object_dataset.label_to_objects.keys())
        return ListObjectsResponse(objects)

if __name__ == '__main__':
    MegaPoseServer()
    rospy.spin()
