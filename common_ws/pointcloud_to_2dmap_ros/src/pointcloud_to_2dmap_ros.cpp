#include <iostream>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>

class MapGenerator
{
public:
    double resolution; // meters per pixel
    double m2pix;      // inverse resolution (pix/m)
    int map_width;
    int map_height;
    int min_points_in_pix;
    int max_points_in_pix;
    double min_height;
    double max_height;
    std::string dest_directory;
    std::string input_pcd;
    std::string map_name;

    cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ> &cloud) const
    {
        cv::Mat map(map_height, map_width, CV_8UC1, cv::Scalar::all(0));

        for (const auto &point : cloud)
        {
            if (point.z < min_height || point.z > max_height)
            {
                continue;
            }

            int x = static_cast<int>(point.x * m2pix + map_width / 2);
            int y = static_cast<int>(-point.y * m2pix + map_height / 2);

            if (x < 0 || x >= map_width || y < 0 || y >= map_height)
            {
                continue;
            }

            map.at<uchar>(y, x)++;
        }

        map -= min_points_in_pix;
        map.convertTo(map, CV_8UC1, -255.0 / (max_points_in_pix - min_points_in_pix), 255);

        return map;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_to_2dmap_node");
    ros::NodeHandle nh("~");

    MapGenerator map_generator;
    std::string dest_directory, input_pcd, map_name;

    nh.param<double>("resolution", map_generator.resolution, 0.1);
    nh.param<int>("map_width", map_generator.map_width, 10240);
    nh.param<int>("map_height", map_generator.map_height, 10240);
    nh.param<int>("min_points_in_pix", map_generator.min_points_in_pix, 2);
    nh.param<int>("max_points_in_pix", map_generator.max_points_in_pix, 5);
    nh.param<double>("min_height", map_generator.min_height, 0.5);
    nh.param<double>("max_height", map_generator.max_height, 3.0);
    nh.param<std::string>("dest_directory", dest_directory, "");
    nh.param<std::string>("input_pcd", input_pcd, "");
    nh.param<std::string>("map_name", map_name, "map");

    map_generator.m2pix = 1.0 / map_generator.resolution;

    ROS_INFO("resolution    : %f", map_generator.resolution);
    ROS_INFO("m2pix         : %f", map_generator.m2pix);
    ROS_INFO("map_width     : %d", map_generator.map_width);
    ROS_INFO("map_height    : %d", map_generator.map_height);
    ROS_INFO("min_points_in_pix    : %d", map_generator.min_points_in_pix);
    ROS_INFO("max_points_in_pix    : %d", map_generator.max_points_in_pix);
    ROS_INFO("min_height    : %f", map_generator.min_height);
    ROS_INFO("max_height    : %f", map_generator.max_height);
    ROS_INFO("dest_directory: %s", dest_directory.c_str());
    ROS_INFO("input_pcd     : %s", input_pcd.c_str());
    ROS_INFO("map_name      : %s", map_name.c_str());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *cloud) == -1)
    {
        ROS_ERROR_STREAM("Failed to open the input cloud: " << input_pcd);
        return 1;
    }
    ROS_DEBUG("Read PointCloud OK!");

    cv::Mat map = map_generator.generate(*cloud);

    if (!boost::filesystem::exists(dest_directory))
    {
        boost::filesystem::create_directories(dest_directory);
    }

    std::string map_path = dest_directory + "/" + map_name + ".png";
    cv::imwrite(map_path, map);
    ROS_INFO_STREAM("Saved 2D map as PNG: " << map_path);

    std::string yaml_path = dest_directory + "/map.yaml";
    std::ofstream ofs(yaml_path);
    ofs << "image: " << map_name << ".png" << std::endl;
    ofs << "resolution: " << map_generator.resolution << std::endl;
    ofs << "origin: ["
        << -map_generator.resolution * map_generator.map_width / 2 << ", "
        << -map_generator.resolution * map_generator.map_height / 2 << ", 0.0]" << std::endl;
    ofs << "occupied_thresh: 0.5" << std::endl;
    ofs << "free_thresh: 0.2" << std::endl;
    ofs << "negate: 0" << std::endl;
    ROS_INFO_STREAM("Saved YAML file: " << yaml_path);

    ROS_INFO("If the 2D map name is changed, update the image name in the YAML file!");

    return 0;
}
