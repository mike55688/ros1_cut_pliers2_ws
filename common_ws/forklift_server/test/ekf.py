#!/usr/bin/env python
class KalmanFilter():
    initialized = False
    _P = .0
    _Q = .0
    _R = .0
    x_hat = .0

    def init (self,P,Q,R):
        self._P = P
        self._Q = Q
        self._R = R
        self.x_hat = 0
        #P = P_minus
        self.initialized = True


    def update(self,z):
        #print (self._P),(self._Q),(self._R)

        if(not self.initialized):
            # print 'Filter is not initialized!'
        self.x_hat_minus = float(self.x_hat) # self.x_hat_minus=0
        P_minus = float(self._P + self._Q)  #P_minus=12
        
        self.K = float(P_minus / (P_minus + self._R))
        
        self.x_hat = float(self.x_hat_minus + self.K * (z - self.x_hat_minus))
        P = float((1 - self.K) * P_minus)
        
        return self.x_hat