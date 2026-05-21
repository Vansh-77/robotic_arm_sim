import numpy as np

class RoboticArm2D:
    def __init__(self, l1 , l2):
        self.l1 = l1
        self.l2 = l2
        
        self.theta1 = 0
        self.theta2 = 0
        
    def set_angles(self , theta1 , theta2):
            self.theta1 = theta1 
            self.theta2 = theta2
        
    def forward_kinematics(self):
            x1 = self.l1 * np.cos(self.theta1)
            y1 = self.l1 * np.sin(self.theta1)
            
            x2 = x1 + self.l2 * np.cos(self.theta1 + self.theta2)
            y2 = y1 + self.l2 * np.sin(self.theta1 + self.theta2)
            
            return [0 , x1 , x2 ] , [0 , y1 , y2] 
        
    def get_end_effector(self):

            x, y = self.forward_kinematics()

            return x[-1], y[-1]
        