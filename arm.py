import numpy as np
from config import *
class RoboticArm2D:
    def __init__(self, l1 , l2):
        self.l1 = l1
        self.l2 = l2
        self.max_velocity = MAX_VELOCITY
        
        self.theta1 = 0
        self.theta2 = 0
        
        self.target_theta1 = 0 
        self.target_theta2 = 0
        
        self.trajectory = []
        self.current_waypoint = 0
        
    def plan_trajectory(self , target_theta1 , target_theta2 , steps = 100):
        self.trajectory=[]
        
        theta1_points = np.linspace(
            self.theta1,
            target_theta1,
            steps
        )

        theta2_points = np.linspace(
            self.theta2,
            target_theta2,
            steps
        )

        self.trajectory = list(
            zip(theta1_points, theta2_points)
        )

        self.current_waypoint = 0    
        
        
    def update_motion(self , kp = 0.5):

        if self.current_waypoint >= len(self.trajectory):
           return

        self.target_theta1, self.target_theta2 = self.trajectory[
            self.current_waypoint
        ]

        self.theta1 += np.clip(kp * (self.target_theta1 - self.theta1), -self.max_velocity , self.max_velocity)
        self.theta2 += np.clip(kp * (self.target_theta2 - self.theta2), -self.max_velocity , self.max_velocity)

        threshold = 0.02

        if (
            abs(self.target_theta1 - self.theta1) < threshold
            and
            abs(self.target_theta2 - self.theta2) < threshold
         ):
             self.current_waypoint += 1
        
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
        