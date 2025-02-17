'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''

import os
from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from joblib import load
import pickle
ROBOT_POSE_CLF = 'robot_pose.joblib'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        
        self.dir = os.path.dirname(__file__) #absolute dir the script is in
        robot_pose_pkl_path = os.path.join(self.dir, ROBOT_POSE_CLF)
        self.posture_classifier = load(robot_pose_pkl_path) 
    
    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        
        # the features are['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'AngleX', 'AngleY']
        
        state = [
            [perception.joint.get('LHipYawPitch'), perception.joint.get('LHipRoll'), perception.joint.get('LHipPitch'),
             perception.joint.get('LKneePitch'), perception.joint.get('RHipYawPitch'), perception.joint.get('RHipRoll'),
             perception.joint.get('RHipPitch'), perception.joint.get('RKneePitch'), perception.imu[0],
             perception.imu[1]]]
        posture = self.posture_classifier.predict(state)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
