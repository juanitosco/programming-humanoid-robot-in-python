'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
import numpy as np
from numpy.matlib import matrix, identity

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE7
                        'RArm': ['RElbowYaw', 'RElbowRoll', 'RShoulderPitch', 'RShoulderRoll'],
                        'LArm': ['LElbowYaw', 'LShoulderPitch', 'LShoulderRoll', 'LElbowRoll'],
                        'RLeg': ['RKneePitch', 'RAnklePitch', 'RHipPitch', 'RHipYawPitch', 'RAnkleRoll', 'RHipRoll'],
                        'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnkleRoll', 'LAnklePitch'],
                        }
        
        self.joint_length ={'HeadYaw': [0.00, 0.00, 126.50], 
                            'HeadPitch': [0.00, 0.00, 0.00],
                            'LShoulderPitch': [0.00, 98.00, 100.00], 
                            'LShoulderRoll': [0.00, 0.00, 0.00],
                            'LElbowYaw': [105.00, 15.00, 0.00], 
                            'LElbowRoll': [0.00, 0.00, 0.00],
                            'RShoulderPitch': [0.00, -98.00, 100.00], 
                            'RShoulderRoll': [0.00, 0.00, 0.00],
                            'RElbowYaw': [105.00, -15.00, 0.00], 
                            'RElbowRoll': [0.00, 0.00, 0.00],
                            'LHipYawPitch': [0.00, 50.00, -85.00], 
                            'LHipRoll': [0.00, 0.00, 0.00],
                            'LHipPitch': [0.00, 0.00, 0.00], 
                            'LKneePitch': [0.00, 0.00, -100.00],
                            'LAnklePitch': [0.00, 0.00, -102.90], 
                            'LAnkleRoll': [0.00, 0.00, 0.00],
                            'RHipYawPitch': [0.00, -50.00, -85.00], 
                            'RHipRoll': [0.00, 0.00, 0.00],
                            'RHipPitch': [0.00, 0.00, 0.00],
                            'RKneePitch': [0.00, 0.00, -100.00], 
                            'RAnklePitch': [0.00, 0.00, -102.90],
                            'RAnkleRoll': [0.00, 0.00, 0.00]
                            }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)

        # YOUR CODE HERE

        s =np.sin(joint_angle)
        c =np.cos(joint_angle)

        if "Pitch" in joint_name:
            T = matrix([[ c, 0, s, 0]
                      ,[ 0, 1, 0, 0]
                      ,[-s, 0, c, 0]
                      ,[ 0, 0, 0, 1]])
        if "Yaw" in joint_name:
            T = matrix([[ c, s, 0, 0]
                      ,[-s, c, 0, 0]
                      ,[ 0, 0, 1, 0]
                      ,[ 0, 0, 0, 1]])
        if "Roll" in joint_name:
             T = matrix([[ 1, 0, 0, 0]
                      ,[ 0, c,-s, 0]
                      ,[ 0, s, c, 0]
                      ,[ 0, 0, 0, 1]])
        T[0,3] = self.joint_length[joint_name][0]
        T[1,3] = self.joint_length[joint_name][1]
        T[2,3] = self.joint_length[joint_name][2]


        
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T, Tl)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
