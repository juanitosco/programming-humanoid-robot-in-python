'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        # YOUR CODE HERE
        
        for chain in self.chains:
            for joint in self.chains[chain]:
                joint_angles[joint] = self.perception.joint[joint]

        target = np.array([self.from_trans(transform)]).T
        
        # Gradient Descent parameters
        learning_rate = 0.1
        max_iterations = 1000
        epsilon = 1e-4

        for iteration in range(max_iterations):
            self.forward_kinematics(joint_angles)

            T = [0] * len(self.chains[effector_name])
            for i, name in enumerate(self.chains[effector_name]):
                T[i] = self.transforms[name]

            Te = np.array([self.from_trans(T[-1])])
            e = target - Te

            # Compute Jacobian matrix
            J = np.zeros((6, len(self.chains[effector_name])))
            for i, name in enumerate(self.chains[effector_name]):
                J[:, i] = (self.from_trans(T[i]) - self.from_trans(T[-1])).T

            J[-1, :] = 1

            # Gradient Descent Update
            d_theta = learning_rate * np.dot(np.linalg.pinv(J), e.T)

            for i, name in enumerate(self.chains[effector_name]):
                joint_angles[name] += np.asarray(d_theta.T)[0][i]

            # Convergence Check
            if np.linalg.norm(d_theta) < epsilon:
                break

        
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)

        names = self.chains[effector_name]
        times = [[0, 5]] * len(names)
        keys = []
        for i, name in enumerate(names):
            keys.insert(i, [[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])

        self.keyframes = (names, times, keys)
    
    
    
        
    
    def from_trans(self, T):
        # return x,y,z
        
        x, y, z = T[3, 0], T[3, 1], T[3, 2]
        
        angle_x, angle_y, angle_z = 0, 0, 0

        if T[0, 0] == 1:
            angle_x = np.arctan2(T[2, 1], T[1, 1])
        elif T[1, 1] == 1:
            angle_y = np.arctan2(T[0, 2], T[0, 0])
        elif T[2, 2] == 1:
            angle_z = np.arctan2(T[1, 0], T[0, 0])
        result = np.array([x, y, z, angle_x, angle_y, angle_z])
        return result



if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
