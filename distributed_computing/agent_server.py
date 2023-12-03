'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from jsonrpc import jsonrpcserver
import logging
import threading
from time import sleep


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()

        logging.basicConfig(level=logging.DEBUG)
        self.server = jsonrpcserver.Server()

        # Register JSON-RPC methods
        self.server.add_method(self.get_angle, name="get_angle")
        self.server.add_method(self.set_angle, name="set_angle")
        self.server.add_method(self.get_posture, name="get_posture")
        self.server.add_method(self.execute_keyframes, name="execute_keyframes")
        self.server.add_method(self.get_transform, name="get_transform")
        self.server.add_method(self.set_transform, name="set_transform")

        # Start the JSON-RPC server in a separate thread
        thread = threading.Thread(target=self.serve_forever)
        thread.start()
        print("Server thread started")
    
    def serve_forever(self):
        # Start serving JSON-RPC requests
        jsonrpcserver.serve_forever()

    
    def get_angle(self, joint_name):
        angle = self.perception.joint[joint_name]
        return joint_name + " angle = " + str("%.2f" % angle)

    def set_angle(self, joint_name, angle):
        self.target_joints[joint_name] = angle
        return {'message': 'set ' + joint_name + " angle to " + str(angle)}

    def get_posture(self):
        return {'message': "Posture = " + self.recognize_posture(self.perception)}

    def execute_keyframes(self, keyframes):
        self.keyframes = keyframes
        max_time = max(max(x) if isinstance(x, list) else x for x in keyframes[1])
        sleep(max_time)
        return {'message': "keyframe executed in " + str("%.2f" % max_time) + "s"}

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

