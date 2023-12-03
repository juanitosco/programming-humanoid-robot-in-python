'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
from threading import Thread
from jsonrpc import jsonrpcclient

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        process = Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        process.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        process = Thread(target=self.proxy.set_transform, args=[effector_name, transform])
        process.start()

    

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)

    def get_angle(self, joint_name):
        '''Get sensor value of given joint'''
        response = jsonrpcclient.request("http://localhost:9999", "get_angle", joint_name)
        return response.result

    def set_angle(self, joint_name, angle):
        '''Set target angle of joint for PID controller'''
        response = jsonrpcclient.request("http://localhost:9999", "set_angle", joint_name, angle)
        return response.result

    def get_posture(self):
        '''Return current posture of robot'''
        response = jsonrpcclient.request("http://localhost:9999", "get_posture")
        return response.result

    def execute_keyframes(self, keyframes):
        '''Execute keyframes, blocking call'''
        response = jsonrpcclient.request("http://localhost:9999", "execute_keyframes", keyframes)
        return response.result

    def get_transform(self, name):
        '''Get transform with given name'''
        response = jsonrpcclient.request("http://localhost:9999", "get_transform", name)
        return response.result

    def set_transform(self, effector_name, transform):
        '''Solve the inverse kinematics and control joints using the results'''
        response = jsonrpcclient.request("http://localhost:9999", "set_transform", effector_name, transform)
        return response.result


if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE


