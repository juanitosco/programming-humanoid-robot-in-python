'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import matplotlib.pyplot as plt
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        current_time = perception.time
        names, times, keys = keyframes
        
        for i in range(len(names)):
            for j in range(len(times[i]) - 1):
                if times[i][j] <= current_time < times[i][j + 1]:
                    t = (current_time - times[i][j]) / (times[i][j + 1] - times[i][j])
                    p0 = keys[i][j][0]
                    p1 = keys[i][j][1][2]  
                    p2 = keys[i][j + 1][1][0]  
                    p3 = keys[i][j + 1][0]

                    target_joints[names[i]] = (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3

# Generate the Bezier curve points

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()


    time_values = np.linspace(0, 0.1, 100)  # Example time values from 0 to 3 seconds

# Calculate angles using the interpolation function
angle_values = {}
for joint_name in hello():
    angle_values[joint_name] = [AngleInterpolationAgent.angle_interpolation(hello, t) for t in time_values]

# Plot the results for each joint
plt.figure()
for joint_name in hello:
    plt.plot(time_values, angle_values[joint_name], label=joint_name)

plt.xlabel("Time (s)")
plt.ylabel("Angle")
plt.title("Interpolation of Angles Over Time")
plt.legend()
plt.grid()
plt.show()
