from morse.builder import *

# Append ATRV robot to the scene
robot = ATRV()

# Append an actuator
motion = MotionVW()
motion.translate(z=0.3)
robot.append(motion)

# Append an Odometry sensor
odometry = Odometry()
odometry.translate(z=0.73)
robot.append(odometry)

# Append a depth camera
camera = DepthCamera()
robot.append(camera)
camera.translate(x=.25,z=.85)

robot.add_default_interface('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
env.show_framerate()
camera.profile()
odometry.profile()
