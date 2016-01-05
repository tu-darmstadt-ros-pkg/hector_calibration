#!/usr/bin/env python
import rosbag
import sys
from tf import transformations
import numpy as np

if len(sys.argv) != 11:
    print "Usage:", sys.argv[0], "[BAG IN], [BAG OUT] [ACTUATOR FRAME] [LASER FRAME] [ROLL] [PITCH] [YAW] [X] [Y] [Z]"
    exit(0)

bag_in_path = sys.argv[1]
bag_out_path = sys.argv[2]
actuator_frame = sys.argv[3]
laser_frame = sys.argv[4]
roll = float(sys.argv[5])
pitch = float(sys.argv[6])
yaw = float(sys.argv[7])
x = float(sys.argv[8])
y = float(sys.argv[9])
z = float(sys.argv[10])

qx = transformations.quaternion_about_axis(roll, (1, 0, 0))
qy = transformations.quaternion_about_axis(pitch, (0, 1, 0))
qz = transformations.quaternion_about_axis(yaw, (0, 0, 1))
q = transformations.quaternion_multiply(qz, qy)
q = transformations.quaternion_multiply(q, qx)

counter = 0
with rosbag.Bag(bag_out_path, 'w') as bag_out:
    for topic, msg, t in rosbag.Bag(bag_in_path).read_messages():
        if topic == "/tf" and msg.transforms:
            for g_msg in msg.transforms:
                if g_msg.header.frame_id == actuator_frame and g_msg.child_frame_id == laser_frame:
                    transform = g_msg.transform
                    transform.translation.x += x
                    transform.translation.y += y
                    transform.translation.z += z

                    old_q = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
                    new_q = transformations.quaternion_multiply(old_q, q)
                    transform.rotation.x = new_q[0]
                    transform.rotation.y = new_q[1]
                    transform.rotation.z = new_q[2]
                    transform.rotation.w = new_q[3]
                    counter += 1
            bag_out.write(topic, msg, t)
        else:
            bag_out.write(topic, msg, t)

if counter == 0:
    print "[WARNING] Didn't find any transforms to replace. Are you sure that the frame names are correct and that they" \
          "are directly connected?"
else:

    print "Replaced", counter, "messages."