#!/usr/bin/env python
import rosbag
import sys
from tf import transformations
import numpy as np

if len(sys.argv) != 14:
    print "Usage:", sys.argv[0], "[BAG IN], [BAG OUT] [SPIN FRAME] [LASER FRAME] [ROLL] [PITCH] [YAW] [X] [Y] [Z] [KROLL] [KPITCH] [KYAW]"
    exit(0)

bag_in_path = sys.argv[1]
bag_out_path = sys.argv[2]
spin_frame = sys.argv[3]
laser_frame = sys.argv[4]
roll = float(sys.argv[5])
pitch = float(sys.argv[6])
yaw = float(sys.argv[7])
x = float(sys.argv[8])
y = float(sys.argv[9])
z = float(sys.argv[10])
kroll = float(sys.argv[11])
kpitch = float(sys.argv[12])
kyaw = float(sys.argv[13])

H = transformations.translation_matrix((x, y, z)).dot(transformations.euler_matrix(roll, pitch, yaw))
K = transformations.euler_matrix(kroll, kpitch, kyaw)

HK = K.dot(H).dot(transformations.inverse_matrix(K))

counter = 0
with rosbag.Bag(bag_out_path, 'w') as bag_out:
    for topic, msg, t in rosbag.Bag(bag_in_path).read_messages():
        if topic == "/tf" and msg.transforms:
            for g_msg in msg.transforms:
                if g_msg.header.frame_id == spin_frame and g_msg.child_frame_id == laser_frame:
                    transform = g_msg.transform

                    Vq = np.array([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
                    V = transformations.translation_matrix((transform.translation.x, transform.translation.y, transform.translation.z)).dot(transformations.quaternion_matrix(Vq))
                    T = HK.dot(V)

                    transform.translation.x = T[0, 3]
                    transform.translation.y = T[1, 3]
                    transform.translation.z = T[2, 3]

                    Tq = transformations.quaternion_from_matrix(T)
                    transform.rotation.x = Tq[0]
                    transform.rotation.y = Tq[1]
                    transform.rotation.z = Tq[2]
                    transform.rotation.w = Tq[3]
                    counter += 1
            bag_out.write(topic, msg, t)
        else:
            bag_out.write(topic, msg, t)

if counter == 0:
    print "[WARNING] Didn't find any transforms to replace. Are you sure that the frame names are correct and that they" \
          "are directly connected?"
else:
    print "Replaced", counter, "messages."