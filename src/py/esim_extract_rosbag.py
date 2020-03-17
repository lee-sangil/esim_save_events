#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 Sangil Lee

"""
Extract topics from a rosbag.
"""

import os
import argparse
import numpy as np
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import IPython

bridge = CvBridge()

def main():
    """
	Extract a topic from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("base_dir", nargs='?', default="./dataset", help="Output directory.")
    args = parser.parse_args()
    bag = rosbag.Bag(args.bag_file, "r")

    print "Extract topics from %s into %s" %(args.bag_file, args.base_dir)

    text_image = open(os.path.join(args.base_dir,"image.txt"), 'w')
    text_depth = open(os.path.join(args.base_dir,"depth.txt"), 'w')
    text_flow = open(os.path.join(args.base_dir,"flow.txt"), 'w')
    text_event = open(os.path.join(args.base_dir,"event.txt"), 'w')
    text_imu = open(os.path.join(args.base_dir,"imu.txt"), 'w')
    text_pose = open(os.path.join(args.base_dir,"pose.txt"), 'w')

    text_image.write("# timestamp[s] filename\n")
    text_depth.write("# timestamp[s] filename\n")
    text_flow.write("# timestamp[s] filename min_vel max_vel\n")
    text_event.write("# timestamp[s] x y polarity\n")
    text_imu.write("# timestamp[s] ax ay az wz wy wz\n")
    text_pose.write("# timestamp[s] x y z qx qy qz qw\n")

    if not os.path.exists(os.path.join(args.base_dir,"image")):
        os.makedirs(os.path.join(args.base_dir,"image"), mode=0o777)
    if not os.path.exists(os.path.join(args.base_dir,"depth")):
        os.makedirs(os.path.join(args.base_dir,"depth"), mode=0o777)
    if not os.path.exists(os.path.join(args.base_dir,"flow")):
        os.makedirs(os.path.join(args.base_dir,"flow"), mode=0o777)
	
    for topic, msg, t in bag.read_messages(topics=["/cam0/image_raw", "/cam0/depthmap", "/cam0/events", "/cam0/optic_flow", "/cam0/pose", "/imu"]):

        if topic == "/cam0/image_raw":
            save_image(msg, t, args.base_dir, "image", text_image)
        elif topic == "/cam0/depthmap":
            save_image(msg, t, args.base_dir, "depth", text_depth)
        elif topic == "/cam0/optic_flow":
            save_flow(msg, t, args.base_dir, "flow", text_flow)
        elif topic == "/cam0/events":
            save_event(msg, text_event)
        elif topic == "/imu":
            save_imu(msg, t, text_imu)
        elif topic == "/cam0/pose":
            save_pose(msg, t, text_pose)

        print "\rTime passed: %i.%09i [s]" %(t.secs, t.nsecs),

    text_image.close()
    text_depth.close()
    text_flow.close()
    text_event.close()
    text_imu.close()
    text_pose.close()
    bag.close()

    return

def save_image(msg, t, base_dir, output_dir, text):
    """
    save image into output directory
    """
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    filename = os.path.join(output_dir, "%i.%09i.png" %(t.secs, t.nsecs))

    cv2.imwrite(os.path.join(base_dir, filename), cv_img)
    text.write("%i.%09i\t%s\n" %( t.secs, t.nsecs, filename ))

def save_flow(msg, t, base_dir, output_dir, text):
    """
    save flow into output directory
    """
    image, min_vel, max_vel = convertColorCodedFlow(msg.flow_x, msg.flow_y, msg.height, msg.width)
    filename = os.path.join(output_dir, "%i.%09i.png" %(t.secs, t.nsecs))

    cv2.imwrite(os.path.join(base_dir, filename), image)
    text.write("%i.%09i\t%s\t%f\t%f\n" %( t.secs, t.nsecs, filename, min_vel, max_vel ))

def save_event(msg, text):
    """
    save events into output directory
    """
    for e in msg.events:
        text.write("%i.%09i\t%i\t%i\t%i\n" %( e.ts.secs, e.ts.nsecs, e.x, e.y, e.polarity+0))

def save_imu(msg, t, text):
    """
    save imu into output directory
    """
    text.write("%i.%09i\t%f\t%f\t%f\t%f\t%f\t%f\n" 
            %( t.secs, t.nsecs, 
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

def save_pose(msg, t, text):
    """
    save pose into output directory
    """
    text.write("%i.%09i\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" 
            %( t.secs, t.nsecs,
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

def convertColorCodedFlow(flow_x, flow_y, height, width):
    magnitude, angle = cv2.cartToPolar(flow_x, flow_y)
    magnitude = magnitude.reshape(height, width)
    angle = angle.reshape(height, width)

    hsv = np.zeros((height,width,3), dtype=np.uint8)
    hsv[...,1] = 255
    hsv[...,0] = 0.5 * angle * 180 / np.pi
    hsv[...,2] = cv2.normalize(magnitude,None,0,255,cv2.NORM_MINMAX)
#    hsv[...,2] = np.minimum(0.5 * magnitude, 255)

    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    return image, np.min(magnitude), np.max(magnitude)

if __name__ == '__main__':
    main()
