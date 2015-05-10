#!/usr/bin/env python

import roslib
roslib.load_manifest('localizer')
import rospy
import math
import udp
import roslib; roslib.load_manifest('lidar_translator')

from lidar_translator.msg import *
from std_msgs.msg import Int16MultiArray, String
from geometry_msgs.msg import Pose2D

def cm_to_in(cm):
    return (cm*0.393701)
def in_to_cm(inches):
    return inches/0.393701
def in_to_ft(inches):
    return inches/12.0
def ft_to_in(ft):
    return ft*12
def cm_to_ft(cm):
    return in_to_ft(cm_to_in(cm))
def ft_to_cm(ft):
    return in_to_cm(ft_to_in(ft))

class WhereAmI(object):
    def __init__(self, init_pose = None, confidence=100,
                    world_units = "ft", camera_units="cm", lidar_units="in",
                    pose_port = 60112, pose_ip = None):
        # if no pose is given, we don't know where we are
        if init_pose is None:
            self.pose = Pose2D()
            self.confidence = 0
        # if a pose is given, use the supplied confidence (100 = most confident)
        else:
            self.pose = init_pose
            self.confidence = confidence

        self.camera_x = 0
        self.camera_y = 0
        self.camera_theta = 0            
        self.last_camera = None
        self.first_lidar_x = 0
        self.current_lidar_x = 0
        self.current_lidar_theta = 0
        self.camera_time = None
        self.lidar_time = None
        self.camera_last_called = True
        self.orientation_multiplier = 0
        
        rospy.Subscriber('camera_location', Int16MultiArray, self.camera_cb)
        rospy.Subscriber('lidar_output', lidar_output, self.lidar_cb)
        self.pub = rospy.Publisher('where_am_i', Pose2D, queue_size=10)


        self.pose_port = pose_port
        if pose_ip is None:
            self.pose_ip = "255.255.255.255"
        else:
            self.pose_ip = pose_ip
    
    def run(self, node_name='where_am_i', anon=False):
        self.pose_sock = udp.udp_socket()
        rospy.init_node(node_name, anonymous=anon)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.check_confidence()
            rospy.loginfo("(%.2f, %.2f, %.2f) conf: %d" % (self.pose.x, self.pose.y, 
                                                            self.pose.theta, self.confidence))
            rate.sleep()
    
    # dummy function to do confidence-checking: older data is less reliable
    def check_confidence(self):
        if self.camera_time is not None or self.lidar_time is not None:
            self.confidence = 100
    
    # def 

    # combining data from all sources based on confidence, update position and publish
    def update_position(self):
        self.pose.x = self.camera_x + self.orientation_multiplier * (self.current_lidar_x - self.first_lidar_x)
        self.pose.y = self.camera_y
        self.pose.theta = self.camera_theta
        if not self.camera_last_called:
            self.pose.theta = self.current_lidar_theta + 90 + self.orientation_multiplier * 90
        self.pub.publish(self.pose)
        message = "({:.3f},{:.3f},{:.3f})".format(self.pose.x, self.pose.y, self.pose.theta)
        self.pose_sock.sendto(message, (self.pose_ip, self.pose_port))
    
    def camera_cb(self, msg):
        # do we need to do anything with the layout field?
        self.camera_x = cm_to_ft(msg.data[0])
        self.camera_y = cm_to_ft(msg.data[1])
        self.camera_theta = msg.data[2]
        if abs(self.camera_theta) > 90:
            self.orientation_multiplier = 1
        else:
            self.orientation_multiplier = -1
        self.first_lidar_x = self.current_lidar_x
        self.camera_time = rospy.get_rostime()
        self.camera_last_called = True
        self.update_position()

    def lidar_cb(self, msg):
        self.current_lidar_x = in_to_ft(msg.closest_obj)
        self.current_lidar_theta = msg.wall_angle
        if self.camera_last_called:
            self.camera_last_called = False #buffer before handling non-detecting camera
        else:
            self.update_position()
        return


if __name__=="__main__":
    try:
        node = WhereAmI() #TODO: work in recovering position from file?
        node.run()
    except rospy.ROSInterruptException:
        pass

