#!/usr/bin/env python

import roslib
roslib.load_manifest('localizer')
import rospy
import math
import udp

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
    unit_conversion = {("in","ft") : in_to_ft, ("cm", "ft") : cm_to_ft}
    def __init__(self, init_pose = None, confidence=100,
                    world_units = "ft", camera_units="ft", lidar_units="in",
                    pose_port = 60112, pose_ip = None):
        # if no pose is given, we don't know where we are
        if init_pose is None:
            self.pose = Pose2D()
            self.confidence = 0
        # if a pose is given, use the supplied confidence (100 = most confident)
        else:
            self.pose = init_pose
            self.confidence = confidence
            
        self.last_camera = None
        self.last_lidar = None
        
        rospy.Subscriber('camera_location', Int16MultiArray, self.camera_cb)
        self.pub = rospy.Publisher('where_am_i', Pose2D, queue_size=10)

        self.pose_port = pose_port
        if pose_ip is None:
            self.pose_ip = "255.255.255.255"
        else:
            self.pose_ip = pose_ip
        
        self.world_units = world_units
        self.camera_units = camera_units
        self.lidar_units = lidar_units
    
    def run(self, node_name='where_am_i', anon=False):
        self.pose_sock = udp.udp_socket()
        rospy.init_node(node_name, anonymous=anon)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.check_confidence()
            message = self.pose_string()
            rospy.loginfo("%s conf: %d" % (message, self.confidence))
            self.pose_sock.sendto(message, (self.pose_ip, self.pose_port))
            rate.sleep()
    
    # dummy function to do confidence-checking: older data is less reliable
    def check_confidence(self):
        if self.last_camera is not None or self.last_lidar is not None:
            self.confidence = 100
    
    def pose_string(self):
        return "({:.3f},{:.3f},{:.3f})".format(self.pose.x, self.pose.y, self.pose.theta)

    # combining data from all sources based on confidence, update position and publish
    def update_position(self):
        self.pose.x = self.camera_x
        self.pose.y = self.camera_y
        self.pose.theta = self.camera_theta
        self.pub.publish(self.pose)
        message = self.pose_string()
        # moved to WhereAmI.run() to fix myRIO connection issue
        # self.pose_sock.sendto(message, (self.pose_ip, self.pose_port))
    
    def camera_cb(self, msg):
        # do we need to do anything with the layout field?
        if (self.world_units == self.camera_units):
            self.camera_x = msg.data[0]
            self.camera_y = msg.data[1]
        else:
            self.camera_x = WhereAmI.unit_conversion.get((self.camera_units, self.world_units))(msg.data[0])
            self.camera_y = WhereAmI.unit_conversion.get((self.camera_units, self.world_units))(msg.data[1])
        self.camera_theta = msg.data[2]
        self.last_camera = rospy.get_rostime()
        self.update_position()

if __name__=="__main__":
    try:
        node = WhereAmI() #TODO: work in recovering position from file?
        node.run()
    except rospy.ROSInterruptException:
        pass
