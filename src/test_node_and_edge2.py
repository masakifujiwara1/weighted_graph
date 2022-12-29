#!/usr/bin/env python3
from __future__ import print_function
from shutil import move

from matplotlib.lines import lineMarkers
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import threading
import time
import rospy
import copy
import numpy as np

from interactive_markers.interactive_marker_server import *
from simple_marker import processFeedback
from visualization_msgs.msg import *
from nav_msgs.msg import Path

POINT_X = 0
POINT_Y = 0
POINT_Z = 0
N  = 2

class cylinder_node:
    def __init__(self):
        rospy.init_node("simple_marker", anonymous=True)
        self.server = InteractiveMarkerServer("simple_marker")
        # self.int_marker = InteractiveMarker()
        # self.int_marker.header.frame_id = "map"
        # self.int_marker.name = "my_marker"
        # self.int_marker.description = "Simple 1-DOF Control"
        # self.move_y_control = InteractiveMarkerControl()
        # self.makeInteractive()
        # self.root_marker = []
        # self.interactive()
        # self.insert_point()

    def interactive(self):
        # print(self.root_marker[0])
        # self.server = InteractiveMarkerServer("simple_marker")
        # for i in range(N):
        #     self.server.insert(self.root_marker[i], self.processFeedback)
        self.insert_point(0)
        # self.server.applyChanges()
        # self.insert_point2(1)
        self.server.insert(self.int_marker, self.processFeedback)
        # self.server.applyChanges()

        self.insert_point(1)
        self.server.insert(self.int_marker, self.processFeedback)
            # self.server.applyChanges()

        # self.server.

        # self.makeLine()
        # for i in range(N):
        # control = self.makeInteractive("move_" + str(i))
        # self.int_marker.controls.append(control)
        # print(self.root_marker)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

        rospy.spin()

    def insert_point(self, i):
    
        # self.int_marker = InteractiveMarker()
        # self.int_marker.header.frame_id = "map"
        # self.int_marker.name = "my_marker"
        # self.int_marker.description = "Simple 1-DOF Control_" + str(i)
        self.int_marker = InteractiveMarker()
        # self.int_marker = []
        self.int_marker.header.frame_id = "map"
        self.int_marker.name = "my_marker" + str(i)
        self.int_marker.description = "Simple 1-DOF Control_" + str(i)
        self.int_marker.pose.position.x = i
        self.int_marker.pose.position.y = i
        # self.root_marker.append(self.int_marker)
        # write
        self.move_y_control = InteractiveMarkerControl()
        # self.move_y_control.description = str
        self.move_y_control.name = str(i)
        self.move_y_control.orientation.w = 1
        self.move_y_control.orientation.x = 0
        self.move_y_control.orientation.y = 1
        self.move_y_control.orientation.z = 0
        self.normalizeQuaternion(self.move_y_control.orientation)
        self.move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.move_y_control.orientation_mode = InteractiveMarkerControl.INHERIT

        # self.int_marker.controls.append(copy.deepcopy(self.move_y_control))

        self.move_y_control.markers.append(self.makeBox(self.int_marker))
        self.move_y_control.always_visible = False
        self.int_marker.controls.append(self.move_y_control)
        # self.server.insert(self.int_marker, self.processFeedback)
        # self.server.setCallback(self.int_marker.name, self.alignMarker,
        #                    InteractiveMarkerFeedback.POSE_UPDATE)

    def insert_point2(self, i):

        # self.int_marker = InteractiveMarker()
        # self.int_marker.header.frame_id = "map"
        # self.int_marker.name = "my_marker"
        # self.int_marker.description = "Simple 1-DOF Control_" + str(i)
        self.int_marker2 = InteractiveMarker()
        # self.int_marker = []
        self.int_marker2.header.frame_id = "map"
        self.int_marker2.name = "my_marker"
        self.int_marker2.description = "Simple 1-DOF Control_" + str(i)
        self.int_marker2.pose.position.x = i
        self.int_marker2.pose.position.y = i
        # self.root_marker.append(self.int_marker)
        # write
        self.move_y_control = InteractiveMarkerControl()
        # self.move_y_control.description = str
        self.move_y_control.name = str(i)
        self.move_y_control.orientation.w = 1
        self.move_y_control.orientation.x = 0
        self.move_y_control.orientation.y = 1
        self.move_y_control.orientation.z = 0
        self.normalizeQuaternion(self.move_y_control.orientation)
        self.move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.move_y_control.orientation_mode = InteractiveMarkerControl.INHERIT

        # self.int_marker2.controls.append(copy.deepcopy(self.move_y_control))

        self.move_y_control.markers.append(self.makeBox(self.int_marker2))
        self.move_y_control.always_visible = False
        self.int_marker2.controls.append(self.move_y_control)
        # self.server.insert(self.int_marker2,  self.processFeedback)

    def alignMarker(self, feedback):
        pose = feedback.pose

        pose.position.x = round(pose.position.x-0.5)+0.5
        pose.position.y = round(pose.position.y-0.5)+0.5

        rospy.loginfo(feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                    str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z))

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()

    
    def normalizeQuaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
            quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def makeInteractive(self, str):
        # write
        self.move_y_control = InteractiveMarkerControl()
        # self.move_y_control.description = str
        self.move_y_control.name = str
        self.move_y_control.orientation.w = 1
        self.move_y_control.orientation.x = 0
        self.move_y_control.orientation.y = 1
        self.move_y_control.orientation.z = 0
        self.normalizeQuaternion(self.move_y_control.orientation)
        self.move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.move_y_control.orientation_mode = InteractiveMarkerControl.INHERIT

        self.move_y_control.markers.append(self.makeBox(self.int_marker))
        self.move_y_control.always_visible = False
        self.int_marker.controls.append(self.move_y_control)

    def processFeedback(self, feedback):
        global POINT_X
        global POINT_Y
        global POINT_Z

        p = feedback.pose.position
        # print(feedback.marker_name + " is now at " +
        #       str(p.x) + ", " + str(p.y) + ", " + str(p.z))

        POINT_X = p.x
        POINT_Y = p.y
        POINT_Z = p.z

        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(s + ": button click" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " +
                        str(feedback.menu_entry_id) + " clicked" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(s + ": pose changed")
    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()

    def processFeedback1(self, feedback):
        global POINT_X
        global POINT_Y
        global POINT_Z

        p = feedback.pose.position
        # print(feedback.marker_name + " is now at " +
        #       str(p.x) + ", " + str(p.y) + ", " + str(p.z))

        POINT_X = p.x
        POINT_Y = p.y
        POINT_Z = p.z

        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo(s + ": button click" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " +
                        str(feedback.menu_entry_id) + " clicked" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(s + ": pose changed")
    # TODO
    #          << "\nposition = "
    #          << feedback.pose.position.x
    #          << ", " << feedback.pose.position.y
    #          << ", " << feedback.pose.position.z
    #          << "\norientation = "
    #          << feedback.pose.orientation.w
    #          << ", " << feedback.pose.orientation.x
    #          << ", " << feedback.pose.orientation.y
    #          << ", " << feedback.pose.orientation.z
    #          << "\nframe: " << feedback.header.frame_id
    #          << " time: " << feedback.header.stamp.sec << "sec, "
    #          << feedback.header.stamp.nsec << " nsec" )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo(s + ": mouse down" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(s + ": mouse up" + mp + ".")
        self.server.applyChanges()

    def makeBox(self, msg):
        box_marker = Marker()

        box_marker.type = Marker.CYLINDER
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 0.01
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.6

        return box_marker


class visualization_node:
    def __init__(self):
        # rospy.init_node("simple_marker")
        self.pub_line_min_dist = rospy.Publisher(
            'simple_marker/line_min_dist', MarkerArray, queue_size=1)
        self.feedback_sub = rospy.Subscriber(
            "/simple_marker/feedback", InteractiveMarkerFeedback, self.feedback_callback, queue_size=3)

        # create an interactive marker server on the topic namespace simple_marker
        # self.server = InteractiveMarkerServer("simple_marker")

    def timer(self):
        rospy.Timer(rospy.Duration(2), self.loop)
        rospy.spin()

    def loop(self):
        # self.makeLine()
        # print("success")
        # rospy.sleep(2.0)
        self.markers()

    def feedback_callback(self, data):
        pass
    #     global POINT_X
    #     global POINT_Y
    #     global POINT_Z
    #     POINT_X = data.pose.position.x
    #     POINT_Y = data.pose.position.y
    #     POINT_Z = data.pose.position.z
        # print(POINT_X)

    def processFeedback(self, feedback):
        p = feedback.pose.position
        print(feedback.marker_name + " is now at " +
              str(p.x) + ", " + str(p.y) + ", " + str(p.z))

    def makeBox(self, msg):
        box_marker = Marker()

        box_marker.type = Marker.CYLINDER
        box_marker.scale.x = 1.0
        box_marker.scale.y = 1.0
        box_marker.scale.z = 0.01
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.6

        return box_marker

    def makeLine(self, list_point, id = 1):
        # int2_marker = InteractiveMarker()
        # int2_marker.header.frame_id = "map"
        # int2_marker.name = "my2_marker"
        # int2_marker.description = "Line_strip"

        # markers = MarkerArray()
        line_marker = Marker()

        line_marker.header.frame_id = "map"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = line_marker.ADD
        # line_marker.name =

        line_marker.scale.x = 0.03
        line_marker.scale.y = 0.03
        line_marker.scale.z = 0.03

        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0

        line_marker.pose.orientation.x = 0.0
        line_marker.pose.orientation.y = 0.0
        line_marker.pose.orientation.z = 0.0
        line_marker.pose.orientation.w = 1.0

        line_marker.pose.position.x = 0.0
        line_marker.pose.position.y = 0.0
        line_marker.pose.position.z = 0.0

        line_marker.points = []
        # markers.markers = []

        line_marker.id = id
        # list_point = np.array([[0.0, 0.0, 0.0], [POINT_X, POINT_Y, POINT_Z], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        for i, (x, y, z) in enumerate(list_point):
            line_point = Point()
            line_point.x = x
            line_point.y = y
            line_point.z = z
            line_marker.points.append(line_point)
        
        # markers.markers.append(line_marker)
        # print(markers)

        # line_marker.points = []
        # line_marker.points.clear()
        # line_marker.id = 2
        # list_point = np.array([[4.0, 5.0, 0.0], [POINT_X, POINT_Y, POINT_Z], [
        #                      3.0, 0.0, 0.0], [0.0, 3.0, 0.0]])
        # for i, (x, y, z) in enumerate(list_point):
        #     line_point = Point()
        #     line_point.x = x
        #     line_point.y = y
        #     line_point.z = z
        #     line_marker.points.append(line_point)

        # markers.markers.append(line_marker)
        # print(markers)
        # int2_marker.controls.append(line_marker)

        # pub_line_min_dist.publish(marker)
        # server.insert(int2_marker, processFeedback)
        # self.pub_line_min_dist.publish(line_marker)
        # self.pub_line_min_dist.publish(markers)
        # print("success")
        # return line_marker
        return line_marker

    def markers(self):
        markers = MarkerArray()
        markers.markers = []

        # list_point = np.array([[0.0, 0.0, 0.0], [POINT_X, POINT_Y, POINT_Z], [
        #                       1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        # list_point2 = np.array([[4.0, 5.0, 0.0], [POINT_X, POINT_Y, POINT_Z], [
        #                          3.0, 0.0, 0.0], [0.0, 3.0, 0.0]])
        list_point = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        list_point2 = np.array([[5.0, 5.0, 0.0], [6.0, 6.0, 0.0]])
        line = self.makeLine(list_point, id=1)
        markers.markers.append(line)
        line = self.makeLine(list_point2, id=2)
        markers.markers.append(line)
        self.pub_line_min_dist.publish(markers)
if __name__ == "__main__":
    rg = visualization_node()
    rd = cylinder_node()
    t1 = threading.Thread(target=rd.interactive)
    # t2 = threading.Thread(target=rg.timer)

    DURATION = 0.25
    r = rospy.Rate(1 / DURATION)
    # rd.interactive()
    t1.start()
    # t2.start()
    while not rospy.is_shutdown():
        # rd.interactive()
        rg.loop()
        r.sleep()
        