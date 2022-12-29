#!/usr/bin/env python3
from __future__ import print_function
from shutil import move

from matplotlib.lines import lineMarkers
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from nav_msgs.msg import Path


class cylinder_node:
    def __init__(self):
        rospy.init_node("simple_marker", anonymous=True)

    def interactive(self):
        self.server = InteractiveMarkerServer("simple_marker")
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "my_marker"
        int_marker.description = "Simple 1-DOF Control"

        # create a grey box marker
        # box_marker = Marker()
        # box_marker.type = Marker.CYLINDER
        # box_marker.scale.x = 1.0
        # box_marker.scale.y = 1.0
        # box_marker.scale.z = 0.01
        # box_marker.color.r = 0.0
        # box_marker.color.g = 0.5
        # box_marker.color.b = 0.5
        # box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        # box_control.always_visible = True
        # box_control.markers.append(box_marker)

        # add the control to the interactive marker
        int_marker.controls.append(box_control)

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        # add the control to the interactive marker
        # int_marker.controls.append(rotate_control)

        # write
        move_y_control = InteractiveMarkerControl()
        move_y_control.name = "move_y"
        move_y_control.orientation.w = 1
        move_y_control.orientation.x = 0
        move_y_control.orientation.y = 1
        move_y_control.orientation.z = 0
        # move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_y_control.orientation_mode = InteractiveMarkerControl.INHERIT
        # move_y_control.independent_marker_orientation = True
        # int_marker.controls.append(copy.deepcopy(move_y_control))

        move_y_control.markers.append(self.makeBox(int_marker))
        # move_y_control.always_visible =
        move_y_control.always_visible = False
        int_marker.controls.append(move_y_control)

        # write line
        # int2_marker = InteractiveMarker()
        # int2_marker.header.frame_id = "map"
        # int2_marker.name = "my2_marker"
        # int2_marker.description = "Line_strip"

        # int2_marker.controls.append(makeLine(int2_marker))

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # self.makeLine()

        # 'commit' changes and send to all clients
        self.server.applyChanges()

        # rospy.spin()

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


class visualization_node:
    def __init__(self):
        # rospy.init_node("simple_marker")
        self.pub_line_min_dist = rospy.Publisher(
            'simple_marker/line_min_dist', Marker, queue_size=1)

        # create an interactive marker server on the topic namespace simple_marker
        # self.server = InteractiveMarkerServer("simple_marker")

    def loop(self):
        self.makeLine()

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

    def makeLine(self):
        # int2_marker = InteractiveMarker()
        # int2_marker.header.frame_id = "map"
        # int2_marker.name = "my2_marker"
        # int2_marker.description = "Line_strip"

        line_marker = Marker()

        line_marker.header.frame_id = "map"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = line_marker.ADD
        # line_marker.name =

        line_marker.scale.x = 1.0
        line_marker.scale.y = 1.0
        line_marker.scale.z = 1.0

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
        # print(line_marker.points)

        first_line_point = Point()
        first_line_point.x = 0.0
        first_line_point.y = 0.0
        first_line_point.z = 0.0
        line_marker.points.append(first_line_point)

        second_line_point = Point()
        second_line_point.x = 1.0
        second_line_point.y = 1.0
        second_line_point.z = 0.0
        line_marker.points.append(second_line_point)

        # int2_marker.controls.append(line_marker)

        # pub_line_min_dist.publish(marker)
        # server.insert(int2_marker, processFeedback)
        self.pub_line_min_dist.publish(line_marker)
        # return line_marker


if __name__ == "__main__":
    rg = visualization_node()
    rd = cylinder_node()
    DURATION = 0.25
    r = rospy.Rate(1 / DURATION)
    # rd.interactive()
    while not rospy.is_shutdown():
        rd.interactive()
        rg.loop()
        r.sleep()
        # rospy.spin()
