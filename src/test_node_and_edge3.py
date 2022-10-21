#!/usr/bin/env python3
from shutil import move

from matplotlib.lines import lineMarkers
from py import process
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import concurrent.futures
import threading
import time
import rospy
import copy
import numpy as np
import math
import yaml
import re
from std_srvs.srv import Trigger

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from simple_marker import processFeedback
from visualization_msgs.msg import *
from nav_msgs.msg import Path

POINT_X = [0.0]*20
POINT_Y = [0.0]*20
POINT_Z = 0.0
N  = 2
BASE_TIME = time.time()
LINE_POINT_LIST = []
POINT_LIST = []

menu_handler = MenuHandler()

# print(POINT_X[1])

class cylinder_node:
    def __init__(self):
        rospy.init_node("simple_marker", anonymous=True)
        self.server = InteractiveMarkerServer("simple_marker")
        menu_handler.insert("Add point", callback=self.addpoint)
        menu_handler.insert("Initialize point", callback=self.init_point)
        sub_menu_handle = menu_handler.insert("Add line")
        menu_handler.insert("Set point1", parent=sub_menu_handle, callback=self.setpoint1)
        menu_handler.insert("Set point2", parent=sub_menu_handle, callback=self.setpoint2)
        # self.point_list = []
        self.point_num = 1
        self.OLD_TIME = 0
        self.ADD_TIME = 0
        self.point_num_for_set = 0
        self.point_num_for_set_x = 0
        self.point_num_for_set_y = 0
        self.init_x = 0
        self.init_y = 0

    def interactive(self):
        # print(time.time() - BASE_TIME)

        # for i in range(self.point_num):
        #     self.insert_point(i)

        self.insert_point(0, 0, 0)

        # self.server.applyChanges()
        # self.insert_point2(1)
        self.server.insert(self.int_marker, self.processFeedback)
        menu_handler.apply(self.server, self.int_marker.name)
        # self.server.applyChanges()

        # self.insert_point(1)
        # self.server.insert(self.int_marker, self.processFeedback)
        # menu_handler.apply(self.server, self.int_marker.name)

        self.server.applyChanges()

        rospy.spin()

    def insert_point(self, i, x, y):
        self.int_marker = InteractiveMarker()
        # self.int_marker = []
        self.int_marker.header.frame_id = "map"
        self.int_marker.name = "my_marker" + str(i)
        self.int_marker.description = "Simple 1-DOF Control_" + str(i)
        self.int_marker.pose.position.x = x
        self.int_marker.pose.position.y = y
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

        self.move_y_control.markers.append(self.makeText(self.int_marker, i))

        self.move_y_control.always_visible = False
        self.int_marker.controls.append(self.move_y_control)

        # self.points = [x, y, 0]
        self.points = {'POINT_[' + str(i) + ']': [x, y, 0]}
        POINT_LIST.append(self.points)

    def alignMarker(self, feedback):
        pose = feedback.pose

        pose.position.x = round(pose.position.x-0.5)+0.5
        pose.position.y = round(pose.position.y-0.5)+0.5

        rospy.loginfo(feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                    str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z))

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()
    
    def setpoint1(self, feedback):
        p = feedback.pose.position
        self.point_num_for_set = int(feedback.control_name)
        self.point_num_for_set_x = p.x
        self.point_num_for_set_y = p.y
        # print(p.x, p.y)

    def setpoint2(self, feedback):
        min_num_x = min_num_y = max_num_x = max_num_y = 0 
        p = feedback.pose.position
        end_point = int(feedback.control_name)
        if self.point_num_for_set > end_point:
            max_num = self.point_num_for_set
            max_num_x = self.point_num_for_set_x
            max_num_y = self.point_num_for_set_y

            min_num = end_point
            min_num_x = p.x
            min_num_y = p.y
        else:
            max_num = end_point
            max_num_x = p.x
            max_num_y = p.y

            min_num = self.point_num_for_set
            min_num_x = self.point_num_for_set_x
            min_num_y = self.point_num_for_set_y

        # print(p.x, p.y)
        # print(min_num_x, min_num_y, max_num_x, max_num_y)
        distance = self.calc_distance(min_num_x, min_num_y, max_num_x, max_num_y)
        self.line_point = {'POINT_[' + str(min_num) + ']': [min_num_x, min_num_y, 0], 'POINT_[' + str(max_num) + ']': [max_num_x, max_num_y, 0], 'DISTANCE': distance}
        LINE_POINT_LIST.append(self.line_point)
        # __ = self.calc_ang(min_num_x, min_num_y, max_num_x, max_num_y)
    
    def calc_distance(self, minx, miny, maxx, maxy):
        return (math.sqrt((maxx - minx)**2 + (maxy - miny)**2))

    def calc_ang(self, minx, miny, maxx, maxy):
        # x1 = x2 = y1 = 0
        # y2 = 1
        # a = np.array([0, 0])
        # b = np.array([0, 1])

        a = np.array([minx, miny])
        b = np.array([maxx, maxy])

        vec = b-a
        res = np.arctan2(vec[0], vec[1])
        print(res)
        ang = (res * 180) /  math.pi
        print(ang)
        return ang

    def addpoint(self, feedback):
        print("add point!")

        p = feedback.pose.position

        self.insert_point(self.point_num, p.x, p.y)

        self.server.insert(self.int_marker, self.processFeedback)
        menu_handler.apply(self.server, self.int_marker.name)

        self.server.applyChanges()

        self.point_num += 1
        # vars(self.server.marker_contexts['my_marker0']['last_feedback'])
        # tmep = vars(self.server.marker_contexts['my_marker0'])
        # print(temp['last_feedback'])
        # print(vars(self.server.pending_updates))
        # print(self.server.marker_contexts.keys())
        # test = list(self.server.marker_contexts.keys())
        # print(type(test))
        # print(test)
        # print(self.server.marker_contexts[test[0]])
        # vars(self.server.marker_contexts[test[0]])

        # print(type(self.server.marker_contexts[test[0]]))
        # print(self.server.marker_contexts['my_marker0']())

    def init_point(self, feedback):
        p = feedback.pose.position
        self.init_x = p.x
        self.init_y = p.y
        print("initialize point: " + str(self.init_x) + "," + str(self.init_y))

    def makeMenuMarker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = 0
        int_marker.scale = 1

        int_marker.name = "context_menu"
        int_marker.description = "Context Menu\n(Right Click)"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description = "Options"
        control.name = "menu_only_control"
        int_marker.controls.append(copy.deepcopy(control))

        # make one control showing a box
        marker = self.makeBox(int_marker)
        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, processFeedback)
        menu_handler.apply(self.server, int_marker.name)
        
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

        self.menu_control = InteractiveMarkerControl()
        self.menu_control.interaction_mode = InteractiveMarkerControl.MENU
        self.menu_control.description = "Options"
        self.menu_control.name = "menu_only_control"
        self.int_marker.controls.append(self.menu_control)

    def processFeedback(self, feedback):
        TIME = time.time()
        global POINT_X
        global POINT_Y
        global POINT_Z

        p = feedback.pose.position
        # print(feedback.marker_name + " is now at " +
        #       str(p.x) + ", " + str(p.y) + ", " + str(p.z))

        # print(self.ADD_TIME)
        # if self.ADD_TIME >= 0.5:
        if (feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP):
            # POINT_X[int(feedback.control_name)] = p.x
            # POINT_Y[int(feedback.control_name)] = p.y
            # POINT_Z = p.z
            # print(LINE_POINT_LIST)
            for i in range(len(LINE_POINT_LIST)):
                # print("POINT_" + str([int(feedback.control_name)]) in LINE_POINT_LIST[i])
                if ("POINT_" + str([int(feedback.control_name)]) in LINE_POINT_LIST[i]):
                    LINE_POINT_LIST[i]["POINT_" + str([int(feedback.control_name)])][0] = p.x
                    LINE_POINT_LIST[i]["POINT_" + str([int(feedback.control_name)])][1] = p.y
            for i in range(len(POINT_LIST)):
                if ("POINT_" + str([int(feedback.control_name)]) in POINT_LIST[i]):
                    POINT_LIST[i]["POINT_" + str([int(feedback.control_name)])][0] = p.x
                    POINT_LIST[i]["POINT_" + str([int(feedback.control_name)])][1] = p.y
                # POINT_Z = p.z
            # print("update")
            # self.ADD_TIME = 0
        # print(TIME - self.OLD_TIME)

        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id
        
        # print(feedback.control_name)
        # print(feedback.event_type)

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

        self.ADD_TIME += TIME
        self.OLD_TIME = TIME

        self.server.applyChanges()

    def makeText(self, msg, i):
        text_marker = Marker()

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.scale.x = 1.0
        text_marker.scale.y = 1.0
        text_marker.scale.z = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = str(i)

        return text_marker

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
        global LINE_POINT_LIST
        # rospy.init_node("simple_marker")
        self.pub_line_min_dist = rospy.Publisher(
            'simple_marker/line_min_dist', MarkerArray, queue_size=10)
        # self.feedback_sub = rospy.Subscriber(
            # "/simple_marker/feedback", InteractiveMarkerFeedback, self.feedback_callback, queue_size=3)

        # create an interactive marker server on the topic namespace simple_marker
        # self.server = InteractiveMarkerServer("simple_marker")

        self.srv = rospy.Service('/all_save', Trigger, self.save)

        self.list_point2 = np.array(
            [[POINT_X[0], POINT_Y[0], POINT_Z], [POINT_X[1], POINT_Y[1], POINT_Z]])

        self.list_point3 = np.array(
            [[POINT_X[3], POINT_Y[3], POINT_Z], [POINT_X[4], POINT_Y[4], POINT_Z]])

        # self.line_point11 = {'POINT_X[0]':POINT_X[0], 'POINT_Y[0]':POINT_Y[0], 'POINT_Z': POINT_Z}
        self.line_point11 = {'POINT_[0]': [POINT_X[0], POINT_Y[0], POINT_Z], 'POINT_[1]': [
            POINT_X[1], POINT_Y[1], POINT_Z]}
        self.line_point12 = {'POINT_[2]': [POINT_X[0], POINT_Y[0], POINT_Z], 'POINT_[3]': [
            POINT_X[1], POINT_Y[1], POINT_Z]}
        self.line_point13 = {'POINT_[1]': [POINT_X[0], POINT_Y[0], POINT_Z], 'POINT_[3]': [
            POINT_X[1], POINT_Y[1], POINT_Z]}


        self.line_point11 = {'POINT_[0]': [0, 0, POINT_Z], 'POINT_[1]': [
            0, 1, POINT_Z]}
        # print(self.line_point11)

        # LINE_POINT_LIST.append(self.list_point2)
        # LINE_POINT_LIST.append(self.list_point11)
        # LINE_POINT_LIST = [self.line_point11]
        LINE_POINT_LIST = []
        # LINE_POINT_LIST = [self.line_point11, self.line_point12]
        # LINE_POINT_LIST.append(self.line_point13)
        # print(LINE_POINT_LIST)
        # LINE_POINT_LIST.append(self.list_point3)

    def timer(self):
        rospy.Timer(rospy.Duration(2), self.loop)
        rospy.spin()

    def loop(self):
        # self.makeLine()
        # print("success")
        # rospy.sleep(2.0)
        self.markers()

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
        line_marker = Marker()

        line_marker.header.frame_id = "map"
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = line_marker.ADD

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
        # for i, (x, y, z,) in enumerate(list_point):
        #     line_point = Point()
        #     line_point.x = x
        #     line_point.y = y
        #     line_point.z = z
        #     line_marker.points.append(line_point)
        # print(list_point.values())
        # 
        
        # print(list_point.keys())
        # (point1, point2) = list_point.keys()
        # print(point1)
        # print(list_point)
        list_point_ = list_point.copy()

        # line_marker.text = str(list_point_['DISTANCE'])

        del list_point_['DISTANCE']

        for (x, y, z) in list_point_.values():
            # print(x, y, z)
            line_point = Point()
            line_point.x = x
            line_point.y = y
            line_point.z = z
            line_marker.points.append(line_point)
        return line_marker

        

    def markers(self):
        markers = MarkerArray()
        markers.markers = []

        list_point = np.array([[0.0, 0.0, 0.0], [POINT_X[1], POINT_Y[1], POINT_Z], [
                              1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        # list_point2 = np.array([[4.0, 5.0, 0.0], [POINT_X, POINT_Y, POINT_Z], [
        #                          3.0, 0.0, 0.0], [0.0, 3.0, 0.0]])
        # list_point = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        # list_point2 = np.array([[5.0, 5.0, 0.0], [6.0, 6.0, 0.0]])
        # list_point2 = np.array(
        #     [[POINT_X[0], POINT_Y[0], POINT_Z], [POINT_X[1], POINT_Y[1], POINT_Z]])

        # list_point3 = np.array(
        #     [[POINT_X[3], POINT_Y[3], POINT_Z], [POINT_X[4], POINT_Y[4], POINT_Z]])

        # LINE_POINT_LIST.append(list_point)
        # LINE_POINT_LIST.append(list_point3)

        # line = self.makeLine(list_point, id=1)
        # markers.markers.append(line)
        # print(LINE_POINT_LIST)

        for i in range(len(LINE_POINT_LIST)):
            line = self.makeLine(LINE_POINT_LIST[i], id=i)
            markers.markers.append(line)
        
        # self.write_line_yaml()
        # self.write_points_yaml()

        # print(LINE_POINT_LIST)
        # print(POINT_LIST)
        # line = self.makeLine(self.line_point11, id=1)
        # markers.markers.append(line)

        # line = self.makeLine(list_point2, id=2)
        # markers.markers.append(line)
        self.pub_line_min_dist.publish(markers)
    
    def save(self, data):
        self.write_line_yaml()
        self.write_points_yaml()

    def write_points_yaml(self):
        # print("write yaml!")
        with open('write_points.yaml', 'w') as f:
            data = {
                'make_points': {

                }
            }
            # data['setting']['list'] = [1, 2, 3]
            data["make_points"] = []
            # yaml.dump(data, f)
            for i in range(len(POINT_LIST)):
                # print(POINT_LIST[i].keys())
                point1 = list(POINT_LIST[i].keys())
                value1 = POINT_LIST[i][point1[0]]
                # list = {
                #     'id': i,
                #     str(point1): value1, 
                #     str(point2): value2
                # }  
                mylist = {
                    'id': i,
                    'position': value1
                }   
                data["make_points"].append(mylist)
            yaml.safe_dump(data, f, sort_keys=False)
    
    def write_line_yaml(self):
        # print("write yaml!")
        with open('write_line.yaml', 'w') as f:
            data = {
                'make_line': {

                }
            }
            # data['setting']['list'] = [1, 2, 3]
            data["make_line"] = []
            # yaml.dump(data, f)
            for i in range(len(LINE_POINT_LIST)):
                (point1, point2, distance) = LINE_POINT_LIST[i].keys()
                value1 = LINE_POINT_LIST[i][str(point1)]
                value2 = LINE_POINT_LIST[i][str(point2)]
                distance_ = LINE_POINT_LIST[i][str(distance)]
                # list = {
                #     'id': i,
                #     str(point1): value1, 
                #     str(point2): value2
                # }  
                mylist = {
                    'id': i,
                    'point1': {
                        'point_name': int(re.sub(r"\D", "", point1)),
                        'position': value1
                    }, 
                    'point2': {
                        'point_name': int(re.sub(r"\D", "", point2)),
                        'position': value2
                    },
                    'distance': distance_
                }   
                data["make_line"].append(mylist)
            yaml.safe_dump(data, f, sort_keys=False)
            # pass

if __name__ == "__main__":
    rg = visualization_node()
    rd = cylinder_node()
    t1 = threading.Thread(target=rd.interactive)
    # executor = concurrent.futures.ThreadPoolExecutor(max_workers=3)
    # t2 = threading.Thread(target=rg.timer)

    DURATION = 0.5

    r = rospy.Rate(1 / DURATION)
    # rd.interactive()
    t1.start()
    # executor.submit(rd.interactive)
    # t2.start()
    # rd.interactive()
    while not rospy.is_shutdown():
        # rd.interactive()
        rg.loop()
        r.sleep()
        