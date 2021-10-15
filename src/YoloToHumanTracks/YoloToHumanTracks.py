#!/usr/bin/env python

import copy
import sys
import numpy as np
import ros_numpy
from math import fabs
from statistics import mean

import rospy
import roslib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

NODE_NAME = 'YoloToHumanTracks'
roslib.load_manifest(NODE_NAME)
from amrl_msgs.msg import HumanStateMsg
from amrl_msgs.msg import HumanStateArrayMsg

global currentDepth
detectionThreshold = 0.45
distThreshold = 0.5
id_counter = 0
ageThreshold = 5
minTrackLength = 5
maxTrackLength = 20
tracks = {}
trackAge = {}
trackTimes = {}
observation = []
prediction = []
depthInit = False
marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray)
human_pub = rospy.Publisher("human_states", HumanStateArrayMsg)
bridge = CvBridge()
# Intrinsic Matrix Parameters (TODO load from file)
# Color Inrinsics
S = 0 # Skew
cx = 642.437622
cy = 363.448151
fx = 612.269653
fy = 612.225464
k1 = 0.472824
k2 = -2.599632
k3 = 1.490216
k4 = 0.351198
k5 = -2.421776
k6 = 1.417328
codx = 0.000000
cody = 0.000000
p2 = -0.000178
p1 = 0.000012

def ConvertFromDepth(u, v):
    global currentDepth
    # Depth is in mm from camera
    depth = currentDepth[v][u]
    z = depth
    z = z / 1000.0
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return z, x, y

def ConvertFromDepthSquare(u, v, size):
    upperLimit = 50
    depthList = []
    yList = []
    for i in range(u-size, u+size):
        for j in range(v-size, v + size):
            x, y, z = ConvertFromDepth(i, j)
            if (x > 0.0 and x < upperLimit):
                depthList.append(x)
                yList.append(y)
    x = 0
    y = 0
    if (len(depthList) > 0):
        x = mean(depthList)
    if (len(yList) > 0):
        y = mean(yList)
    return x, y, 0

def DrawHumans():
    global prediction
    markerArray = MarkerArray()
    for human in prediction:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position.x = human.pose.x
        marker.pose.position.y = human.pose.y
        marker.pose.position.z = 0.75
        marker.color.a = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 1.75
        markerArray.markers.append(marker)
        arrowMarker = Marker()
        arrowMarker.header.frame_id = "base_link"
        arrowMarker.type = marker.ARROW
        arrowMarker.action = marker.ADD
        point1 = Point();
        point2 = Point();
        point1.x = human.pose.x
        point1.y = human.pose.y
        point1.z = 0.75
        point2.x = human.pose.x + human.translational_velocity.x
        point2.y = human.pose.y + human.translational_velocity.y
        point2.z = 0.75
        arrowMarker.points.append(point1)
        arrowMarker.points.append(point2)
        arrowMarker.color.a = 1.0
        arrowMarker.scale.x = 0.25
        arrowMarker.scale.y = 0.25
        arrowMarker.scale.z = 0
        markerArray.markers.append(arrowMarker)
    if (len(markerArray.markers) > 0):
        marker_pub.publish(markerArray)
    else:
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.action = marker.DELETEALL
        markerArray.markers.append(marker)
        marker_pub.publish(markerArray)

def DepthCallback(msg):
    global currentDepth
    global depthInit
    currentDepth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    depthInit = True

def GetObservation(msg):
    global observation
    observation = []
    for box in msg.bounding_boxes:
        if (box.Class == "person"):
            if (box.probability) > detectionThreshold:
                u = box.xmin + ((box.xmax - box.xmin) // 2)
                v = box.ymin + ((box.ymax - box.ymin) // 2)
                x, y, z = ConvertFromDepthSquare(u, v, 5)
                human = HumanStateMsg()
                human.pose.x = x
                human.pose.y = -y
                human.pose.theta = 0
                human.translational_velocity.x = 0
                human.translational_velocity.y = 0
                human.rotational_velocity = 0
                # If Depth is zero there's something wrong
                if (x > 0):
                    observation.append(human)

def Distance(h1, h2):
    x1 = h1.pose.x
    y1 = h1.pose.y
    x2 = h2.pose.x
    y2 = h2.pose.y
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def GetClosest(human):
    global tracks
    if len(tracks) == 0:
        return -1, -1
    best_index = 0
    best_dist = 99999
    for key, value in tracks.items():
        distance = Distance(human, value[-1])
        if (distance < best_dist):
            best_dist = distance
            best_index = key
    return best_index, best_dist

def CreateTrackSimple(obs, time):
    global tracks
    global trackAge
    global id_counter
    obsList = [obs]
    tracks[id_counter] = obsList
    trackAge[id_counter] = 0
    trackTimes[id_counter] = [time]
    id_counter += 1

def UpdatePose(index, obs):
    tracks[index][-1].pose.x += obs.pose.x
    tracks[index][-1].pose.x /= 2
    tracks[index][-1].pose.y += obs.pose.y
    tracks[index][-1].pose.y /= 2

def UpdateTrackSimple(index, obs, time):
    global tracks
    global trackAge
    trackAge[index] = 0
    if (time == trackTimes[index][-1]):
        UpdatePose(index, obs)
    else:
        if (len(tracks[index]) >= maxTrackLength):
            tracks[index].pop(0)
            trackTimes[index].pop(0)
        tracks[index].append(obs)
        trackTimes[index].append(time)

def PruneTracks():
    global tracks
    global trackAge
    newTracks = copy.deepcopy(tracks)
    newAges = copy.deepcopy(trackAge)
    for index, track in tracks.items():
        newAges[index] += 1
        if (newAges[index] > ageThreshold):
            del newTracks[index]
            del newAges[index]
    tracks = newTracks
    trackAge = newAges

def GetVelocity(track, times):
    vxs = []
    vys = []
    if (len(track) < 2):
        return 0, 0
    for i in range(0, len(track)):
        t1 = track[i - 1]
        t2 = track[i]
        t1X = t1.pose.x
        t1Y = t1.pose.y
        t2X = t2.pose.x
        t2Y = t2.pose.y
        deltaT = times[i] - times[i - 1]
        vx = (t2X - t1X) / deltaT
        vy = (t2Y - t1Y) / deltaT
        if (fabs(vx) > 0 or fabs(vy) > 0):
            vxs.append((t2X - t1X) / deltaT)
            vys.append((t2Y - t1Y) / deltaT)
    return mean(vxs), mean(vys)

def EstimateVelocity():
    global prediction
    global tracks
    prediction = []
    print("Tracks: " + str(len(tracks)))
    for index, track in tracks.items():
        if (len(track) < minTrackLength):
            continue
        human = track[-1]
        human.id = index
        vx, vy = GetVelocity(track, trackTimes[index])
        human.translational_velocity.x = vx
        human.translational_velocity.y = vy
        print(human)
        prediction.append(human)

def PublishHumans():
    global prediction
    stateArray = HumanStateArrayMsg()
    for human in prediction:
        stateArray.human_states.append(human)
    human_pub.publish(stateArray)

def DetectionCallback(msg):
    global observation
    global prediction
    global id_counter
    # Need a depth frame to get 3d poses
    if (not depthInit):
        return
    # Update the detection times
    now = rospy.get_time()
    # Retrieve the human pose observations
    GetObservation(msg)
    # Associate the new observations to existing tracks
    for obs in observation:
        index, dist = GetClosest(obs)
        if (dist > distThreshold or index < 0):
            CreateTrackSimple(obs, now)
        else:
            UpdateTrackSimple(index, obs, now)
    PruneTracks()
    # Estimate Velocity / Forward Predict
    EstimateVelocity()
    # Publish Detections and Visualizations
    PublishHumans()
    DrawHumans()

def main():
    rospy.init_node(NODE_NAME)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, DetectionCallback)
    rospy.Subscriber('/camera/depth/image_raw', Image, DepthCallback)
    rospy.spin()

if __name__ == '__main__':
    main()
