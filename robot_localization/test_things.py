from asyncio import start_unix_server
import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler
import pf
   
def initialize_particle_cloud(timestamp, xy_theta=None):
    """ Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                    particle cloud around.  If this input is omitted, the odometry will be used """
    particle_cloud = []
    # TODO create particles
    length=math.floor(math.sqrt(17*17))
    change=0.1
    box=change*math.floor(length/2)
    start_x=xy_theta[0]-box
    start_y=xy_theta[1]-box
    start_theta=xy_theta[2]
    xshift=0
    for i in range(length):
        yshift=0
        for j in range(length):
            particle_cloud.append([start_x+xshift,start_y+yshift,start_theta,0])
            yshift=yshift+change
        xshift=xshift+change
    print(particle_cloud)
initialize_particle_cloud(1,(2,2,0))