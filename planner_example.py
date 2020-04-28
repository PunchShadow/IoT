import rclpy
import numpy as np
from rclpy.node import Node
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

# Input a 1x3 numpy vector and get its orthogonal vector
import math

def RotateMap(vector, direction):
    rot_clo = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    rot_count = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    
    # direction = 1: clockwise
    if direction is 1:
        return np.dot(rot_clo, vector)
    else:
        return np.dot(rot_count, vector)

# Find a cross b's direction (1x3)
# Return 1 if Z is positive, else -1.
def CrossDirection(a, b):
    result = np.cross(a, b)
    if result[2] < 0:
        return -1;
    else:
        return 1;

# Project vector a to direction
# Output (projection, rejection)
def VectorProjection(a, direction):
    cos = np.dot(direction, a) / (np.linalg.norm(direction) * np.linalg.norm(a))
    projection = (np.dot(direction, a) / np.dot(a, a)) * direction
    rejection = a - projection
    return projection, rejection

# Determine if a,b is the same direction by minus them
# If a, b is the same direction, return 1, else 0
def FindSameDirection(a, b):
    # If either a or b is zero vector, return -1
    if (np.linalg.norm(a) is 0) or (np.linalg.norm(b) is 0):
        return -1;
    else:
        norm_a = a / np.linalg.norm(a)
        norm_b = b / np.linalg.norm(b)
        c = norm_a - norm_b
        # a, b is the same direction
        if (np.linalg.norm(c) < math.sqrt(2)):
            return 1
        else:
            return 0
# TODO
def TransformAngle(direction, src):
    norm_a = direction / np.linalg.norm(direction)
    cos = np.linalg.norm(np.dot(norm_a, np.array([0,1,0])))
    sin = np.linalg.norm(np.cross(norm_a, np.array([0,1,0])))


def CalculateCollisionDetect(item_vector, direction_vector, width, alert_distance):
    item_vector

# If two vector's multiplication's elements are all positive,
# two vectors are same direction
def SameDirection(a, project_a):
    if ((np.sign(a) * np.sign(project_a)).all() >= 0): 
        return 1
    else:
        return 0
    

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand, '/vehicle_cmd', 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes', self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl_odom', self.odom_callback, 10)
        self.tmr = self.create_timer(1.0, self.controller_callback)
        #TODO
        self.position = None
        self.boxes_old = None
        self.boxes_new = None
        self.odom_old = None
        self.odom_new = None
        self.car_direction = None
        self.car_direction_pre = None
        self.alert_distance = 5.0# The distance to take the brake
        self.car_width = 1.0
        self.car_height = 10.0

    def controller_callback(self):
        #TODO
        msg = RawControlCommand()
        msg.throttle = 10
        '''
        if self.boxes_new is not None:
            for box in self.boxes_new.boxes:
                print('central of box is at %f %f %f.' % (box.centroid.x, box.centroid.y, box.centroid.z))
        '''
        #if self.odom_new is not None:
            #position = self.odom_new.pose.pose.position
            #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))

        # detect and act
        if (self.odom_new is not None) and (self.boxes_new is not None) and (self.odom_old is not None) and (self.boxes_old is not None):
            position_new = self.odom_new.pose.pose.position
            position_old = self.odom_old.pose.pose.position
            self.car_direction = np.array([position_new.x - position_old.x, position_new.y - position_old.y, 0])
            
            # If the velocity is not detected then take the previous speed instead
            if (self.car_direction.all() == 0):
                self.car_direction = self.car_direction_pre
            else:
                self.car_direction_pre = self.car_direction 
            

            for box in self.boxes_new.boxes:
                # If the item is too high, like traffic light,etc.
                if (box.centroid.y > self.car_height):
                    continue

                # item location
                item_position = np.array([box.centroid.x, box.centroid.y, 0]) + np.array([position_new.x, position_new.y , 0])
                item_vector = np.array([box.centroid.x, box.centroid.y, 0])
                projection, rejection = VectorProjection(item_vector, self.car_direction)
                item_othogonol = np.linalg.norm(rejection) # The distance orthogonol between item and car
                
                item_distance = np.linalg.norm(projection)
                #print(SameDirection(projection, self.car_direction), projection, self.car_direction)
                print("Othogonol distance is: ", item_othogonol)
                print("Item distance is: ", item_distance)
                print("-----------------------------------------------------------------")
                if (SameDirection(projection, self.car_direction)):
                    if (item_othogonol < self.car_width):
                        if (item_distance < self.alert_distance):
                            # The Hitting alert
                            #print(item_position, " item_distance:", item_distance)
                            print("Almost hit XDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD")
                        print("Nearby the item !!!!!!!!!~~~~~~~~~")
                        #print(item_position, "Bypassing:", item_othogonol)
                        # Bypassing alert
                    



        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        #print('There are %d bounding boxes.' % len(data.boxes))
        #for box in data.boxes:
        #    print('central of box is at %f %f %f.' % 
        #            (box.centroid.x, box.centroid.y, box.centroid.z))
        #TODO
        self.boxes_old = self.boxes_new
        self.boxes_new = data

    def odom_callback(self, data):
        #position = data.pose.pose.position
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #TODO
        self.odom_old = self.odom_new
        self.odom_new = data

rclpy.init()
node = Driver()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass

node.destroy_node()
rclpy.shutdown()



# TODO List:
"""
    1. Coordination
    2. 
"""
