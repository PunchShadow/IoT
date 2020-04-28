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
    item_vector_r = np.linalg.norm(item_vector)
    item_vector_x = item_vector[0]
    item_vector_y = item_vector[1]
    item_vector_sin = item_vector_x/item_vector_r
    item_vector_cos = item_vector_y/item_vector_r
 
    direction_vector_r =  np.linalg.norm(direction_vector)
    direction_vector_x = direction_vector[0]
    direction_vector_y = direction_vector[1]
    direction_vector_sin = direction_vector_x/direction_vector_r
    direction_vector_cos = direction_vector_y/direction_vector_r
    
    side_judge = item_vector_r * (item_vector_sin * direction_vector_cos - direction_vector_sin * item_vector_cos)
    distance_judge = item_vector_r * (item_vector_cos * direction_vector_cos + item_vector_sin * direction_vector_sin)

    print("item_vector: ",item_vector, "direction_vector: ", direction_vector)
    print("side_judge: %f distance_judge: %f" % (side_judge, distance_judge))

    if (side_judge * side_judge < (width/2) * (width/2)) and (distance_judge > 0) and (distance_judge * distance_judge < alert_distance * alert_distance):
        return 1
    else:
        return -1



    

class Driver(Node):

    def __init__(self):
        super().__init__('Driver')
        self.pub = self.create_publisher(RawControlCommand, '/vehicle_cmd', 10)
        self.create_subscription(BoundingBoxArray, '/lidar_bounding_boxes', self.bounding_boxes_callback, 10)
        self.create_subscription(Odometry, '/lgsvl_odom', self.odom_callback, 10)
        self.tmr = self.create_timer(1.0, self.controller_callback)
        #TODO
        self.info = None
        self.info_bounding_boxes_data = None
        self.info_odom_data = None
        self.info_bounding_boxes_data_new = None
        self.info_bounding_boxes_data_old = None
        self.info_odom_data_new = None
        self.info_odom_data_old = None
        self.info_direction_vector = None
        self.info_direction_vector_old = None
        self.info_width_parameter = 0.5
        self.info_direction_unit_vector_old = np.array([1,0,0])
        self.info_left_point = None
        self.info_right_point = None
        self.info_alert_distance = 5
        self.info_current_position = None

    def controller_callback(self):
        #TODO
        msg = RawControlCommand()
        msg.throttle = 10
        
        '''
        if self.info_bounding_boxes_data_new is not None:
            for box in self.info_bounding_boxes_data_new.boxes:
                print('central of box is at %f %f %f.' % (box.centroid.x, box.centroid.y, box.centroid.z))
        '''
        if self.info_odom_data_new is not None:
            position = self.info_odom_data_new.pose.pose.position
            self.info_current_position = np.array([position.x,position.y,position.z])
            print('Current pos:',self.info_current_position)

        # detect and act
        if (self.info_odom_data_new is not None) and (self.info_bounding_boxes_data_new is not None) and (self.info_odom_data_old is not None) and (self.info_bounding_boxes_data_old is not None):
            position_new = self.info_odom_data_new.pose.pose.position
            position_old = self.info_odom_data_old.pose.pose.position
            self.info_direction_vector = np.array([position_new.x - position_old.x, position_new.y - position_old.y, 0])
            info_direction_unit_vector = self.info_direction_unit_vector_old
            info_direction_vector_length = np.linalg.norm(self.info_direction_vector)
            
            if info_direction_vector_length != 0:
                info_direction_unit_vector = self.info_direction_vector / np.linalg.norm(self.info_direction_vector)
                self.info_direction_unit_vector_old = info_direction_unit_vector

            for box in self.info_bounding_boxes_data_new.boxes:
                # item
                item_position = np.array([box.centroid.x, box.centroid.y, 0]) + np.array([position_new.x, position_new.y , 0])
                item_vector = np.array([box.centroid.x, box.centroid.y, 0])
                print("item",item_position-self.info_current_position)
                if CalculateCollisionDetect(item_vector, self.info_direction_vector, self.info_width_parameter, self.info_alert_distance) == 1:
                    print("collision alert")
                    print(item_vector)
        
        self.pub.publish(msg)

    def bounding_boxes_callback(self, data):
        #print('There are %d bounding boxes.' % len(data.boxes))
        #for box in data.boxes:
        #    print('central of box is at %f %f %f.' % 
        #            (box.centroid.x, box.centroid.y, box.centroid.z))
        #TODO
        self.info_bounding_boxes_data_old = self.info_bounding_boxes_data_new
        self.info_bounding_boxes_data_new = data

    def odom_callback(self, data):
        #position = data.pose.pose.position
        #print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
        #TODO
        self.info_odom_data_old = self.info_odom_data_new
        self.info_odom_data_new = data

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
