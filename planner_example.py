import rclpy
import numpy as np
from rclpy.node import Node
from autoware_auto_msgs.msg import BoundingBoxArray
from autoware_auto_msgs.msg import RawControlCommand
from nav_msgs.msg import Odometry

# Input a 1x3 numpy vector and get its orthogonal vector
def RotateMap(vector, direction):
    rot_clo = np.array([0, 1, 0], [-1, 0, 0], [0, 0, 1])
    rot_count = np.array([0, -1, 0], [1, 0, 0], [0, 0, 1])
    
    # direction = 1: clockwise
    if direction == 1:
        return rot_clo.dot(vector)
    else:
        return rot_count.dot(vector)


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
        self.info_width_parameter = 1.0

    def controller_callback(self):
        #TODO
        msg = RawControlCommand()
        msg.throttle = 100
        
        if self.info_bounding_boxes_data_new is not None:
            for box in self.info_bounding_boxes_data_new.boxes:
                print('central of box is at %f %f %f.' % 
                        (box.centroid.x, box.centroid.y, box.centroid.z))

        if self.info_odom_data_new is not None:
            position = self.info_odom_data_new.pose.pose.position
            print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))

        if (self.info_odom_data_new is not None) and (self.info_bounding_boxes_data_new is not None) and (self.info_odom_data_old is not None) and (self.info_bounding_boxes_data_old is not None):
            position_new = self.info_odom_data_new.pose.pose.position
            position_old = self.info_odom_data_old.pose.pose.position
            self.info_direction_vector = np.array([position_new.x - position_old.x, position_new.y - position_old.y, 0])
            info_direction_unit_vector = self.info_direction_vector / np.linalg.norm(self.info_direction_vector)
            info_direction_counterclockwise_unit_vector = RotateMap(info_direction_unit_vector,1)
            info_direction_clockwise_unit_vector = RotateMap(info_direction_unit_vector,-1)


            

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


