#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
import image_geometry 
from tf import TransformListener
from std_srvs.srv import Empty
import move_base


class IdentifyAndSpray:
    def __init__(self):
        # set update frequency
        rospy.Rate(3)

        # subscribe to the darknet bounding box topic and check if objects have been identified, and bounding boxes drawn
        self.detection_sub =rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bbox_callback)

        # array to store real world coordinates of weed
        self.world_array = []

        # get information about the camera. This will aid in converting the pixel coordinates to world coordinates
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', CameraInfo, self.camera_info_callback)

        self.tf_listener = TransformListener()

        # spray service topi
        self.spray_service = '/thorvald_001/spray'


    def bbox_callback(self, msg):
        # iterate through all the bounding boxes
        for box in msg.bounding_boxes:
            # only perform weeding operation if the probability of the bounding boxes >= 42%
            if box.probability >= 0.42:
                # finds the center of the bounding boxes
                center = self.find_bbox_center(box)

                # convert the center of the bounding boxes to world coordinates, in our case 'map
                world_coordinate = self.convert_to_world_frame(center)

                # call move_base action to move robot to the weed position
                m = move_base.MoveRobotToGoal()
                m.move2goal(world_coordinate)

                # call spray service
                self.spray_weed()
            

    """
    finds the center of the weed image by using darknet_ros bounding box xmin, xmax, ymin, ymax
    """
    def find_bbox_center(self, box):
        x = (box.xmax + box.xmin) / 2
        y = (box.ymin + box.ymax) / 2
        return (x, y)


    """
    converts the center of the bounding boxes to a 3d ray using Camera Pinhole. The position is stored as a PoseStamped
    messaged and transformed to our world frame, /map
    """
    def convert_to_world_frame(self, center):
        # get the 3d ray of rectified yolo pixel coordinates
        cam_point = self.camera_model.projectPixelTo3dRay(self.camera_model.rectifyPoint(center))
        # define camera point
        t_robot = PoseStamped()
        t_robot.pose.orientation.w = 1.0
        t_robot.pose.position.x= cam_point[0]
        t_robot.pose.position.y=cam_point[1]
        t_robot.pose.position.z= cam_point[2]
        
        t_robot.header.stamp = rospy.Time.now()
        t_robot.header.frame_id = self.camera_model.tfFrame()
        
        self.tf_listener.waitForTransform(self.camera_model.tfFrame(), '/map', rospy.Time.now(), rospy.Duration(1.0))
        tf_point = self.tf_listener.transformPose('/map', t_robot)
        
        return tf_point
        

    """
    waits for spray service to avoid an error, and calls it using the ServiceProxy function
    """
    def spray_weed(self):
        rospy.wait_for_service(self.spray_service)
        service = rospy.ServiceProxy(self.spray_service, Empty)
        try:
            do_spray = service()
        except rospy.ServiceException as e:
            print("Service failed. Error message: " + str(e))


    """
    this gives information about our camera model
    """
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        



if __name__ == '__main__':
    try: 
        rospy.init_node('identify_and_spray')
        IdentifyAndSpray()
        rospy.spin()
    except:
        rospy.loginfo("Error with execution file.")