#!/usr/bin/env python
import rospy
from gpg.msg import GraspMsg
from gpg.msg import GraspArrayMsg
from sensor_msgs.msg import Image
import cv2
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge
from cv2 import imwrite

    #Send them to network and receive back best grasp.
    
class Image_Aligner:
    def __init__(self):
        self.x = []
        self.y = []
        self.phi = []
        self.image = None
        self.grasp_num = 0

    #TODO: Receive grasp locaions     
    def GraspCB(self, grasps):
        Grasps = grasps.grasps
        # rospy.loginfo(grasps)
        for grasp in Grasps:
            self.x.append(grasp.x)
            self.y.append(grasp.y)
            self.phi.append(grasp.phi)
            self.grasp_num += 1
        rospy.loginfo('In GraspCB')

    #TODO: Receive depth image
    def ImageCB(self, msg):
        self.image = msg
        rospy.loginfo('In ImageCB')
    
    #TODO: Take the image and transform by x, y and phi
    def Transform(self, image, x, y, phi):

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')

        rows,cols = cv_image.shape
        # x = float(x)
        # y = float(y)
        M = np.float32([[1,0,int(x.data*1000.0)],[0,1,int(y.data*1000.0)]]) # might need scale up
        dst = cv2.warpAffine(cv_image,M,(cols,rows)) 

        rows,cols = dst.shape
        M = cv2.getRotationMatrix2D((cols/2,rows/2),phi.data,1)
        rot = cv2.warpAffine(dst,M,(cols,rows))
        rospy.loginfo('In TransformCB')

        return rot

if __name__ == '__main__':
    # catkin_pkg = os.getenv('ROS_PACKAGE_PATH').split(':')
    # catkin_pkg = str(catkin_pkg[0])

    rospy.init_node('grasp_alignment')
    ImgAligner = Image_Aligner()
 
    rospy.Subscriber("/grasp_set", GraspArrayMsg, ImgAligner.GraspCB)
    rospy.Subscriber("/camera/depth/image_raw", Image, ImgAligner.ImageCB)
    tf_pub = rospy.Publisher("/ImageTF", Image)
    
    while not rospy.is_shutdown():
        rospy.loginfo("In While loop")
        # DO stuff
        rospy.wait_for_message("/grasp_set", GraspArrayMsg)
        rospy.wait_for_message("/camera/depth/image_raw", Image)
        rospy.loginfo("Done Waiting, action ")
        for i in range(ImgAligner.grasp_num):
            rot_image  = ImgAligner.Transform(ImgAligner.image, ImgAligner.x[i], ImgAligner.y[i], ImgAligner.phi[i])
            resized = cv2.resize(rot_image, (32, 32), interpolation = cv2.INTER_AREA)
            # cv2.imread('img_tf', rot_image)
            bridge = CvBridge()
            image_message = bridge.cv2_to_imgmsg(resized, encoding="passthrough")
            tf_pub.publish(image_message)
            # rospy.WARN(rot_image)
            # cv2.imwrite('/home/jibran_old/catkin_ws/src/gpg/src/gpg/images/img'+str(i)+'.npy', resized)
            np.save('/home/jibran_old/catkin_ws/src/gpg/src/gpg/images/img'+str(i)+'.npy', resized)


        rospy.loginfo("Miricle")

        rospy.sleep(0.1)
    
