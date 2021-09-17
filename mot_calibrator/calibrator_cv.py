# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image,CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import cv2 # OpenCV library
import math


from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

import tf_transformations
from scipy.spatial.transform import Rotation as R


camera_matrix=np.array([ [989.437842, 0.000000, 980.319408], 
                        [0.000000, 996.522603, 658.606498], 
                        [0.000000, 0.000000, 1.000000]], dtype=np.float32)

dist_coefficients=np.array([-0.299377, 0.069821, -0.000843, -0.001001, 0.000000], dtype=np.float32)


checkerboard_size=[4,5]



img_points=np.array([
[[ 957.1643,  1161.1813 ]],#p15
[[ 958.51874, 1187.1802 ]],#p45
[[ 894.47974, 1163.0116 ]],#p13
[[ 889.74634, 1188.9985 ]],#p43
[[ 832.6407,  1162.851  ]],#p11
[[ 821.0164,  1188.3483 ]]#p41
],dtype=np.float32)

trans_vec=np.array([-0.21408474,
        1.4380101 ,
        1.008065], dtype=np.float32)

rot_vec=np.array([[0.7338462, -0.4683586, -0.4920467],

                   [-0.4416951, 0.2213383, -0.8694336],

                   [0.5161155, 0.8553652, -0.0444434]], dtype=np.float32)



printed_value=False

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      CompressedImage, 
      '/port_a_camera_green/image_raw/compressed', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.objp = [] # 3d point in real world space
    self.imgp = [] # 2d points in image plane.
  
    
    #self.worldPOintsCalculated=np.zeros((checkerboard_size[0]*checkerboard_size[1],3), np.float32)
    self.calibrated=False
    self.counter=0

    self.br = TransformBroadcaster(self)
    while True:
      self.transform_publisher()

  def transform_publisher(self):
    t = TransformStamped()

    # Read message content and assign it to
    # corresponding tf variables
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'world'
    t.child_frame_id = "test"

   
    t.transform.translation.x =trans_vec[0]
    t.transform.translation.y = trans_vec[1]
    t.transform.translation.z = trans_vec[2]
   
    r = R.from_matrix(rot_vec)
  
    #q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
    q=r.as_quat()
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # Send the transformation
    self.br.sendTransform(t)

   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    print(current_frame.shape)

    self.getCorners(current_frame)
    #self.drawImgPoints(current_frame)
    
  def drawImgPoints(self,img):
   
    #img = cv2.circle(img, (img_points[2][0][0],img_points[2][0][1]), radius=5, color=(0, 0, 255), thickness=-1)
    #img = cv2.circle(img, (img_points[3][0][0],img_points[3][0][1]), radius=5, color=(0, 255, 0), thickness=-1)
    #img = cv2.circle(img, (img_points[4][0][0],img_points[4][0][1]), radius=5, color=(255, 0, 0), thickness=-1)
    img = cv2.circle(img, (img_points[5][0][0],img_points[5][0][1]), radius=5, color=(0, 0, 255), thickness=-1)
    i2=cv2.resize(img,(1440,906))
    cv2.imshow("test", i2)
    cv2.waitKey(500)

  def getCorners(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (4,5), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("gefunden")
        
        cv2.imwrite("output"+str(self.counter)+".jpg",img)
        self.counter+=1
        corners2=cv2.cornerSubPix(gray,corners, (4,5), (-1,-1), criteria)
        self.imgp.append(corners)
        #print(repr(corners2))SS

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (4,5), corners2, ret)
        print(self.imgp[0])
        if self.calibrated==False:
          self.calibrate_with_worldpoints(img)

    i2=cv2.resize(img,(960,604))
    cv2.imshow("test", i2)
    cv2.waitKey(500)
          
    
  def calibrate_with_worldpoints(self,img):
    
    abc,rvecs, tvecs = cv2.solvePnP(self.objp[0], self.imgp[0],camera_matrix , dist_coefficients)

    print("r vecs")
    print(repr(rvecs))

    print("t Vecs")
    print(repr(tvecs))

    rot_mat, test=cv2.Rodrigues(rvecs)
    print(rot_mat)
    self.calibrated=True





  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node exse it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()