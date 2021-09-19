# Import the necessary libraries
from numpy.core.fromnumeric import shape
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image,CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import cv2 # OpenCV library
import math


# camera_matrix=np.array([ [989.437842, 0.000000, 980.319408], 
#                         [0.000000, 996.522603, 658.606498], 
#                         [0.000000, 0.000000, 1.000000]], dtype=np.float32)

# dist_coefficients=np.array([-0.299377, 0.069821, -0.000843, -0.001001, 0.000000], dtype=np.float32)


# checkerboard_size=[4,5]



camera_matrix=np.array([ [976.637135, 0.000000, 952.880210], 
                        [0.000000, 983.500564, 649.387647], 
                        [0.000000, 0.000000, 1.000000]], dtype=np.float32)

dist_coefficients=np.array([-0.270602, 0.051047, -0.000236, 0.000302, 0.000000], dtype=np.float32)

world_points=np.array([
    [[0.255, 3.2,0]], #p15
    [[0.245,2.72,0]],#p45
    [[-0.014,3.21,0]],#p13
    [[0,2.73,0]],#p43
    [[-0.32, 3.225, 0]],#p11
    [[-0.32,2.75,0]]#p41
],dtype=np.float32)


img_points=np.array([
[[ 957.1643,  1161.1813 ]],#p15
[[ 958.51874, 1187.1802 ]],#p45
[[ 894.47974, 1163.0116 ]],#p13
[[ 889.74634, 1188.9985 ]],#p43
[[ 832.6407,  1162.851  ]],#p11
[[ 821.0164,  1188.3483 ]]#p41
],dtype=np.float32)

rotation_matrix=np.array([[ 0.998915, -0.00823395, -0.04583641],
[-0.04174363, 0.27802867, -0.9596653 ],
[ 0.02064567, 0.9605375, 0.2773833 ]],dtype=np.float32)

rotation_vector=np.array([[ 1.2889872 ],
[-0.04462786],
[-0.02249426]], dtype=np.float32)


translation_vector=np.array([[-0.21408474],
[ 1.4380101 ],
[ 1.008065  ]], dtype=np.float32)


printed_value=False

width = int(1920 * 0.75 / 100)
height = int(1208 * 0.75 / 100)
dim = (width, height)


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
    self.objp = world_points # 3d point in real world space
    self.imgp = img_points # 2d points in image plane.
    self.camera_matrix=camera_matrix
    self.dist_coeff=dist_coefficients
    self.rot_mat=rotation_matrix
    self.translation_vec=translation_vector
    self.rvec=rotation_vector
    

    self.calibrated=False
    self.counter=0

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    print(current_frame.shape)
    #self.calculate_xyz(957.1643,  1161.1813)
    #self.reproject_points(current_frame)
    self.getCorners(current_frame)
    #self.drawImgPoints(current_frame)


  def drawImgPoints(self,img,imgpts):
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 0.5
    fontColor              = (255,255,255)
    lineType               = 2

    
    print(repr(imgpts))
    for point in imgpts:
      #img = cv2.circle(img, (point[0][0],point[0][1]), radius=5, color=(0, 0, 255), thickness=-1)
      img=cv2.putText(img,f"X={0},Y={0}", 
        (point[0][0],point[0][1]), 
        font, 
        fontScale,
        fontColor,
        lineType)
    i2=cv2.resize(img,dim)
    cv2.imshow("test", i2)
    cv2.waitKey(500)
  
  def calculate_xyz(self,img,imgpts):
    print("calculating xyz")
    undistortedAndNormalizedPointMatrix = cv2.undistortPoints(imgpts, self.camera_matrix,  dist_coefficients,P=self.camera_matrix)
    projected_points = []

    #print(grid_shape_x)
    #print(list(get_divisors(image_width_px)))
    #print(grid_shape_y)
    #print(list(get_divisors(image_height_px)))

    print(undistortedAndNormalizedPointMatrix)
    #print(undistortedAndNormalizedPointMatrix.shape)
    for norm in undistortedAndNormalizedPointMatrix:
        #print(norm.shape)
        norm_expanded = np.array([[norm[0][0], norm[0][1], 1]])
        #print(norm_expanded.shape)
        rotated_ray = norm_expanded * np.linalg.inv(self.rot_mat) * np.linalg.inv(self.camera_matrix)
        #print(rotated_ray.shape)
        rotated_normalised_ray = rotated_ray / rotated_ray[0][2]
        #print(rotated_normalised_ray.shape)
        xGround = (-1) * self.translation_vec[2] * rotated_normalised_ray[0][0] + (-1) * self.translation_vec[0]
        yGround = self.translation_vec[2] * rotated_normalised_ray[0][1] +self.translation_vec[1]
        print(f"xGround={xGround}")
        print(f"yGround={yGround}")

        projected_points.append((xGround,yGround,0.0))
  
  


  def getCorners(self,img):

    img=cv2.undistort(img,camera_matrix,dist_coefficients)
    print(shape(img))
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
    cv2.imwrite("output"+"distorted"+".jpg",img)
    i2=cv2.resize(img,dim)
    cv2.imshow("t11", i2)
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

  def reproject_points(self,img):
    print("Reprojecting and Drawing")
    # print(self.rot_mat)
    # print(self.translation_vec)
    # print(self.objp[0])
    imgpts, jac = cv2.projectPoints(self.objp, self.rvec, self.translation_vec, self.camera_matrix, self.dist_coeff)
    self.calculate_xyz(img,imgpts)
    #self.drawImgPoints(img,imgpts=imgpts)






  
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