# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image,CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import cv2 # OpenCV library





camera_matrix=np.array([ [989.437842, 0.000000, 980.319408], 
                        [0.000000, 996.522603, 658.606498], 
                        [0.000000, 0.000000, 1.000000]], dtype=np.float32)

dist_coefficients=np.array([-0.299377, 0.069821, -0.000843, -0.001001, 0.000000], dtype=np.float32)



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
  
    self.centerPoint= [-0.4191396, 2.035, 1.792]
    self.worldPOintsCalculated=np.zeros((10*7,3), np.float32)
    self.calibrated=False
   

   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    #self.draw_line(current_frame)
    self.test(current_frame)
    self.calculate_worldpoints()
    if self.calibrated==False:
      self.calibrate_with_worldpoints(current_frame)


  def getCorners(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,10), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        
        corners2=cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        self.imgp.append(corners)
        #print(repr(corners2))

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7,10), corners2, ret)
        im2=cv2.resize(img,(960,604))
        cv2.imshow("test", im2)
        cv2.waitKey(500)
      
    
  def calibrate_with_worldpoints(self,img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #temp_imgpoints=np.array(self.imgpoints[0], dtype=np.float32)
    

    self.objp.append(self.worldPOintsCalculated)

    
    print(self.worldPOintsCalculated)
    print(self.imgp[0])
    
    ret, cam_mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objp, self.imgp, gray.shape[::-1], camera_matrix, dist_coefficients,rr,tt,
    flags=cv2.CALIB_USE_INTRINSIC_GUESS)

    
  
    print("r vecs")
    print(repr(rvecs))

    print("t Vecs")
    print(repr(tvecs))

    
    rot_mat, _=cv2.Rodrigues(rvecs[0])
    print(rot_mat)
    

    self.calibrated=True

  def calculate_worldpoints(self,pattern=[7,10],widthCheckerboard=0.057):
    index=0
    for i in range(pattern[1]):
      for j in range(pattern[0]):
        # self.worldPOintsCalculated[index]=[self.centerPoint[0]+i*widthCheckerboard, 
        #                                    self.centerPoint[1],
        #                                    self.centerPoint[2]-j*widthCheckerboard]
        self.worldPOintsCalculated[index]=[self.centerPoint[0]+(pattern[1]-i-1)*widthCheckerboard, 
                                           self.centerPoint[1],
                                           self.centerPoint[2]-j*widthCheckerboard]

        index=index+1
    #print(repr(self.worldPO # self.worldPOintsCalculated[index]=[self.centerPoint[0]+i*widthCheckerboard, 
        #                                    self.centerPoint[1],
        #                                    self.centerPoint[2]-j*widthCheckerboard]


  def draw_line(self, img):
    # Start coordinate, here (225, 0)
    # represents the top right corner of image

    height, width = img.shape[:2]
    start_point = (int(width/2), int(0))
    
    # End coordinate, here (0, 225)
    # represents the bottom left corner of image
    end_point = (int(width/2), int(height-1))
    
    print(height)
    print(width)
    # Black color in BGR
    color = (0, 0, 0)
    
    # Line thickness of 5 px
    thickness = 5
    
    # Using cv2.line() method
    # Draw a diagonal black line with thickness of 5 px
    cv2.line(img, start_point, end_point, color, thickness)


    s2=(0, int(height/2))
    e2=(width, int(height/2))
    cv2.line(img, s2, e2, color, thickness)




    
  
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