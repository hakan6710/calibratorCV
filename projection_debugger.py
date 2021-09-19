import numpy as np
import cv2
from pprint import pprint
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import time
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

image_width_px = 1920
image_height_px = 1208

def get_divisors(n):
    for i in range(1, int(n / 2) + 1):
        if n % i == 0:
            yield i
    yield n
grid_shape_x = None
grid_shape_y = None
image_width_px
last_divisor = 0
for divisor in get_divisors(image_width_px):
    if divisor > 10:
        break
    last_divisor = divisor

grid_shape_x = 10#int(image_width_px/list(get_divisors(image_width_px))[6])

for divisor in get_divisors(image_height_px):
    if divisor > 10:
        break
    last_divisor = divisor

grid_shape_y = 10#int(image_height_px/list(get_divisors(image_height_px))[6])

#print(grid_shape_x, grid_shape_y)

points_to_project = np.zeros(((grid_shape_x)*(grid_shape_y), 1, 2), np.float32)
index = 0
for i in range(0,grid_shape_x):
    for j in range(0,grid_shape_y):
        points_to_project[index][0][0] = int((image_width_px/grid_shape_x) * i)
        points_to_project[index][0][1] = int((image_height_px/grid_shape_y) * j)
        index += 1

camera_matrix=np.array([ [976.637135, 0.000000, 952.880210], \
    [0.000000, 983.500564, 649.387647], \
    [0.000000, 0.000000, 1.000000]], dtype=np.float32)

dist_coefficients=np.array([-0.270602, 0.051047, -0.000236, 0.000302, 0.000000], dtype=np.float32)



#pprint(projected_points)


rclpy.init(args=None)
node = rclpy.create_node('projection_debugger')
publisher = node.create_publisher(Marker, 'debug_grid', 10)
tf2_broadcaster = TransformBroadcaster(node) 


header = Header()
header.frame_id = "map"
marker = Marker()
marker.header = header
marker.ns = "main"
marker.id = 0
marker.type = Marker.POINTS
marker.action = Marker.ADD
marker.lifetime = Duration().to_msg()
marker.scale.x = 0.1
marker.scale.y = 0.1

rotation_matrix = np.eye(3)

transform_matrix = np.array([0.0, 0.0, 1.0])

while rclpy.ok():
    

    t = TransformStamped()

    # Read message content and assign it to
    # corresponding tf variables
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = 'debug'

   
    t.transform.translation.x = transform_matrix[0]
    t.transform.translation.y = transform_matrix[1]
    t.transform.translation.z = transform_matrix[2]
   
    r = Rotation.from_matrix(rotation_matrix)

    print(r.as_euler('xyz', degrees=True))
  
    #q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
    q=r.as_quat()
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    tf2_broadcaster.sendTransform(t)
    undistortedAndNormalizedPointMatrix = cv2.undistortPoints(points_to_project, camera_matrix,  dist_coefficients, P=camera_matrix)
    projected_points = []

    #print(grid_shape_x)
    #print(list(get_divisors(image_width_px)))
    #print(grid_shape_y)
    #print(list(get_divisors(image_height_px)))

    pprint(undistortedAndNormalizedPointMatrix)
    #print(undistortedAndNormalizedPointMatrix.shape)
    for norm in undistortedAndNormalizedPointMatrix:
        #print(norm.shape)
        norm_expanded = np.array([[norm[0][0], norm[0][1], 1]])
        #print(norm_expanded.shape)
        rotated_ray = norm_expanded * np.linalg.inv(rotation_matrix) * np.linalg.inv(camera_matrix)
        #print(rotated_ray.shape)
        rotated_normalised_ray = rotated_ray / rotated_ray[0][2]
        #print(rotated_normalised_ray.shape)
        xGround = (-1) * transform_matrix[2] * rotated_normalised_ray[0][0] + (-1) * transform_matrix[0]
        yGround = transform_matrix[2] * rotated_normalised_ray[0][1] + transform_matrix[1]
        #print(xGround.shape)

        projected_points.append((xGround,yGround,0.0))

        for i, proj in enumerate(projected_points):
            point = Point()
            point.x = proj[0]
            point.y = proj[1]
            point.z = proj[2]
            color = ColorRGBA()
            color.r = points_to_project[i][0][0] / image_width_px
            color.g = points_to_project[i][0][1] / image_height_px
            color.b = 0.0
            color.a = 1.0
            marker.points.append(point)
            marker.colors.append(color)

    header.stamp = node.get_clock().now().to_msg()

    publisher.publish(marker)
    marker.points = []
    marker.colors = []
    try:
        command  = ""
        command = input("Next command: ")
        if command[0] == "r":
            if command[1] == "x":
                r = Rotation.from_matrix(rotation_matrix)
                if command[2] == "-":
                    rotation_matrix = Rotation.from_euler('xyz', r.as_euler('xyz', degrees=True) + np.array([-int(command[3:]), 0.0, 0.0]), True).as_matrix()
                else:
                    rotation_matrix = Rotation.from_euler('xyz', r.as_euler('xyz', degrees=True) + np.array([int(command[2:]), 0.0, 0.0]), True).as_matrix()
            if command[1] == "y":
                r = Rotation.from_matrix(rotation_matrix)
                if command[2] == "-":
                    rotation_matrix = Rotation.from_euler('xyz', r.as_euler('xyz', degrees=True) + np.array([0.0, -int(command[3:]), 0.0]), True).as_matrix()
                else:
                    rotation_matrix = Rotation.from_euler('xyz', r.as_euler('xyz', degrees=True) + np.array([0.0, int(command[2:]), 0.0]), True).as_matrix()
            if command[1] == "z":
                r = Rotation.from_matrix(rotation_matrix)
                if command[2] == "-":
                    rotation_matrix = Rotation.from_euler('xyz',r.as_euler('xyz', degrees=True) + np.array([0.0, 0.0, -int(command[3:])]), True).as_matrix()
                else:
                    rotation_matrix = Rotation.from_euler('xyz', r.as_euler('xyz', degrees=True) + np.array([0.0, 0.0, int(command[2:])]), True).as_matrix()
        #if command[0] == "t":
            #if command[1] == "x":
            #    if command[2] == "-":
            #        print()
            #    else:
            #        print()
            #if command[1] == "y":
            #    rot = Rotation.from_matrix(rotation_matrix).as_euler()
            #    print(rot)
            #if command[1] == "z":
            #    rot = Rotation.from_matrix(rotation_matrix).as_euler()
            #    print(rot)
    except Exception:
        print("Wrong input!")
    print("Publishing!")
    #time.sleep()

node.destroy_node()

rclpy.shutdown()
exit()