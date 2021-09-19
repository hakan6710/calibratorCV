from os import initgroups
import numpy as np
import cv2 # OpenCV library

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

rot_mat_test=np.array([0.998915, -0.00823395, -0.04583641,-0.04174363, 0.27802867, -0.9596653 ,
 0.02064567, 0.9605375, 0.2773833],dtype=np.float32)

rotation_vector=np.array([[ 1.2889872 ],
[-0.04462786],
[-0.02249426]], dtype=np.float32)


translation_vector=np.array([[-0.21408474],
[ 1.4380101 ],
[ 1.008065  ]], dtype=np.float32)


# translation_vector_offseted=np.array([[-0.19951057],
#        [ 0.94589937],
#        [-0.6920863 ]], dtype=np.float32)

# rotation_vector_offseted=np.array([[ 1.2889872 ],
#        [-0.04462796],
#        [-0.02249431]], dtype=np.float32)

# rotation_matrix_offseted=np.array([[ 0.998915,   -0.00823397, -0.04583651],
#  [-0.04174372,  0.27802867, -0.9596653 ],
#  [ 0.02064572,  0.9605375,   0.2773833 ]],dtype=np.float32)

# world_points_offseted=np.array([
#     [[0.255, 4.97,0]], #p15
#     [[0.245,4.49,0]],#p45
#     [[-0.014,4.98,0]],#p13
#     [[0,4.50,0]],#p43
#     [[-0.32, 4.995, 0]],#p11
#     [[-0.32,4.52,0]]#p41
# ],dtype=np.float32)




# Extrinsic Parameters Matrix
translation_vector_transposed = np.transpose(translation_vector)
#print(np.shape(translation_vector_transposed))
#print(np.shape(rotation_matrix))

extrinsic_matrix=np.column_stack((rotation_matrix, translation_vector_transposed[0]))



# Projection Matrix
projection_matrix = np.mat(camera_matrix) * np.mat(extrinsic_matrix)
#print(projection_matrix)

# Homography Matrix
p11 = projection_matrix[0,0]
p12 = projection_matrix[0,1]
p14 = projection_matrix[0,3]
p21 = projection_matrix[1,0]
p22 = projection_matrix[1,1]
p24 = projection_matrix[1,3]
p31 = projection_matrix[2,0]
p32 = projection_matrix[2,1]
p34 = projection_matrix[2,3]
homography_matrix = np.array([[p11,p12,p14], [p21,p22,p24], [p31,p32,p34]], dtype=np.float)
homography_matrix_inverse = np.linalg.inv(homography_matrix)



projection_result=[]
def getRevAndTvec(worldpoints,img_points, camera_matrix,dist_coeff):
    boolFlag,rvecs, tvecs = cv2.solvePnP(worldpoints, img_points,camera_matrix , dist_coeff,flags=0)

    print("r vecs")
    print(repr(rvecs))

    print("t Vecs")
    print(repr(tvecs))
    


def calculate_rot(rvec):
    rot_mat, _=cv2.Rodrigues(rvec)
    print(rot_mat)

def calculate_xyz(idx,u=0,v=0):
    #print(f"calculating for u={u},v={v}")

    uv=np.mat(np.transpose([u,v,1]))

    dunno=np.squeeze(np.asarray(homography_matrix_inverse))
    uv_squeezed=np.squeeze(np.asarray(uv))
    point_3D_w = np.dot(dunno,uv_squeezed) 

    # Normalization
    point_3D = np.divide(point_3D_w,point_3D_w[2])
    p_x=point_3D[0]
    p_y=point_3D[1]
    print(f"x={p_x},x_offset={p_x-world_points[idx][0][0]},y={p_y}, y_offset={p_y-world_points[idx][0][1]}")

def reproject_points(w_points=world_points,rvec=rotation_vector,tvec=translation_vector,cam_mx= camera_matrix, dist_coef=dist_coefficients):
    zero_dist=np.array([0, 0, 0,0, 0.000000], dtype=np.float32)
    print("Reprojecting and Drawing")
    imgpts, jac = cv2.projectPoints(w_points, rvec, tvec, cam_mx, zero_dist)
    imgpts_undistorted, jac = cv2.projectPoints(w_points, rvec, tvec, cam_mx, dist_coef)
    print("Zerodist")
    print(imgpts)
    print("Undistored")
    print(imgpts_undistorted)


# def old_2dto3d(imgpoints):
#     undistortedAndNormalizedPointMatrix = cv2.undistortPoints(imgpoints, cameraMatrix=camera_matrix, distCoeffs= dist_coefficients,P=camera_matrix)
#     projected_points = []


#     print(undistortedAndNormalizedPointMatrix)
#     #print(undistortedAndNormalizedPointMatrix.shape)
#     for norm in undistortedAndNormalizedPointMatrix:
#         #print(norm.shape)
#         norm_expanded = np.array([[norm[0][0], norm[0][1], 1]])
#         #print(norm_expanded.shape)
#         rotated_ray = norm_expanded * np.linalg.inv(rotation_matrix)# * np.linalg.inv(camera_matrix)
#         #print(rotated_ray.shape)
#         rotated_normalised_ray = rotated_ray / rotated_ray[0][2]
#         #print(rotated_normalised_ray.shape)
#         xGround =   translation_vector[2] * rotated_normalised_ray[0][0] +   translation_vector[0]
#         yGround = translation_vector[2] * rotated_normalised_ray[0][1] + translation_vector[1]
#         #print(xGround.shape)
#         print(f"xGround={xGround},yGround={yGround}")
#         projected_points.append((xGround,yGround,0.0))


def new_shit(input_imgPoints=img_points):
 

    Extrincic=cv2.hconcat([rotation_matrix,translation_vector])
    Projection_mtx=camera_matrix.dot(Extrincic)

    #delete the third column since Z=0 
    Projection_mtx = np.delete(Projection_mtx, 2, 1)

    #finding the inverse of the matrix 
    Inv_Projection = np.linalg.inv(Projection_mtx)

    
    
    for idx,img in enumerate(input_imgPoints):

        #print(idx)
        #detected image point (extracted from a que)
        img_point=np.array( [[img[0][0]],  [img[0][1]]] )

        #adding dimension 1 in order for the math to be correct (homogeneous coordinates)
        img_point=np.vstack((img_point,np.array(1)))

        #calculating the 3D point which located on the 3D plane
        projected_Point=Inv_Projection.dot(img_point)
        #print(projected_Point)
        p_x=projected_Point[0]/projected_Point[2]
        p_y=projected_Point[1]/projected_Point[2]
        print(f"x={p_x},x_offset={p_x-world_points[idx][0][0]},y={p_y}, y_offset={p_y-world_points[idx][0][1]}")
        projection_result.append([projected_Point[0]/projected_Point[2],projected_Point[1]/projected_Point[2]])

 


def main(args=None):
    #getRevAndTvec(world_points_offseted,img_points,camera_matrix,dist_coefficients)
    #calculate_rot(rotation_vector_offseted)

    print("First Method")
    for idx,point in enumerate(img_points):
        #print(point)
        calculate_xyz(idx,u=point[0][0],v=point[0][1])

    print("Second Method")
    new_shit()
    print("Reprojecting")
    reproject_points(w_points=world_points,tvec=translation_vector,rvec=rotation_vector)
    #old_2dto3d(img_points)
if __name__ == '__main__':
  main()

