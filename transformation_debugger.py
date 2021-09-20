from os import initgroups
import numpy as np
import cv2 # OpenCV library
from scipy.spatial.transform import Rotation as R

Camera_matrix=np.array([ [976.637135, 0.000000, 952.880210], 
                        [0.000000, 983.500564, 649.387647], 
                        [0.000000, 0.000000, 1.000000]], dtype=np.float64)

Dist_coefficients=np.array([-0.270602, 0.051047, -0.000236, 0.000302, 0.000000], dtype=np.float64)

World_points=np.array([
    [[0.255, 3.2,0]], #p15
    [[0.245,2.72,0]],#p45
    [[-0.014,3.21,0]],#p13
    [[0,2.73,0]],#p43
    [[-0.32, 3.225, 0]],#p11
    [[-0.32,2.75,0]]#p41
],dtype=np.float64)





Img_points=np.array([
[[ 957.1643,  1161.1813 ]],#p15
[[ 958.51874, 1187.1802 ]],#p45
[[ 894.47974, 1163.0116 ]],#p13
[[ 889.74634, 1188.9985 ]],#p43
[[ 832.6407,  1162.851  ]],#p11
[[ 821.0164,  1188.3483 ]]#p41
],dtype=np.float64)

Rotation_matrix=np.array([[ 0.998915, -0.00823395, -0.04583641],
[-0.04174363, 0.27802867, -0.9596653 ],
[ 0.02064567, 0.9605375, 0.2773833 ]],dtype=np.float64)



Rot_mat_test=np.array([0.998915, -0.00823395, -0.04583641,-0.04174363, 0.27802867, -0.9596653 ,
 0.02064567, 0.9605375, 0.2773833],dtype=np.float64)

Rotation_vector=np.array([[ 1.2889872 ],
[-0.04462786],
[-0.02249426]], dtype=np.float64)


Translation_vector=np.array([[-0.21408474],
[ 1.4380101 ],
[ 1.008065  ]], dtype=np.float64)



Newcameramtx, roi = cv2.getOptimalNewCameraMatrix(Camera_matrix, Dist_coefficients, (1920, 1208), 1, (1920, 1208))
Img_points_undistored=cv2.undistortPoints(Img_points,Camera_matrix,Dist_coefficients,P=Camera_matrix)



World_points_offseted=np.array([
    [[0.255, 4.97,0]], #p15
    [[0.245,4.49,0]],#p45
    [[-0.014,4.98,0]],#p13
    [[0,4.50,0]],#p43
    [[-0.32, 4.995, 0]],#p11
    [[-0.32,4.52,0]]#p41
],dtype=np.float64)







def getRevAndTvec(worldpoints,input_img_points, camera_matrix,dist_coeff):
    zero_dist=np.array([0, 0, 0, 0, 0.000000], dtype=np.float64)
    boolFlag,rvecs, tvecs = cv2.solvePnP(worldpoints, input_img_points,camera_matrix , zero_dist,flags=0)

    print("r vecs")
    print(repr(rvecs))

    print("t Vecs")
    print(repr(tvecs))

    return rvecs,tvecs
    


def calculate_rot(rvec):
    rot_mat, _=cv2.Rodrigues(rvec)
    print(rot_mat)
    return rot_mat

def calculate_xyz(rot_mat,tvec,cam_mat, img_points,world_points):
    # Extrinsic Parameters Matrix
    translation_vector_transposed = np.transpose(tvec)
    #print(np.shape(translation_vector_transposed))
    #print(np.shape(rotation_matrix))

    extrinsic_matrix=np.column_stack((rot_mat, translation_vector_transposed[0]))



    # Projection Matrix
    projection_matrix = np.mat(Camera_matrix) * np.mat(extrinsic_matrix)
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
    homography_matrix = np.array([[p11,p12,p14], [p21,p22,p24], [p31,p32,p34]], dtype=np.float64)
    Homography_matrix_inverse = np.linalg.inv(homography_matrix)

    for idx,point in enumerate(img_points):
        #print(point)
        u=point[0][0]
        v=point[0][1]
        print(f"calculating for u={u},v={v}")

        uv=np.mat(np.transpose([u,v,1]))

        dunno=np.squeeze(np.asarray(Homography_matrix_inverse))
        uv_squeezed=np.squeeze(np.asarray(uv))
        point_3D_w = np.dot(dunno,uv_squeezed) 

        # Normalization
        point_3D = np.divide(point_3D_w,point_3D_w[2])
        p_x=point_3D[0]
        p_y=point_3D[1]
        print(f"x={p_x},x_offset={p_x-world_points[idx][0][0]},y={p_y}, y_offset={p_y-world_points[idx][0][1]}")

def reproject_points(w_points,rvec,tvec,cam_mx, dist_coef):
    
    print("Undistored Points")
    print(Img_points_undistored)
    print("Reprojecting and Drawing")
    imgpts, jac = cv2.projectPoints(w_points, rvec, tvec, cam_mx, dist_coef)

    
    print(imgpts)
    # print("Undistored")
    # print(imgpts_undistorted)





def new_shit(input_imgPoints,camera_mat, tvec, rot_mat,worldpoints):
 

    Extrincic=cv2.hconcat([rot_mat,tvec])
    Projection_mtx=camera_mat.dot(Extrincic)

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
        print(f"x={p_x},x_offset={p_x-worldpoints[idx][0][0]},y={p_y}, y_offset={p_y-worldpoints[idx][0][1]}")
        #projection_result.append([projected_Point[0]/projected_Point[2],projected_Point[1]/projected_Point[2]])

# def old_2dto3d(imgpoints,rot_mat,tvec,worldpoints):
#     undistortedAndNormalizedPointMatrix = cv2.undistortPoints(imgpoints, cameraMatrix=Camera_matrix, distCoeffs= Dist_coefficients)
#     projected_points = []
#     Extrinsic=cv2.hconcat([rot_mat,tvec])
#     Extrinsic_inv = np.delete(np.linalg.pinv(Extrinsic), 2, 1)
#         Rotation_vector = R.from_matrix(rot_mat)

#     Rotation_inv = np.linalg.inv(rot_mat)


#     #print(undistortedAndNormalizedPointMatrix)
#     print("Results old_shit")
#     #print(undistortedAndNormalizedPointMatrix.shape)
#     for idx, norm in enumerate(undistortedAndNormalizedPointMatrix):
#         #print(norm.shape)
#         #print(norm)
#         norm_expanded = np.array([[norm[0][0], norm[0][1], 1]])
#         #print(norm_expanded.shape)
#         rotated_ray = Rotation_inv.dot(norm_expanded.T)
#         #rotated_ray = norm_expanded * np.linalg.inv(Rotation_matrix) #* np.linalg.inv(Camera_matrix)
#         #print(rotated_ray.shape)

#         rotated_normalised_ray = rotated_ray #/ rotated_ray[0][2]
#         #print(rotated_ray)
#         #print(rotated_normalised_ray.shape)

#         xGround = rotated_normalised_ray[0]/rotated_normalised_ray[2] + Translation_vector[0]
#         yGround = rotated_normalised_ray[1]/rotated_normalised_ray[2] + Translation_vector[1]
#         #print(xGround.shape)
#         #print(f"xGround={xGround},yGround={yGround}")
#         print(f"x={xGround},x_offset={xGround-worldpoints[idx][0][0]},y={yGround}, y_offset={yGround-worldpoints[idx][0][1]}")
#         #projected_points.append((xGround,yGround,0.0))
 


def main(args=None):
    #print("Testing Method One")
    #print(Img_points_undistored)
    new_rvec,new_tvec=getRevAndTvec(World_points,Img_points_undistored,Camera_matrix,Dist_coefficients)
    new_rot_mat=calculate_rot(new_rvec)
    #print(Img_points_undistored)
    #new_shit(Img_points_undistored,Camera_matrix,new_tvec,new_rot_mat,World_points)
    calculate_xyz(new_rot_mat,new_tvec,Camera_matrix,Img_points_undistored,World_points)
    # print("Testing with distorted points")
    # new_rvec,new_tvec=getRevAndTvec(World_points_offseted,Img_points,Camera_matrix,Dist_coefficients)
    # new_rot_mat=calculate_rot(new_rvec)
    # new_shit(Img_points,Camera_matrix,new_tvec,new_rot_mat,World_points_offseted)
    
    

    # print("Other Method")
    # new_rvec,new_tvec=getRevAndTvec(World_points_offseted,Img_points_undistored,Camera_matrix,Dist_coefficients)
    # new_rot_mat=calculate_rot(new_rvec)
    # calculate_xyz(new_rot_mat,new_tvec,Camera_matrix,Img_points_undistored,World_points_offseted)

    # print("Ohne Rvec UNdistorted")
    # new_shit(Img_points_undistored,Camera_matrix,Translation_vector,Rotation_matrix,World_points)
    
    #print("distorted Projection")
    #new_shit(Img_points,Camera_matrix,Translation_vector,Rotation_matrix)
    #print(test)
    # print("First Method")
    # for idx,point in enumerate(img_points):
    #     #print(point)
    #     calculate_xyz(idx,u=point[0][0],v=point[0][1])

    # print("Second Method")
    # new_shit()
    # print("Reprojecting")
    # reproject_points(w_points=world_points,tvec=translation_vector,rvec=rotation_vector)

    print("Nikolovski")
    #old_2dto3d(Img_points,Rotation_matrix,Translation_vector,World_points)

if __name__ == '__main__':
  main()




