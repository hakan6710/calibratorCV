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

def reproject_points(w_points,img_points,rvec,tvec,cam_mx, dist_coef):
    
    print("Undistored Points")
    print(Img_points_undistored)
    print("Reprojecting and Drawing")
    projected_imgpts, jac = cv2.projectPoints(w_points, rvec, tvec, cam_mx, dist_coef)

    print("Projected Points")
    print(projected_imgpts)

    error_x=0
    error_y=0

    for idx,p_img in enumerate(projected_imgpts):
        print(p_img)
        print(img_points[idx])

        x_diff=(img_points[idx][0][0]-p_img[0][0])**2
        y_diff=(img_points[idx][0][1]-p_img[0][1])**2
        print(x_diff)
        print(y_diff)

    print(error_x)
    print(error_y)

    # print("Undistored")
    # print(imgpts_undistorted)





def calculate_projection(input_imgPoints,camera_mat, tvec, rot_mat,worldpoints):
    
  
    Extrincic=cv2.hconcat([rot_mat,tvec])
    Projection_mtx=camera_mat.dot(Extrincic)
    print(Projection_mtx)
    #delete the third column since Z=0 
    Projection_mtx = np.delete(Projection_mtx, 2, 1)
    print(repr(Projection_mtx))
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
        
        p_x=projected_Point[0]/projected_Point[2]
        p_y=projected_Point[1]/projected_Point[2]
        print(f"x={p_x},x_offset={p_x-worldpoints[idx][0][0]},y={p_y}, y_offset={p_y-worldpoints[idx][0][1]}")
        #projection_result.append([projected_Point[0]/projected_Point[2],projected_Point[1]/projected_Point[2]])




def main(args=None):
    #print("Testing Method One")
    print(Img_points_undistored)
    new_rvec,new_tvec=getRevAndTvec(World_points,Img_points_undistored,Camera_matrix,Dist_coefficients)
    new_rot_mat=calculate_rot(new_rvec)

    #reproject_points(World_points,Img_points,new_rvec,new_tvec,Camera_matrix,Dist_coefficients)

    calculate_projection(Img_points_undistored,Camera_matrix,new_tvec,new_rot_mat,World_points)

    

if __name__ == '__main__':
  main()




