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





def getRevAndTvec(worldpoints,img_points, camera_matrix,dist_coeff):
    boolFlag,rvecs, tvecs = cv2.solvePnP(worldpoints, img_points,camera_matrix , dist_coeff)

    imgpts, jac = cv.projectPoints(world_points, rvecs, tvecs, camera_matrix, dist_coeff)

    img = cv2.circle(img, (img_points[5][0][0],img_points[5][0][1]), radius=5, color=(0, 0, 255), thickness=-1)

    print("r vecs")
    print(repr(rvecs))

    print("t Vecs")
    print(repr(tvecs))
    


def calculate_rot(rvec):
    rot_mat, _=cv2.Rodrigues(rvec)
    print(rot_mat)

def main(args=None):
    getRevAndTvec(world_points,img_points,camera_matrix,dist_coefficients)

if __name__ == '__main__':
  main()

