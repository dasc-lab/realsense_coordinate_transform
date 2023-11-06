import pyrealsense2 as rs
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
from tf2_ros.transform_listener import TransformListener
from numpy.linalg import inv
from pyrealsense2 import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String

def rot_to_hom(rot_matrix, trans):

    '''

    rot_matrix: (3x3) np.ndarray
    trans: (3x1) np.ndarray

    '''
    hom_mat = np.eye(4)
    hom_mat[:3,:3] = rot_matrix
    hom_mat[:3,-1] = trans
    return hom_mat

def robot_to_world(hom_mat, coord):
    return np.mat_mul(hom_mat, coord)
def  world_to_drone(hom_mat, coord):
    '''

    hom_mat: (4x4) np.ndarray
    coord  : (4x1) np.ndarray
    
    '''
    ############## for testing ################
    distance = 2.86
    hom_mat2 = np.eye(4)
    hom_mat2[2,3] = -distance

    ###########################################
    # hom_mat2 = inv(hom_mat)
    drone_coord = (hom_mat2) @ coord
    return drone_coord

def drone_to_camera(drone_coord):
    '''
        drone -> cam

        H = [R t]
            [0 1]

    '''
    coord = drone_coord[0:3]
    mat = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    camera_coord = np.matmul(mat,coord)
    return camera_coord

# world_coordinates = tf2
# distance = tf2
r = R.from_quat([0,0,1,0.5])
rotate = r.as_matrix()
# hom_matrix = rot_to_hom(rot_matrix, distance)
# drone_ coordinates = world_to_camera(hom_matrix, world_coordinates)
# camera_coordinates = drone_to_camera(drone_coordinates)
# coordinates = (-0.2,-0.1,3)
# pipe = rs.pipeline()
# cfg = rs.config()
# cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
# cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,30)

# c = pipe.start(cfg)
# profile = c.get_stream(rs.stream.color)
# color_intr = profile.as_video_stream_profile().get_intrinsics()
# pixel_coordinates = np.floor(rs.rs2_project_point_to_pixel(color_intr, coordinates))
# print(pixel_coordinates)
# device = profile.get_device()
# align_to = rs.stream.color
# align = rs.align(align_to)

# while True:
#     frame = pipe.wait_for_frames()
#     depth_frame = frame.get_depth_frame()
#     color_frame = frame.get_color_frame()

#     depth_image = np.asanyarray(depth_frame.get_data())
#     color_image = np.asanyarray(color_frame.get_data())
#     #color_intrin = color_frame.as_video_stream_profile().intrinsics
    
#     cv2.imshow('rgb', color_image)
#     cv2.imshow('depth', depth_image)

#     if cv2.waitKey(1) == ord('q'):
#         break
# pipe.stop()

