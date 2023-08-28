"""!
Class to represent the camera.
"""

import cv2
import time
import copy  # temp
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError


class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.VideoFrameWarped = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720, 1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720, 1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([])
        self.block_loc = {}
        self.block_loc_xyz =[]
        self.H = 1

        # mouse clicks & calibration variables
        self.cameraCalibrated = False
        # Factory intrinsic matrix
        self.intrinsic_matrix = np.array([[908.3550415039062, 0.0, 642.5927124023438], \
                                        [0.0, 908.4041137695312, 353.12652587890625], \
                                        [0.0, 0.0, 1.0]])
        self.distortion_matrix = np.array([0.17931543290615082, -0.5406785011291504, \
            -0.0007807965739630163, -0.0004374352574814111, 0.4746035635471344])
        self.extrinsic_matrix = np.eye(4)
        self.last_click = np.array([0, 0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-450, 500, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points))
        self.tag_detections = np.array([])
        self.tag_locations = [[-250, -25], [250, -25], [250, 275]]
        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            if self.cameraCalibrated:
                frame = cv2.warpPerspective(frame, self.H, (frame.shape[1], frame.shape[0]))
                self.VideoFrameWarped = frame
                # if True:
                    
                #     frame = self.blockDetector()
                # self.VideoFrame = frame
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            if self.cameraCalibrated:
                frame = cv2.warpPerspective(frame, self.H, (frame.shape[1], frame.shape[0]))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  Points in coordinate frame 1
        @param      coord2  Points in coordinate frame 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        print(cv2.getAffineTransform(pts1, pts2))
        return cv2.getAffineTransform(pts1, pts2)

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

                    TODO: use this to load in any calibration files you need to

        @param      file  The file
        """
        pass

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        #load image frame and convert to HSV=
        rbg_im = copy.deepcopy(self.VideoFrameWarped)
        np.save("block_img",rbg_im)
        # np.save("outfile", rbg_im)
        font = cv2.FONT_HERSHEY_TRIPLEX
        #color list in HSV
        colors = list((
            {'id': 'red',       'lower': (0, 50, 50),'upper': (3, 255, 255),'lower1': (170, 50, 50),'upper1': (180, 255, 255)},
            {'id': 'orange',    'lower': (3, 50, 50),'upper': (15, 255, 255)},
            {'id': 'yellow',    'lower': (18, 100, 50),'upper': (30, 255, 255)},
            {'id': 'green',     'lower': (65, 50, 50),'upper': (85, 255, 255)},
            {'id': 'blue',      'lower': (100, 130, 50),'upper': (110, 255, 255)},
            {'id': 'violet',    'lower': (111, 25, 25),'upper': (160, 255, 255)})
        )
        self.block_loc = {'red_s':[],'orange_s':[],'yellow_s':[],'green_s':[],'blue_s':[],'violet_s':[],
                          'red_b':[],'orange_b':[],'yellow_b':[],'green_b':[],'blue_b':[],'violet_b':[]}
        colors_mask = []
        #create mask that removes outside of the board and the robot arm
        mask = np.zeros_like(rbg_im[:,:,0])
        cv2.rectangle(mask,(130,25),(1151,678),255,cv2.FILLED)
        cv2.rectangle(mask,(567,425),(720,720),0,cv2.FILLED)

        rbg_im = cv2.bitwise_and(rbg_im,rbg_im,mask=mask)
        np.save("mask_img",rbg_im)
        hsv_im = cv2.cvtColor(rbg_im, cv2.COLOR_RGB2HSV)
        for i in range(len(colors)):
            color = colors[i]['id']
            
            color_mask = cv2.inRange(hsv_im,colors[i]['lower'],colors[i]['upper'])
            if colors[i]['id']=='red':
                color_mask2 = cv2.inRange(hsv_im,colors[i]['lower1'],colors[i]['upper1'])
                color_mask = color_mask+color_mask2
            color_mask = cv2.medianBlur(color_mask,11)
            _,contours,_ = cv2.findContours(color_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE )
            colors_mask.append(color_mask)
            for contour in contours:
                
                theta = cv2.minAreaRect(contour)[2]
                M = cv2.moments(contour)
                if M['m00'] > 450:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.putText(self.VideoFrameWarped, color, (cx-30, cy+40), font, 1.0, (0,0,0), thickness=2)
                    # cv2.putText(self.VideoFrameWarped, str(theta)[0:4], (cx, cy), font, 1.0, (0,0,0), thickness=2)
                    cv2.drawContours(self.VideoFrameWarped, contour, -1, (255,255,255), 3)
                    # print( M['m00'])
                    if M['m00'] <1200:
                        self.block_loc[str(color)+'_s'].append((cx,cy,theta))
                        
                    else:
                        self.block_loc[str(color)+'_b'].append((cx,cy,theta))
            self.convert_uv_to_xyz_block()
            print(self.block_loc_xyz)
        return self.VideoFrameWarped

    def convert_uv_to_xyz_block(self):
        self.block_loc_xyz = []
        for block_type in self.block_loc.keys():
            for block in self.block_loc[block_type]:
                xyz = self.apriltag_pixel_to_world(block[0],block[1])
                dist = (xyz[0]**2+xyz[1]**2)**0.5
                color = block_type[:-2]
                if block_type[-1] == "s":
                    size = "small"
                elif block_type[-1] == "b":
                    size = "big"
                self.block_loc_xyz.append([dist,(xyz[0],xyz[1],xyz[2],block[2],color,size)])
        self.block_loc_xyz.sort()             


    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """
        pass
    
    def apriltag_pixel_to_world(self, x_pos, y_pos):
        # Mouse tracking with apriltag calculation
        pixel_coords_orig = np.array([[x_pos], [y_pos], [1]])
        
        pixel_coords_homography = np.matmul(np.linalg.inv(self.H), pixel_coords_orig)
        pixel_coords = pixel_coords_homography * (1 / pixel_coords_homography[2])
        x_coord = int(pixel_coords[0])  # margin of error with casting
        y_coord = int(pixel_coords[1])  # might be a better idea to apply homography transf to depth camera
        z_c = self.DepthFrameRaw[y_coord][x_coord]
        camera_coords = z_c * np.linalg.inv(self.intrinsic_matrix) #this is actually K matrix
        camera_coords = np.matmul(camera_coords, pixel_coords)
        homo_camera_coords = np.append(camera_coords, [[1]])
        
        homo_world_coords = np.matmul(np.linalg.inv(self.extrinsic_matrix),homo_camera_coords)       
        
        world_coords = homo_world_coords[:-1]
        return world_coords
    
    def apriltag_calibration(self):
        success = False
        rot_vec = None
        trans_vec = None
        model_points = np.array([[-250, -25, 0],
                                 [250, -25, 0],
                                 [250, 275, 0],
                                 [-250, 275, 0]],dtype=np.float32)
        # ,
        #                          [-375,50,152],
        #                          [375,-100,152],
        #                          [-475,400,152],
        #                          [125,350,241]  # 1, 2, 3, 4, 5, 6, 7, 8
        data = self.tag_detections
        apriltag_camera_pts = []
        for detection in data.detections:
            pose = detection.pose.pose.pose.position
            I0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
            pixels = (1.0/pose.z)*np.matmul(np.matmul(self.intrinsic_matrix,I0),np.array([pose.x,pose.y,pose.z,1]))[:2]
            apriltag_camera_pts.append([detection.id[0],pixels])
            
        apriltag_camera_pts.sort()
        image_points = np.array([pt[1] for pt in apriltag_camera_pts])
        try:
            (success, rot_vec, trans_vec) = cv2.solvePnP(model_points, image_points, self.intrinsic_matrix, self.distortion_matrix,flags=cv2.SOLVEPNP_ITERATIVE)
            rot_mat,_ = cv2.Rodrigues(rot_vec)
            self.extrinsic_matrix = np.r_[np.c_[rot_mat, trans_vec], [[0, 0, 0, 1]]]
        except:
            print("APRIL TAGS NOT PLACED CORRECTLY. FIX AND RECALIBRATE.")
        # print(success)
        return success, rot_vec, trans_vec
    
    
    # OBSOLETE: Naive, measured approach
    def compute_extrinsic_naive(self):
        # extrinsic matrix calculation with measured data
        homo_trans = np.array([[1, 0, 0, 0 + 25], 
                               [0, -1, 0, 384 - 210+20],
                               [0, 0, -1, 984 + 55],
                               [0, 0, 0, 1]])
        theta = 0.19
        rotation_gyro_x = np.array([[1, 0, 0, 0],
                                    [0, np.cos(theta), -np.sin(theta), 0],
                                    [0, np.sin(theta), np.cos(theta), 0],
                                    [0, 0, 0, 1]])
        theta1 = -0.01
        rotation_gyro_y = np.array([[np.cos(theta1), 0, np.sin(theta1), 0],
                                    [0, 1,0 , 0],
                                    [ -np.sin(theta1),0, np.cos(theta1), 0],
                                    [0, 0, 0, 1]])
        rotation_gyro = np.matmul(rotation_gyro_x,rotation_gyro_y)
        world_to_image = np.matmul(homo_trans,rotation_gyro)  # extrinsic
        return world_to_image

    def helper_pixel_to_world(self, x_pos, y_pos, z_c):
        #intrinsic from calibration
        intrinsic_mat = np.array([[916.281372, 0.000000, 646.431511],
                                  [0.000000, 917.909180, 361.799174],
                                  [0.000000, 0.000000, 1.000000]]) 
        pixel_coords = np.array([[x_pos], [y_pos], [1]])
        camera_coords = z_c * np.linalg.inv(intrinsic_mat) #this is actually K matrix
        camera_coords = np.matmul(camera_coords, pixel_coords)
        homo_camera_coords = np.append(camera_coords, [[1]])
        world_to_image = self.compute_extrinsic_naive()
        homo_world_coords = np.matmul(np.linalg.inv(world_to_image),homo_camera_coords)       
        
        world_coords = homo_world_coords[:-1]
        return world_coords

    def projectGridInRGBImage(self):
        """!
        @brief      projects

                    TODO: Use the intrinsic and extrinsic matricies to project the gridpoints 
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame and
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
        """
        self.GridFrame = copy.deepcopy(self.VideoFrame)
        I0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
        intrinsic_mat_homo = np.matmul(self.intrinsic_matrix,I0)
        for i in range(self.grid_points.shape[1]):
            for j in range(self.grid_points.shape[2]):
                x_coord = self.grid_points[0][i][j]
                y_coord = self.grid_points[1][i][j]
                world_coords = np.array([[x_coord],[y_coord],[0],[1]])
                
                camera_coords = np.matmul(self.extrinsic_matrix, world_coords)
                z = camera_coords[2]
                pixel_coords = np.matmul(intrinsic_mat_homo, camera_coords)
                final_pixel_coords = pixel_coords[:-1]
                if z:
                    final_x_coord = final_pixel_coords[0]/z
                    final_y_coord = final_pixel_coords[1]/z
                    cv2.circle(self.GridFrame, (final_x_coord,final_y_coord), 5, (255, 0, 0), -1)

    def calculate_homography_matrix(self):        
        data = self.tag_detections
        apriltag_camera_pts = []
        for detection in data.detections:
            pose = detection.pose.pose.pose.position
            I0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
            pixels = 1/pose.z*np.matmul(np.matmul(self.intrinsic_matrix,I0),np.array([pose.x,pose.y,pose.z,1]))[:2]
            apriltag_camera_pts.append([detection.id[0],pixels])
            
        apriltag_camera_pts.sort()
        image_points = np.array([pt[1] for pt in apriltag_camera_pts[:4]]).reshape(4,2)
        # Using april tags instead of the corners
        x = 640
        y = 250
        model_points = np.array([[x-250, y+275],
                                 [x+250, y+275],
                                 [x+250, y-25],
                                 [x-250, y-25]
                                 ])
        src_coords = image_points
        # # use the cv2 transform
        self.H = cv2.findHomography(src_coords, model_points)[0]
        



class ImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image


class TagImageListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.TagImageFrame = cv_image


class TagDetectionListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, AprilTagDetectionArray,
                                        self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.tag_detections = data
        #for detection in data.detections:
        #print(detection.id[0])
        #print(detection.pose.pose.pose.position)


class CameraInfoListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.tag_sub = rospy.Subscriber(topic, CameraInfo, self.callback)
        self.camera = camera

    def callback(self, data):
        self.camera.intrinsic_matrix = np.reshape(data.K, (3, 3))
        #print(self.camera.intrinsic_matrix)


class DepthListener:
    def __init__(self, topic, camera):
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            #cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        #self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)

    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        while True:
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            # TODO: fix this
            self.camera.projectGridInRGBImage()
            grid_frame = self.camera.convertQtGridFrame()
            if ((rgb_frame != None) & (depth_frame != None)):
                self.updateFrame.emit(
                    rgb_frame, depth_frame, tag_frame, grid_frame)
            time.sleep(0.03)
            if __name__ == '__main__':
                cv2.imshow(
                    "Image window",
                    cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow(
                    "Tag window",
                    cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                cv2.imshow("Grid window",
                    cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(3)
                time.sleep(0.03)


if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
