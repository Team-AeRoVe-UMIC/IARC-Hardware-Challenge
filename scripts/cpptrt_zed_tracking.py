#!/usr/bin/env python3
from numpy.lib.function_base import angle
from copy import copy
import rospy
import numpy as np
import cv2
from time import sleep, time
import math
import transformation as trans
import yolo_trt as yolo
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from mlcv.msg import mlcv
from zed import ZED_output
from plane_srv_client import plane_client


import matplotlib.pyplot as plt

def calc(out):
    xyz = [out.x, out.y, out.z]
    coff = np.array([out.a, out.b, out.c, out.d])
    if coff[3] < 0:
        coff = -1*coff
    angle = trans.angle_with_z(coff[0:3])
    sense = coff[0]*coff[2]
    return xyz, angle, sense

class tracker_out:
    def __init__(self) -> None:
        #rospy.init_node('mlcvout', anonymous=True)

        self.mlcv_pub = rospy.Publisher('/mlcv/mlout', mlcv, queue_size = 10)

    def start_visualizer(self):
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def pub(self, detected, xyz, angle, sense):
        cvout = mlcv()
        cvout.detected = detected
        cvout.x = xyz[0]
        cvout.y = xyz[1]
        cvout.z = xyz[2]
        cvout.angle = angle
        cvout.sense = sense

        self.mlcv_pub.publish(cvout)
        
        

def tracker_yolo(camera, tiny = False):
    
    TO = tracker_out() 
    trt_engine = yolo.trt_yolo(tiny = tiny)
    rate = rospy.Rate(20)
    print("Plane segmentation started")
    plane_segmentation = plane_client(full_cloud = True)
    print('CV loaded to be used')
    print('getting started')
    size = camera.img_bgr_left[...,::-1].shape[:2]
    result = cv2.VideoWriter('bag bag 2.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, (size[1], size[0]))
    while True:
        t0 = time()
        bgr = np.array(camera.img_bgr_left[...,::-1]).copy()
        depth = copy(camera.depth_data.data)
        ret,x1,y1,x2,y2=trt_engine(bgr)
        if ret == True:
            yolo_out = cv2.rectangle(bgr.astype(np.uint8).copy() ,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),3)
            x1_new = int((2*x1 + x2)/3)
            x2_new = int((x1 + 2*x2)/3)

            y2_new = int((y1 + 5*y2)/6)
            y1_new = int((2*y1 + 4*y2)/6)

            yolo_out = cv2.rectangle(yolo_out ,(int(x1_new),int(y1_new)),(int(x2_new),int(y2_new)),(0,255,255),3)

            # out = plane_segmentation(depth, camera.depth_data.height, camera.depth_data.width,camera.cx, 
            # camera.cy, camera.fx, camera.fy,int(x1) - 400,int(y1)-30,int(x2)+500,int(y2)+400, debug = True, find_plane = False)
            out = plane_segmentation(depth, camera.depth_data.height, camera.depth_data.width,camera.cx, 
            camera.cy, camera.fx, camera.fy,x1_new ,y1_new ,x2_new ,y2_new, debug = True, find_plane = True)
            print(out)


            xyz = [0, 0, 0]
            angle = 0
            sense = 0
            if out[0]:
                if out[1].if_found:
                    xyz, angle, sense = calc(out[1])
                    TO.pub(1, xyz, angle, sense)  
                    print("The angle is:",angle)
                    print("Sense is:", sense)
                else:
                    print('Error in plane finding')
                    # TO.pub(0, xyz, angle, sense)

            else:
                print('Error in plane finding')
                # TO.pub(0, xyz, angle, sense)
            
            # avg_z = np.sum(pc.T[2])/pc.T[2].shape[0]
            # x_center = (center[0] - bgr.shape[1]/2)*avg_z/Fx
            # y_center = (center[1] - bgr.shape[0]/2)*avg_z/Fy
            # xyz = [x_center, y_center, avg_z]
            # try:
            #     sense = line.best_line(pc_sense)
            #     sense = -1*sense[1]/sense[0]
            #     m = plane.face_vector(pc)
            #     a ,b, c, d = m
            #     angle = trans.angle_with_z([a, b ,c])
            #     #print(angle)
            #     #print(sense)
            #     TO.pub(1, xyz, angle, sense)
            # except:
            #     print('Error in plane finding '+ str(round(error_count/total_count, 3)) + ' kx: ' + str(kx) + ' ky: ' + str(ky))
            #     error_count += 1
            #     TO.pub(0, xyz, angle, sense)
            # try:
            #     sense, best_fit_found = line.best_line(pc_sense)
            #     if best_fit_found:
            #         sense = -1*sense[1]/sense[0]
            #     else:
            #         sense = sense0

            #     m, best_fit_found = plane.face_vector(pc)
            #     if best_fit_found:
            #         a ,b, c, d = m
            #         angle = trans.angle_with_z([a, b ,c])
            #     #print(angle)
            #     #print(sense)
            #     sense0 = sense
            #     TO.pub(1, xyz, angle, sense)
            # except:
            #     print('Error in plane finding')
            #     TO.pub(0, xyz, angle, sense)
            
            #print(xyz)
  
            
                
            # # Capture frames in the video
            # ret, frame = cap.read()
        
            # # describe the type of font
            # # to be used.

            # dimensions of video
            # img = cv2.imread(yolo_out,0)
            

            

            #truncating xyz co-ordinates to 2 decimal places
            for i in range(len(xyz)):
                xyz[i] = round(xyz[i], 2)
            sense=round(sense,2)
            angle=round(angle,2)

            font = cv2.FONT_HERSHEY_SIMPLEX
        
            # Use putText() method for
            # inserting text on video
            cv2.putText(yolo_out, 
                        str(sense), 
                        (50, 50), 
                        font, 1, 
                        (0, 0, 255), 
                        2, 
                        cv2.LINE_4)

            cv2.putText(yolo_out, 
                        str(angle), 
                        (50, 100), 
                        font, 1, 
                        (0, 0, 255), 
                        2, 
                        cv2.LINE_4)


            cv2.putText(yolo_out, 
                        str(xyz), 
                        (50, 150), 
                        font, 1, 
                        (0, 0, 255), 
                        2, 
                        cv2.LINE_4)
            
            
            # out.write(yolo_out)
        
            # Display the resulting frame
            cv2.imshow('YOLO Output', yolo_out)
            result.write(yolo_out)
        
            
        else:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(bgr, 
                        "Not Detected", 
                        (50, 50), 
                        font, 1, 
                        (0, 0, 255), 
                        2, 
                        cv2.LINE_4)
            cv2.imshow('YOLO Output', bgr)
            result.write(bgr)

        

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
        t1 = time()

        print(str(1/(t1 - t0)) + ' fps output')
    cv2.destroyAllWindows()
        





















if __name__ == '__main__':
    rospy.init_node('mlcvout_trtcpp', anonymous=True)
    tiny = False
    try:
        camera = ZED_output()
    except rospy.ROSInterruptException:
            pass

    sleep(5)
    
    tracker_yolo(camera, tiny= tiny)
    
