## ToDo
## A. Automatic View Window Mapping
## B. FPS Measurement

import os
import json # for Make Calibration Config File
import argparse

import winsound as sd
import time
import ctypes
import screeninfo as sinfo

import cv2
import numpy as np
import open3d as o3d # for MS Azure Kinect

global ctn

def print_info(msg: str, **kwargs) -> None:
    """
    msg :
    \tstring value for standard print like print() function        
    
    kwargs :
    \tinfo = "fail" -> means that semantic info if any conntection is failed
    \t     =  None  -> means all other info for default
    
    return : None
    """
    s: int = 2000
    
    for key, value in kwargs.items():
        #if 'info' in kwargs.keys():
        if key.lower() == 'info' and value.lower() == 'fail':
            s = 3000
            
    print(msg)
    sd.Beep(s, 100)    

def calibration(args: argparse.ArgumentParser) -> None:
    
    if not os.path.isdir("calibration"):
        os.mkdir("calibration")
    
    # Status
    # ├┬1. View Mode        (validate viewing)
    # │└─ # STATUS = VIEW
    # └┬2. Calibration Mode (make config file)
    #  └─ # STATUS = CALB
    
    STATUS = 'VIEW' # To be replaced to argparse
    
    
    # ----------------------------------------------------------------------- #
    
    
    ### 0. Camera Connection and Window Settings
    ## 0-1. Kinect Config
    kinect = o3d.io.AzureKinectSensor(o3d.io.AzureKinectSensorConfig())
    
    if not kinect.connect(0):
        raise RuntimeError("[FAIL] Failed to Connection to Sensor")
    
    print_info("[INFO] Connected to Sensor")
    
    ## 0-2. Window Properties
    #screen_num = len(sinfo.get_monitors())
    screen_x, screen_y = sinfo.get_monitors()[-1].x, sinfo.get_monitors()[-1].y
    cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
    cv2.moveWindow("window", screen_x, screen_y)
    cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    print_info("[INFO] Created Window")
    
    
    # ----------------------------------------------------------------------- #
    
    
    ### 1. Operating Mode
    ## 1-1. Camera View Mode 
    if STATUS.upper() == 'VIEWe':
        
        while True:
            rgbd = kinect.capture_frame(True)

            if rgbd == None:
                continue

            frame = np.asarray(rgbd.color)[:,:,::-1]
            cv2.imshow("window", frame)
                        
            if cv2.waitKey(33) & 0xFF == ord('q'):
                print_info("[TERMINATION] with Keyboard Interrupt")
                cv2.destroyAllWindows()#, cap.release()
                break
            
    ## 1-2. Calibration Mode        
    else:
        
        # 2-1. 1-Way Projection
        u = ctypes.windll.user32
        window_w, window_h = u.GetSystemMetrics(0), u.GetSystemMetrics(1)                
        one_way_image = np.ones((window_h, window_w, 3))* 255  
        #cv2.rectangle(one_way_image, (0, 0), (window_w, window_h),
        #              color=(0, 255, 0), thickness=50)
        
        cv2.imshow("window", one_way_image)
        cv2.waitKey(1)
        cv2.imwrite("0_1-way.jpg", one_way_image)
        
        # 2-2. 2-Way Capture
        crt = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        time.sleep(2) # Connection Ready
        rgbd = kinect.capture_frame(True)
        
        if rgbd == None:
            print("[WARNING] kincet is not connected")
            pass
        
        print_info("[INFO] 2-Way Capture")
        two_way_image = np.asarray(rgbd.color)[:,:,::-1]
        cv2.imwrite("0_2-Way.jpg", two_way_image)
        
        
        # 2-3. 3-Way Projection
        three_way_image = two_way_image.copy()
        grey = cv2.cvtColor(three_way_image, cv2.COLOR_BGR2GRAY)
        ret, binary = cv2.threshold(grey, 137, 255, cv2.THRESH_BINARY_INV)
        
        binary = cv2.blur(binary, (5, 5))
        binary = cv2.dilate(binary, np.ones((5, 5)))
        
        cv2.imwrite("binary.jpg", binary)
        
        contour, hrc = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print(len(contour), hrc)
        #for i, cont in enumerate(contour):
        
        #cv2.drawContours(three_way_image, contour, -1, (0, 0, 255), 3)
    
        #print("ddddddddddddd", contour)
        global ctn
        ctn = contour
        
        rect = cv2.minAreaRect(ctn[1])
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print("Point", box)
        cv2.drawContours(three_way_image, [box], 0, (0,255,0), 3)
        
        # Homography
        # [Top-Left], [Bottom-Left], [Bottom-Right], [Top-Right]
        point_src = box
        point_dst = [[0,0],[1920,0],[],[]]
        
        # 22-03-18
        # ToDo : Image Transformation
        #
        
        cv2.imshow("window", three_way_image)

        if cv2.waitKey(0) & 0xFF == ord('q'):
            print_info("[TERMINATION] with Keyboard Interrupt")
            cv2.destroyAllWindows()#, cap.release()
            
        cv2.imwrite("0_3-way.jpg", three_way_image)
        
        # Make Json Config File
        pass

if __name__ == '__main__':
    # argparser
    calibration(argparse.ArgumentParser())
