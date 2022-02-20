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
import open3d as o3d # for MS Azure Kinect

def print_info(msg: str, **kwargs) -> None:
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
    
    if STATUS.upper() == 'VIEW':
        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        while True:            
            ret, frame = cap.read()
            
            if ret == False:
                print_info("[FAIL] Camera Connection Failed", info='fail')
                break
            
            binary = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, binary = cv2.threshold(binary, 127, 255, cv2.THRESH_BINARY_INV)
            
            contour, hrc = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #print(len(contour), hrc)
            #for i, cont in enumerate(contour):
            cv2.drawContours(frame, contour, -1, (0, 255, 0), 3)
        
            cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
            cv2.moveWindow("window", 0, 0)
            cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow("window", frame)
                        
            if cv2.waitKey(33) & 0xFF == ord('q'):
                print_info("[TERMINATION] with Keyboard Interrupt")
                cv2.destroyAllWindows(), cap.release()
                break
    else:
        #kinect = o3d.io.AzureKinectSensor(o3d.io.AzureKinectSensorConfig())
        
        #if not kinect.connect(0):
        #    raise RuntimeError("[FAIL] Failed to connect to sensor")
        
        crt = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        # 2. Webcam
        #u = ctypes.windll.user32
        #wdw_w, wdw_h = u.GetSystemMetrics(0), u.GetSystemMetrics(1)
        
        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        print_info("[INFO] Created Window")
        
        error = 10.0
        
        # 1-way 
        ret, frame = cap.read()
        if ret == False:
            print_info("[FAIL] Camera Connection Failed", info='fail')
            raise RuntimeError("[ERROR TERMINATION]")
        
        print_info("[INFO] 1-way Captured, and Synchronization")
        time.sleep(2)
        print_info("[INFO] 1-way Sleep Down")   
        
        #screen_num = len(sinfo.get_monitors())
        screen_x, screen_y = sinfo.get_monitors()[-1].x, sinfo.get_monitors()[-1].y
        
        # 2. Webcam
        cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
        cv2.moveWindow("window", screen_x, screen_y)
        cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        print_info("[INFO] Set Window")  
        
        # 2-way
        cv2.imshow("window", frame)
        cv2.waitKey(33)
        print_info("[INFO] 2-way Shown, and Synchronization")
        time.sleep(2)
        cv2.imwrite("calibration/1-way.jpg", frame)
        print_info("[INFO] 3-way Captued, and Synchronization") 
        time.sleep(2)
        
        # 3-way
        ret, frame = cap.read()
        cv2.imwrite("calibration/3-way.jpg", frame)
        print_info("[INFO] Done")
        
        cap.release()
        cv2.destroyAllWindows()
        print_info("[TERMINATION]")
        
        # Calibration
        
        #image = cv2.imread("calibration/3-way.jpg")
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
        
        #if ret == True:
        #    corner = cv2.cornerSubPix(gray, corners, (9, 6), (-1, -1), crt)
        #    frame = cv2.drawChessboardCorners(frame, (9, 6), corner, ret)
        
        """
        while error > 1.0:
            # Image Input
            rgbd = kinect.capture_frame(0)    
            if rgbd is None:
                continue
            
            # 2. Webcam
            ret, frame = cap.read()
        
            sd.Beep(2000, 100)
            # Calibration using ChessBoard
        """
        
        # Make Json Config File
        pass

if __name__ == '__main__':
    # argparser
    calibration(argparse.ArgumentParser())
