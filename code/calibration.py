import open3d as o3d

import cv2
import numpy as np

import ctypes
import winsound as sd
import time
import screeninfo as sinfo
import json

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


if __name__ == "__main__":
    kinect = o3d.io.AzureKinectSensor(o3d.io.AzureKinectSensorConfig())
    
    if not kinect.connect(0):
        raise RuntimeError("[FAIL] Failed to Connection to Sensor")
    
    # CV window setting
    screen_x, screen_y = sinfo.get_monitors()[-1].x, sinfo.get_monitors()[-1].y
    cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
    cv2.moveWindow("window", screen_x, screen_y)
    cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    print_info("[INFO] Created Window")
    
    
    
    # 1-Way Projection
    u = ctypes.windll.user32
    window_w, window_h = u.GetSystemMetrics(0), u.GetSystemMetrics(1)                
    one_way_image = np.ones((window_h, window_w, 3))* 255
    
    cv2.imshow("window", one_way_image), cv2.waitKey(1)
    cv2.imwrite("0_1-way.jpg", one_way_image)
    
    
    
    # 2-Way Capture
    crt = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    time.sleep(2) # Connection Ready
    rgbd = kinect.capture_frame(True)
    
    if rgbd == None:
        print("[WARNING] kincet is not connected")
        pass
    
    print_info("[INFO] 2-Way Capture")
    two_way_image = np.asarray(rgbd.color)[:,:,::-1]
    capture_h, capture_w, _ = two_way_image.shape
    cv2.imwrite("0_2-Way.jpg", two_way_image)
    
    
    
    # 3-Way Projection
    three_way_image = two_way_image.copy()
    grey = cv2.cvtColor(three_way_image, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.threshold(grey, 137, 255, cv2.THRESH_BINARY_INV)
    
    binary = cv2.blur(binary, (5, 5))
    binary = cv2.dilate(binary, np.ones((5, 5)))
    
    cv2.imwrite("binary.jpg", binary)
    
    contour, hrc = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    approxs = list()
    for cont in contour:
        poly = cv2.approxPolyDP(cont, cv2.arcLength(cont, True) * 0.02, True)
        if len(poly) == 4 and 1279 not in poly:
            approxs.append(poly)
            
    t = np.sum(approxs, axis=1)[:,:,0]
    i = np.argmin(np.abs(t - capture_w*2))
        
    cv2.drawContours(three_way_image, [approxs[i]], 0, (0,255,0), 3)
        
    print_info("[INFO] 3-Way Show")
    cv2.imshow("window", three_way_image)
    
    if cv2.waitKey(3000) & 0xFF == ord('q'):
        print_info("[TERMINATION] with Keyboard Interrupt")
        cv2.destroyAllWindows()

    cv2.destroyAllWindows()        
    cv2.imwrite("0_3-way.jpg", three_way_image)
    
       
    
    # Homography
    # [Top-Left], [Bottom-Left], [Bottom-Right], [Top-Right]
    p1, p2, p3, p4 = list(approxs[i][3][0]), list(approxs[i][0][0]),\
                     list(approxs[i][1][0]), list(approxs[i][2][0])
    
    point_src = [[int(p1[0]), int(p1[1])], [int(p2[0]), int(p2[1])],
                 [int(p3[0]), int(p3[1])], [int(p4[0]), int(p4[1])]]
    point_dst = [[0,0],[1920,0],[1920, 1080],[0, 1080]]
    
    matrix = {"PointSrc" : list(point_src),
              "PointDst" : list(point_dst)}        
    
    # Make Json Config File
    with open("calibration_config.json", "w") as json_file:
        json.dump(matrix, json_file)
        
