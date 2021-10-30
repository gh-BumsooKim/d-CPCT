import cv2
#import open3d

#import numpy as np
import time

def MousePoint(event, x, y, flags, param):
    global frame
    global x1, x2, y1, x2
    global step, points
    
    if event == cv2.EVENT_LBUTTONDOWN:
        step += 1
        
        if step == 1:
            print('EVENT_LBUTTONDOWN: %d, %d' % (x, y))
            #x1, y1 = x, y
            points.append([x, y])
        elif step == 2:
            print('EVENT_LBUTTONDOWN: %d, %d' % (x, y))
            #x2, y2 = x, y
            points.append([x, y])
        else: pass
        
def main():
    # 
    ratio = 1
    width = 1280
    height = 720
    
    # zoom in and out
    _zoom = 0
    
    global points
    points = list()
    
    x1, x2, y1, y2 = 0, 0, 0, 0
    global step 
    step = 2
    
    rsz_width = int(1/ratio * width)
    rsz_height = int(1/ratio * height)
    
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    while True:
        
        start = time.time()
        
        ret, frame = capture.read()
        
        # calibration
        #crop = frame[height//2 - int(rsz_height//2): height//2 + int(rsz_height//2),
        #             width//2 - int(rsz_width//2): width//2 + int(rsz_width//2)]
        # 
        
        
        #crop = frame[y1: y2, x1: x2]
        #resize = cv2.resize(crop, (width, height))
        cv2.namedWindow("VideoFrame")
        cv2.setMouseCallback("VideoFrame", MousePoint)
        
        if step >= 2:
            try:
                #crop = frame[points[0][1]: points[1][1],
                #         points[0][0]: points[1][0]]
                
                _w_z = int(width * _zoom)
                _h_z = int(height * _zoom)
                
                
                print("Debug", _w_z, _h_z)
                
                crop = frame[_h_z : height - _h_z,
                             _w_z : width - _w_z]
                
                frame = cv2.resize(crop, (width, height))
                
                # Zoom Phase
                
                
            except:
                error_msg_crop = "Faild cv2.resize() function\n"
                print(error_msg_crop)
        
        try:
            cv2.imshow("VideoFrame", frame)
        except:
            error_msg = """Error : Camera is not connected
                  \rCannot execute cv2.imshow() function\n"""
            #make_log(time.time(), status, error_msg)
            print(error_msg)
            break
        
        end = time.time()
        print(round((end - start), 3), "sec \t", round(1/(end - start), 3), "fps")
        
        key = cv2.waitKey(33)
        if  key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('w'):
            step = 0
            points = list()
        # zoom in
        elif key & 0xFF == ord('d'):
            print("Zoom In")
            _zoom += 0.025
        # zoom out
        elif key & 0xFF == ord('a'):
            print("Zoom Out")
            _zoom -= 0.025
        
    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    pass
