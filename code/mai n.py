import cv2
#import open3d sa o3d
#import numpy as np
import time

def mouse_function(event, x, y, flags, param) -> None:
    global frame
    global step, points
    
    # cropWindow: function()
    if event == cv2.EVENT_LBUTTONDOWN:
        step += 1
        
        if step < 3:
            print('EVENT_LBUTTONDOWN: %d, %d' % (x, y))
            points.append([x, y])
        else:
            pass
        
def main():
    _ratio = 1
    _width = 1280
    _height = 720
    
    global points
    points = list()
    
    x1, x2, y1, y2 = 0, 0, 0, 0
    global step 
    step = 0
    
    rsz_width = int(1/_ratio * _width)
    rsz_height = int(1/_ratio * _height)
    
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, _width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, _height)
    
    while True:
        start = time.time()
        
        ret, frame = capture.read()
        cv2.namedWindow("VideoFrame")
        cv2.setMouseCallback("VideoFrame", mouse_function)
        
        if step >= 2:
            try:
                crop = frame[points[0][1]: points[1][1],
                         points[0][0]: points[1][0]]
                frame = cv2.resize(crop, (_width, _height)) # mapping
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
        
        # keyboard interrupt
        key = cv2.waitKey(33)
        if  key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('w'):
            step = 0
            points = list()
        
    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
