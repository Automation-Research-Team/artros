#!/usr/bin/env python
import cv2
import numpy as np
import sys
import nep
import time

id_ = 8
x_resolution = 1920
y_resolution = 1080
vflip = 0
hflip = 0

node = nep.node('camera_web_py/script')
conf = node.hybrid("163.220.51.108")
pub_image = node.new_pub('camera_web/image','image',conf)


try:
    print (sys.argv[1])
    id_ = int(sys.argv[1])
    print("Camera id:" + str(id_))
    x_resolution = int(sys.argv[2])
    print("X:" + str(x_resolution))
    y_resolution = int(sys.argv[3])
    print("Y:" + str(y_resolution))
    vflip = int(sys.argv[4])
    print("flip:" + str(vflip))
    hflip = int(sys.argv[5])
    print("flip:" + str(hflip))
    sys.stdout.flush()


except:
    pass


try:

    print ("waiting service")
    sys.stdout.flush()
    video = cv2.VideoCapture(id_)
    print ("service started")
    sys.stdout.flush()

    video.set(cv2.CAP_PROP_FRAME_WIDTH, x_resolution)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, y_resolution)
    time.sleep(1)



    while True:
        success, frame = video.read()
        if (vflip == 1):
            frame = cv2.flip(frame,1)
        if (hflip == 1):
            frame = cv2.flip(frame,0)
        pub_image.publish(frame)
        time.sleep(.001)


except:
    # Used in the interface - not erase
    video.release()
    cv2.destroyAllWindows()
    time.sleep(1)
    pass
