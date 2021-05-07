import cv2
import numpy as np
from geometry_msgs import msg as gmsg

def depths_to_points(self, camera_info, u, v, d):
    """
    Back-project 2D image points to 3D space using depths
    """
    npoints = len(d)
    xy = cv2.undistortPoints(np.expand_dims(np.array(list(zip(u, v)),
                                                     dtype=np.float32),
                                            axis=0),
                             np.array(camera_info.K).reshape((3, 3)),
                             np.array(camera_info.D))
    xy = xy.ravel().reshape(npoints, 2)
    return [ gmsg.Point(xy[i, 0]*d[i], xy[i, 1]*d[i], d[i])
             for i in range(npoints) ]
