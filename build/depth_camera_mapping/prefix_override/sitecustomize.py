import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abd/rosbot_ws/src/2d_mapping/install/depth_camera_mapping'
