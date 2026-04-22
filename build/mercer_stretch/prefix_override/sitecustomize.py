import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hello-robot/ament_ws/src/mercer_stretch/install/mercer_stretch'
