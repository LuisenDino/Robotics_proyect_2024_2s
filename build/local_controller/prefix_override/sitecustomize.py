import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/luisen/Documents/Universidad/Robotica/proyect_ws/install/local_controller'
