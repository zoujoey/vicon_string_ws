import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zoujoey/LSY_Projects/vicon_strings_ws/install/vicon_string_obstacles'
