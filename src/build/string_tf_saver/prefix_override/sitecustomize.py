import sys
if sys.prefix == '/home/benchmark/.mamba/envs/benchmark':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/benchmark/vicon_string_ws/src/install/string_tf_saver'
