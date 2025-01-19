import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raz/projects/ros2_advanced_concepts/ros2_ws/install/actions_py'
