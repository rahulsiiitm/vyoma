"""Keep ROS regression publishers out of the running simulator's DDS domain."""

import os


os.environ['ROS_DOMAIN_ID'] = os.environ.get('VYOMA_TEST_DOMAIN_ID', '196')
