#!/usr/bin/env python3

import rospy

from ocs2_api.dummy import dummy_from_1
import ocs2_api.dummy2 as dummy2

from ocs2_api.dummy_folder.dummy3 import dummy_from_3

def main():
    rospy.init_node("dummy_node")
    dummy_from_1()
    
    dummy_from_3()
    # dummy2.dummy_from_2()
    
    print("Bye!")
    
if __name__ == '__main__':
    main()