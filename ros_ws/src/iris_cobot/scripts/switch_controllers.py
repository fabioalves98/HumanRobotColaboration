#!/usr/bin/env python
import rospy

import cobot.helpers as helpers

def main():
    print(helpers.switchControllers())


if __name__ == "__main__":
    main()