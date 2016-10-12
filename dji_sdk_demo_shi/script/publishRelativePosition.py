#!/usr/bin/env python

import rospy
import dji_sdk.msg
from geometry_msgs.msg import Twist

class publisher():
    def init(self):
        self.local_position = dji_sdk.msg.LocalPosition()
        self.local_position.x = 0
        self.local_position.y = 0
        self.pub = rospy.Publisher('relativePosition', Twist, queue_size = 10)
        self.sub = rospy.Subscriber("dji_sdk/local_position", dji_sdk.msg.LocalPosition, self.local_position_subscriber_callback)

    def local_position_subscriber_callback(self, localPosition):
        self.local_position = localPosition

    def run(self):
        rospy.init_node('publishRelativePosition', anonymous=True)
        rate = rospy.Rate(50)
        goal_x = 40.0
        goal_y = 40.0

        while not rospy.is_shutdown():
            pubTwist = Twist()
            pubTwist.linear.x = goal_x - self.local_position.x
            pubTwist.linear.y = goal_y - self.local_position.y
            self.pub.publish(pubTwist)
            rate.sleep()

if __name__ == '__main__':
    try:
        myPublish = publisher()
        myPublish.init()
        myPublish.run()
    except rospy.ROSInterruptException:
        pass
