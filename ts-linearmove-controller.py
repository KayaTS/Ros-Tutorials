#/usr/bin/env python
import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

pose=Pose()
poseflag=False

def callbackPose(msg):
    global pose
    global poseflag
    pose=msg
    poseflag=True
    print("Pose: ", msg.x, msg.y, msg.theta)

def move(speed, distance):
    while poseflag ==False:
        rospy.sleep(0.01)
    
    x0 = pose.x
    y0 = pose.y
    print("Initial position: ", x0, y0)

    vel_msg = Twist()
    while True:
        dist = math.sqrt((pose.x-x0)**2 + (pose.y-y0)**2)
        print("travelled distance: ", dist)
        if dist < 0.99 * distance:
                vel_msg.linear.x = speed
                vel_pub.publish(vel_msg) #publish edilen mesaj
        else:
            vel_msg.linear.x = 0
            vel_pub.publish(vel_msg) #publish edilen mesaj
            break
        loop_rate.sleep()

def rotatate(speed, angle):
    while poseflag ==False:
        rospy.sleep(0.01)
    
    vel_msg = Twist()
    while True:
        diff = abs(pose.theta-angle) # mutlak deger 'le yoneldigimiz aci ile hedef arasindaki fark
        print("Angle Diff: ", diff)
        if diff > 0.01:
                vel_msg.angular.z = speed
                vel_pub.publish(vel_msg)
        else:
                vel_msg.angular.z = 0
                vel_pub.publish(vel_msg)
                break
        loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('controller2', anonymous = True)
   
    rospy.Subscriber('/turtle2/pose', Pose, callbackPose)
    vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=5)
    loop_rate = rospy.Rate(5)

    move(0.5, 4)
    rotatate(0.1, math.pi/2)

    rospy.spin()