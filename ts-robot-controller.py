#/usr/bin/env python
import rospy
import math
import random
import json
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

#json dosyasi okunur
f = open('parameters.json',) 
parameters = json.load(f)

#json dosyasindaki parametreler alinir
maxLinVel = parameters["lineerMaxHiz"]
maxAngVel = parameters["angularMaxHiz"]
maxFolDist = parameters["followingDist"]

pose=Pose()
poseflag=False

def callbackPose(msg):
    global pose
    global poseflag
    pose=msg
    poseflag=True
    print("Pose: ", msg.x, msg.y, msg.theta)

def robot(): #robotu surekli hareket ettirir
    dist_random = random.uniform(1, 5) # Robotu rastgele bir mesafeye gonderir
    #if (pose.x > 0.5 or pose.x < 10.5) and (pose.y > 0.5 or pose.y< 10.5) : # duvara carpmadiysa
    move(2, dist_random)
    rotatate(0.1, math.pi/6)
    #elif (pose.x < 10.5):
     #   rotatate(3, math.pi - pose.theta)

def move(speed, distance):
    while poseflag ==False:
        rospy.sleep(0.01)
    
    x0 = pose.x
    y0 = pose.y
    print("Initial position: ", x0, y0)

    vel_msg = Twist()
    while True:
        dist = math.sqrt((pose.x-x0)**2 + (pose.y-y0)**2) # katedilen mesafe
        print("travelled distance: ", dist)
        if (pose.x > 0.5 and pose.x < 10.5) and (pose.y > 0.5 and pose.y< 10.5) : # duvara carpmadiysa normal hareketine devam eder
            if dist < 0.99 * distance:
                    if speed < maxLinVel: # maksimum hizi gecemez
                        vel_msg.linear.x = speed
                    else:
                        vel_msg.linear.x = maxLinVel    
                    vel_pub.publish(vel_msg) #publish edilen mesaj
            else: 
                vel_msg.linear.x = 0
                vel_pub.publish(vel_msg) #publish edilen mesaj
                break
        # duvara carptiysa donerek tekrar yoluna devam eder
        elif pose.x > 10.5: # sag duvara carpti ise
            rotatate(0.1, math.pi)
            break
        elif pose.x < 0.5: # sol duvara carpti ise
            rotatate(0.1, math.pi - pose.theta)
            break

        loop_rate.sleep()

def rotatate(speed, angle):
    while poseflag ==False:
        rospy.sleep(0.01)
    
    vel_msg = Twist()
    while True:
        diff = abs(pose.theta-angle) # mutlak deger ile yoneldigimiz aci ile hedef arasindaki fark
        print("Angle Diff: ", diff)
        if diff > 0.05:
                if speed < maxAngVel: # maksimum hizi gecemez
                    vel_msg.angular.z = speed
                else:
                    vel_msg.linear.x = maxAngVel   
                vel_pub.publish(vel_msg)
        else:
                vel_msg.angular.z = 0
                vel_pub.publish(vel_msg)
                if pose.x > 10.5: # eger duvara carptiysa dondukten sonra biraz ileri giderek duvardan kurtul
                    move(2, 1)
                break
        
        loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('controller1', anonymous = True)
    rospy.Subscriber('/turtle1/pose', Pose, callbackPose) #Kendi robotumuzun pub sub islemleri
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)
    loop_rate = rospy.Rate(5)
    while True: # surekli olarak robotu hareket ettir
        robot()
    rospy.spin()

    '''   
        while True :
            move(0.5, 4)
            rotatate(0.1, math.pi/2)
        rospy.spin()

        rospy.Subscriber('/a/pose', Pose, callbackPose)
        vel_pub = rospy.Publisher('/a/cmd_vel', Twist, queue_size=5)
        loop_rate = rospy.Rate(5)

        move(0.5, 4)
        rotatate(0.1, math.pi/2)
    '''
    