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

def robot(): #robotu surekli hareket ettiren fonksiyon
    dist_random = random.uniform(1, 3) # Robotu rastgele bir mesafeye gonderir
    angle_random = random.uniform(1, 18) # Robotu rastgele bir aci gonderir
    if (pose.x > 0.5 or pose.x < 10.5) and (pose.y > 0.5 or pose.y< 10.5) : # duvara carpmadiysa surekli rastgele hareket eder ve doner
        move(2, dist_random)
        rotatate(0.1, (2*math.pi)/angle_random)

#eger duvara carpti  ise duvardan isik gibi yansimasi icin
#duvardan normale yaptigi aci kadar yansimasi icin (pi-tetha) kadar donmelidir
#gerektigi kadar dondukten sonra biraz cizgisel yol katederek duvardan kurtulur

    if pose.x > 10.5: # sag duvara carpti ise
        rotatate(0.1, math.pi - pose.theta) 
        turnBackWall(1, 2) 

    elif pose.x < 0.5: # sol duvara carpti ise
        rotatate(0.1, math.pi - pose.theta)
        turnBackWall(1, 2)    

    elif pose.y < 0.5: # alt duvara carpti ise
        rotatate(0.1, math.pi - pose.theta)
        turnBackWall(1, 2)

    elif pose.y > 10.5: # ust duvara carpti ise
        rotatate(0.1, math.pi - pose.theta)
        turnBackWall(1, 2)        


# duvara carpdiysa kacarak hareketine devam eder
def turnBackWall(speed, distance):
    print("duvardan kurtuluyoruz:")
    x0 = pose.x
    y0 = pose.y
    vel_msg = Twist()
    while True:
        dist = math.sqrt((pose.x-x0)**2 + (pose.y-y0)**2)   #katedilen mesafe
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
#duvara carpmadiysa
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
                vel_pub.publish(vel_msg)
            else: 
                vel_msg.linear.x = 0
                vel_pub.publish(vel_msg)
                
                break
        else:   #duvara carptiysa hareket fonktan cikar
            break
        loop_rate.sleep()

def rotatate(speed, angle):
    while poseflag ==False:
        rospy.sleep(0.01)
    
    vel_msg = Twist()
    while True:
        diff = abs(pose.theta-angle) # mutlak deger ile yoneldigimiz aci ile hedef arasindaki fark
        print("Angle Diff: ", diff)
        if diff > 0.1:
                if speed < maxAngVel: # maksimum hizi gecemez
                    vel_msg.angular.z = speed
                else:
                    vel_msg.angular.z = maxAngVel   
                vel_pub.publish(vel_msg)
        else:
                vel_msg.angular.z = 0
                
                vel_pub.publish(vel_msg)
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
    