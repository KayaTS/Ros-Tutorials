#/usr/bin/env python
import rospy
import math
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
  

def followrobot(robotpose): # takip edilecek robotun pose bilgisini alir
    
    vel_msg = Twist()

    while True: #surekli takip etmeli
        #takip yapilacak robotun bilgilerini surekli gunceller
        x_robot = robotpose.x
        y_robot = robotpose.y 
        dist = abs((math.sqrt((x_robot-pose.x)**2 + (y_robot-pose.y)**2)))  #mesafe
        if ( dist < maxFolDist): # belirlenen bir mesafeye kadar takip eder, bu mesafeye geldiyse diger robotun ilerlemesini bekler
            break
        else:

            Kv = 0.5 # cizgisel hiz K gaini
            if Kv * dist > maxLinVel: # maximum hizdan fazla olursa, hizimiz maksimum hiz kadar olur
                v = maxLinVel
            else:
                v = Kv * dist  # maximum cisgisel hiz maximum degeri gecemez
            

            Kw = 2 # acisal hiz K
            tethaStar = math.atan2(y_robot-pose.y, x_robot-pose.x)  # desired heading

            if (tethaStar-pose.theta) * Kw > maxAngVel: # maximum hizdan fazla olursa, hizimiz maksimum hiz kadar olur
                w = maxAngVel
            else:
                w = (tethaStar-pose.theta) * Kw # acisal hiz maximum degeri gecemez

            # bu bilgileri  guncelleyip publish ederiz
            vel_msg.linear.x = v
            vel_msg.angular.z = w 

            vel_pub.publish(vel_msg) 

        

if __name__ == '__main__':
    rospy.init_node('controller', anonymous = True)
    # kendi robotumuzun subs pub islemleri
    rospy.Subscriber('/turtle2/pose', Pose, callbackPose)
    vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=5)
    # takip edecegimiz robota subscribe olup surekli olarak onu takip edecek 'followrobot' fonksiyonunu cagiririz
    rospy.Subscriber('/turtle1/pose', Pose, followrobot)
    loop_rate = rospy.Rate(5)
    
    rospy.spin()