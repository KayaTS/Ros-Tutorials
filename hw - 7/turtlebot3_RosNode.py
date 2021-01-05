#/usr/bin/env python
import rospy
import math
import numpy as np

# 1- Prediction #
# initial condition

x_pos = 0
y_pos = 0
x_vel = 0
y_vel = 0
deltaT = 0.5

var_px = 0.1
var_py = 0.1 
var_vx = 0.1
var_vy = 0.1

def kalmanFilterPrediction(acceleration):
    #Predict state hesaplamasi
    #               xhead(k+1) = F(k) * X(k) + G(k) * U(k)  
    # formulumuz;   x(yeni)    = x0 + t.v + 1/2 . a . t^2
    xhead = [x_newP, 
             y_newP,
             x_newV,
             y_newV]
    
    Fk = [[1, 0, deltaT, 0],
         [0, 1, 0, deltaT],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]
    
    Xk = [x_pos,
         y_pos,
         x_vel,
         y_vel]

    Bk = [[0.5 * deltaT*deltaT, 0],
          [0, 0.5 * deltaT*deltaT],
          [deltaT, 0],
          [0, deltaT]]

    x_acc = acceleration.x #imudan gelen x ivme degeri
    y_acc = accekeration.y  #imuden gelen y ivme degeri

    u = [x_acc, y_acc]

    toplam1 = np.dot(Fk, Xk) # [Fk][Xk] islemi
    toplam2 = np.dot(Bk, u) # [Bk][u] islemi

    xhead = np.add(toplam1, toplam2)
    # bu islem bitince yeni hesaplanan degerler bir sonraki hesaplamaya aktarilir
    x_pos = x_newP 
    y_pos = y_newP
    x_vel = x_newV
    y_vel = y_newV
    #p matrixi
    P = [[var_px, 0, 0, 0],
    [0, var_py, 0, 0],
    [0, 0, var_vx, 0],
    [0, 0, 0, var_vy]]


    #P = FK.P.Ft
    carpim = np.dot(Fk, P)
    F_Transpoz = Fk.matrix.transpoze()
    P = np.dot(carpim, F_Transpoz) 

    return P, xhead

# 2- Optimal Gain #

def kalmanFilterGain(P_koveryans):
    
    H = [[0, 0, 1, 0],
        [0, 0, 0, 1]]

    H_transpoz = H.matrix.transpoze()

    gain = np.dot(H, P_koveryans, H_transpoz)
    # Kg = P.Ht.(H.P.Ht)
    K_gain = np.dot(P_koveryans, H_transpoz, np.linalg.inv(gain))
    
    return K_gain

# 3- Correction #
def kalmanFilterCorrection(gps, P, xhead, K_gain):
    
    Y_meas = [gps.position.x, gps.position.y]
    temp = np.dot(H, xhead)
    Y = np.subtract(Y_meas, temp)

    xhead = np.add(xhead, np.dot(K_gain, Y))

    P = np.dot(np.subtract(np.identity(4), np.dot(K_gain, H)), P)

    return P, xhead

# Iteratif fonksiyon #
def Kalman_positioning(acceleration, position):
    (P, xhead) = kalmanFilterPrediction(acceleration) # 1
    Kgain = kalmanFilterGain(P) # 2
    kalmanFilterCorrection(position, P, xhead, Kgain) # 3 




if __name__ == '__main__':
    rospy.init_node('positioning', anonymous = True)
    rospy.Subscriber('/gps/fix', Pose, Kalman_positioning)
    rospy.Subscriber('/imu', gaussian, Kalman_positioning)
    vel_pub = rospy.Publisher('/turtlebot3/', Twist, queue_size=5)
    loop_rate = rospy.Rate(5)
    rospy.spin()