#!/usr/bin/python2


from std_msgs.msg import Float32, Bool, Int32MultiArray
from sensor_msgs.msg import CompressedImage
import time
import rospy
import math
from header import *

import cv2
import numpy as np
from keras.models import Model, load_model
from keras.layers import Input
from keras.layers.core import Lambda, RepeatVector, Reshape
from keras.layers.convolutional import Conv2D, Conv2DTranspose
from keras.layers.pooling import MaxPooling2D
from keras.layers.merge import concatenate
from keras.callbacks import EarlyStopping, ModelCheckpoint, ReduceLROnPlateau
from keras import backend as K
class RoadML:
    def __init__(self):
        self.image_np = None
        self.flag = False
        self.speed = 60
        self.preSteer = 0
        self.center = [0,0]
        self.obs = (0,0,0,0)
        self.sign = []
        self. decision = 0
        self.flag2 = False
        self.temp = False
        self.index = 0

        self.height, self.width = 240, 320
        input_img = Input((self.height, self.width, 3), name='img')

        c1 = Conv2D(8, (3, 3), activation='relu', padding='same') (input_img)
        c1 = Conv2D(8, (3, 3), activation='relu', padding='same') (c1)
        p1 = MaxPooling2D((2, 2)) (c1)

        c2 = Conv2D(16, (3, 3), activation='relu', padding='same') (p1)
        c2 = Conv2D(16, (3, 3), activation='relu', padding='same') (c2)
        p2 = MaxPooling2D((2, 2)) (c2)

        c3 = Conv2D(32, (3, 3), activation='relu', padding='same') (p2)
        c3 = Conv2D(32, (3, 3), activation='relu', padding='same') (c3)
        p3 = MaxPooling2D((2, 2)) (c3)

        c4 = Conv2D(64, (3, 3), activation='relu', padding='same') (p3)
        c4 = Conv2D(64, (3, 3), activation='relu', padding='same') (c4)

        u5 = Conv2DTranspose(64, (2, 2), strides=(2, 2), padding='same') (c4)
        u5 = concatenate([u5, c3])
        c6 = Conv2D(32, (3, 3), activation='relu', padding='same') (u5)
        c6 = Conv2D(32, (3, 3), activation='relu', padding='same') (c6)

        u7 = Conv2DTranspose(32, (2, 2), strides=(2, 2), padding='same') (c6)
        u7 = concatenate([u7, c2])
        c7 = Conv2D(16, (3, 3), activation='relu', padding='same') (u7)
        c7 = Conv2D(16, (3, 3), activation='relu', padding='same') (c7)

        u8 = Conv2DTranspose(16, (2, 2), strides=(2, 2), padding='same') (c7)
        u8 = concatenate([u8, c1])
        c8 = Conv2D(8, (3, 3), activation='relu', padding='same') (u8)
        c8 = Conv2D(8, (3, 3), activation='relu', padding='same') (c8)

        outputs = Conv2D(1, (1, 1), activation='sigmoid') (c8)

        self.model = Model(inputs=[input_img], outputs=[outputs])
        self.model.load_weights('/home/transon/catkin_ws/src/roadml/model/final-road-seg-model-v2-map2.h5')

        self.subscriber = rospy.Subscriber(rgbNode, CompressedImage, self.RgbCallback, queue_size=1)
        self.speed_pub = rospy.Publisher(speedNode, Float32, queue_size=1)
        self.steerAngle_pub = rospy.Publisher(steerNode, Float32, queue_size=1)
        self.signSub = rospy.Subscriber(signNode, Float32, self.SignCallback, queue_size=1)
        self.obsSub = rospy.Subscriber(obsNode, Int32MultiArray, self.ObsCallback, queue_size=1)
        print ("OK")

    def RgbCallback(self, rgb_data):
        if(rgb_data.data != None):
            np_arr = np.fromstring(rgb_data.data, np.uint8)
            self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.flag = True
            time.sleep(0.01)


    def SignCallback(self, sign_data):
        if(sign_data.data != None):
            #self.sign = sign_data.data
            #print ("sign: " + str(self.sign) + "\tobs: " + str(self.obs))
            if(sign_data.data != 0):
                self.sign.append(1)
                #print ("data: " + str(sign_data.data))
                self.decision = sign_data.data
            else:
                self.sign.append(0)
            self.flag2 = False
            _len = len(self.sign)
            if(_len > 2):
                if(self.sign[_len - 1] == 0 and self.sign[_len - 2] == 1):
                    self.flag2 = True
                else:
                    self.flag2 = False
            if(self.flag2):
                self.sign = []
            time.sleep(0.01)

    def ObsCallback(self, obs_data):
        if(obs_data.data != None):
            self.obs = obs_data.data
            time.sleep(0.01)

    def Roi(self, src):
        mask = np.zeros_like(src)
        pts = np.array([[(0, self.height), (80, 80), (self.width - 80, 80), (self.width, self.height)]])
        cv2.fillPoly(mask, pts, 255)
        segment = cv2.bitwise_and(src, mask)
        return segment

    def GetPoint(self, src):
        midX, midY, count, limit = 0,0,0,80
        for i in range(limit,240):
            count += 1
            left = 0
            while(src[i][left] != 255 and left < 160):
                left += 1
            right = 320 - 1
            while(src[i][right] != 255 and right >= 160):
                right -= 1
            midX += (int) ((left + right) / 2)
            midY += i
        return ((int) (midX / count), (int) (midY / count))

    def GetSteer(self, p):
        dx = p[0] - 160 + 1
        dy = 240.0 - p[1]
        return math.atan(dx/dy) * 57.32
            
    def RectBlack(self, img, rect):
        start_point = (rect[0], rect[1])
        end_point = (rect[0] + rect[2], rect[1] + rect[3])
        cv2.rectangle(img, start_point, end_point, (0, 0, 0), -1)
        return img

    def DynamicSpeed(self, steer, speed):
        return speed * math.cos(abs(steer) * 0.0174)
    def publish(self):
        while(True):
            if(self.flag == True):
                my_preds = self.model.predict(np.expand_dims(self.image_np, 0))
                my_preds = np.where(my_preds > 0.5, 1, 0)
                img = my_preds.reshape(self.height, self.width)
                #cv2.imshow('seg', img*255.*1.)
                img = img*255.*1.
                img = self.Roi(img)
                img = self.RectBlack(img, self.obs)
                self.center = self.GetPoint(img)
                cv2.line(img, (160, 239), (self.center[0], self.center[1]), (0, 0, 0), 2)
                cv2.imshow("bin", img)

                steer = self.GetSteer(self.center)*0.68 - self.preSteer*0.32
                self.preSteer = steer
                speed = self.DynamicSpeed(steer, self.speed)

                if(self.flag2):
                    self.temp = True
                
                if(self.temp == True and self.index < 17):
                    self.index += 1
                    if(self.decision == 1):
                        #print ("sign: " + str(self.decision))
                        steer = -20.0
                        speed = 40
                    if(self.decision == 2):
                        #print ("sign: " + str(self.decision))
                        steer = 20.0
                        speed = 40
                    print ("decision: " + str(self.decision) + "\tindex: " + str(self.index))
                    if(self.index >= 17):
                        self.temp = False
                        self.index = 0
                    self.speed_pub.publish(speed)
                    self.steerAngle_pub.publish(steer)
                    time.sleep(0.1)

                self.speed_pub.publish(speed)
                self.steerAngle_pub.publish(steer)
                cv2.waitKey(1)
                time.sleep(0.001)

if __name__ == '__main__':
    rospy.init_node('RoadML', anonymous=True)
    road = RoadML()
    road.publish()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()
    
