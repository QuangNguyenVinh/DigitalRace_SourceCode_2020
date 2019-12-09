This is our source code for Digital Race 2020 Competition

Complete packages:
+ [CDSNTU1](cdsntu1)
+ ...

Description:
+ [CDSNTU](cdsntu): (update soon)
+ [CDSNTU1](cdsntu1): 
  - DetectLane: detect road by HSV channel; using erosion, dilation to denoise pepper and salt noises; get ROI(Region of interest) by 4 points and 6 points; detect snow by HSV channel and hide 2 sides; remove noises by getting maximum area and delete other area. Finally hide sky area.  
  - DetectObstacle: read mask.png and convert it to gray, then convert depth image to gray, calculating different between 2 images and threshold it. Get ROI of image that we calculated, finding all contours and getting maximum area to detect obstacle. Publish boolean data to "/obstacle" topic ( has obsatcle or not).  
  - DetectSign: detect blue color by HSV channel, find all contours and remove noise by calculating ratio, area of rects.Then crop it and calculating HOG vector.After that, put it into SVM model to predict what kind of sign (0: None, 1: Left sign, 2: Right sign, 3: Stop sign). Publish float data to "/sign".  
  - ControlCar: Nothing do say :)  
+ [CDSNTU2](cdsntu2): (update soon)
+ [CDSNTU2_bk1](cdsntu2_bk1): (update soon)
+ [CDSNTU2_final](cdsntu2_final): Complete 3 maps. (Warning: Not fix bugs yet(like using Rect(), using absolute path instead of relative path to read mask.png, etc...))
+ [cdsntu2_9_12_2019](cdsntu2_9_12_2019): Complete 4 maps. (something bug with pid, detect sign wrong)
+ [roadml](roadml): Using segmentation with python.


