This is our source code for Digital Race 2020 Competition

Complete packages:
+ [CDSNTU](cdsntu)
+ [CDSNTU1](cdsntu1)
+ [CDSNTU3](cdsntu3)

Description:
+ [CDSNTU](cdsntu): Remove tree.
+ [CDSNTU1](cdsntu1): Not remove tree.
  - DetectLane: detect road by HSV channel; using erosion, dilation to denoise pepper and salt noises; get ROI(Region of interest) by 4 points and 6 points; detect snow by HSV channel and hide 2 sides; remove noises by getting maximum area and delete other area. Finally hide sky area.  
  - DetectObstacle: read mask.png and convert it to gray, then convert depth image to gray, calculating different between 2 images and threshold it. Get ROI of image that we calculated, finding all contours and getting maximum area to detect obstacle. Publish boolean data to "/obstacle" topic ( has obsatcle or not).  
  - DetectSign: detect blue color by HSV channel, find all contours and remove noise by calculating ratio, area of rects.Then crop it and calculating HOG vector.After that, put it into SVM model to predict what kind of sign (0: None, 1: Left sign, 2: Right sign, 3: Stop sign). Publish float data to "/sign".  
  - ControlCar: Nothing do say :)  
+ [CDSNTU2](cdsntu2): (update soon)
+ [CDSNTU3](cdsntu3): Complete 4 maps. (something bug with pid, detect sign wrong)
+ [roadml](roadml): Using segmentation with python.


