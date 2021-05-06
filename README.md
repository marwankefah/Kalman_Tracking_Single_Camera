
# Kalman Filter Tracker

Unofficial implementation of [SORT](http://arxiv.org/abs/1602.00763), A simple online and realtime tracking algorithm for 2D multiple object tracking in video sequences.

SORT is based on the idea of a discrete Kalman Filter,representing each object to be tracked.

Kalman Filter estimates the state vector X of a discrete-time controlled process that is governed by the linear stochastic difference equation.

---
## Using Kalman Tracker (SORT) with YOLOV4
![result1](https://github.com/marwankefah/emotionRecognition/blob/master/result1.jpg)

## Google Colab Example (Oxford TownCentre)
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/16g9ovKNglXNvJXIW8E4Hqg0vjHRg36Q-?usp=sharing)

# Prerequisites
   -  Numpy
   -  scipy.optimize.linear_sum_assignment


-  removeTrackAfternFramesThres  - Number of frames to try to match a missed object before you delete it
-  uncertaintyCount              - Number of frames to consider an object as a confirmed tracker (an object can be  Confirmed ("C"), UnCertain ("U), Missed ("M"))         
```
$ git clone https://github.com/marwankefah/Kalman-Tracking-Single-Camera
```
```
$ from KalmanTrackingSingleCamera.src import tracking
$ from KalmanTrackingSingleCamera.src import helpers as hp 
$ tracker=tracking.KalmanTracking(IOUThreshold=0.3 ,removeTrackAfternFramesThres=40,uncertaintyCount=1)
 
LOOP 
$$ GET DETECTION FROM YOUR DETECTION MODEL [[xminNew, yminNew, xmaxNew, ymaxNew],[xminNew, yminNew, xmaxNew, ymaxNew],....]
$$ tracker.correctAll(detections)
$$ trackedCoordinates = [t.getMean()[:4] for t in tracker.trackedPeople if t.getState()=="C"]
$$ trackedIds=[t.id for t in tracker.trackedPeople if t.getState()=="C"]
$$ correctedDetNpArr = hp.KalmanMeasuresTobbox(np.asarray(trackedCoordinates, dtype=np.float32).reshape(-1,4))
$$ correctedDetNpArr now have the coordinates for all the Confirmed ("C") tracks in your system.
$$ correctedDetNpArr==[[xminNew, yminNew, xmaxNew, ymaxNew],...] 
END LOOP    
```

# Tracking with constant velocity (linear observation model)
### Equation of Motion 
-  #### y<sup>''</sup> (t)= 0 (acceleration a)  (constant Velocity model) 
-  #### y<sup>'</sup>(t)=y<sup>'</sup>(to) - <del>a(t-to)</del> 
-  #### y(t)= y(to) + y<sup>'</sup>(to) (t-to) - <del>a/2 (t-to)^2 </del> 
### State Representation
-  #### X = A X<sub>t-1</sub> + B<sub>t</sub> U<sub>t</sub> +  R<sub>t</sub>
-  #### Z = C<sub>t</sub> X<sub>t</sub> + Q<sub>t</sub>
   -    X  (state of persons in the system)
   -    z  (measurements taken from Object Detection Model)
   -    A  (Matrix that maps previous state to current State)
   -    B  (Matrix that maps actions to states) 
   -    U  (Actions Taken)
   -    C  (Matrix that maps mesurement to state)
   -    R  (Process/Observational Noise, noise comming from the motion model)
   -    Q  (Measurement Noise, noise comming from the object detection model)

 Q and R covariance matrices are assumed to be independent and normally distributed.
 Q is chosen to be small as it correlate with how well your Object Detection Model Performs.
 R was assumed and adopted from the official Repository of SORT. However, someone can try to estimate such matrix from this [paper](https://ieeexplore.ieee.org/document/6719478).
#### State X is (nx1), where n=8 (dimensions) =[X,Y,A,H,Vx,Vy,Va,Vh]
####  Measurement  Z vector is [X,Y,A,H] (Object Detection Model)
   -    X (bounding box center position along the x axis)
   -    Y (bounding box center position along the y axis)
   -    A (Aspect Ratio of the bounding box calculaed by Width/Height)
   -    H (Height of the bounding box)
   -    V<sub>x</sub>, V<sub>y</sub>, V<sub>a</sub>,V<sub>h</sub> are the rate of change of the above variables (velocity) respectively.

 



