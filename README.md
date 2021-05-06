
# Kalman Filter Tracker

Unofficial implementation of [SORT](http://arxiv.org/abs/1602.00763), A simple online and realtime tracking algorithm for 2D multiple object tracking in video sequences.

SORT is based on the idea of a discrete Kalman Filter,representing each object to be tracked.

Kalman Filter estimates the state vector X of a discrete-time controlled process that is governed by the linear stochastic difference equation.

---

#### X = A X<sub>t-1</sub> + B<sub>t</sub> U<sub>t</sub> +  R<sub>t</sub>
#### Z = C<sub>t</sub> X<sub>t</sub> + Q<sub>t</sub>
   -    X  (state of persons in the system)
   -    z  (measurements taken from Object Detection Model)
   -    A  (Matrix that maps previous state to current State)
   -    B  (Matrix that maps actions to states) 
   -    U  (Actions Taken)
   -    C  (Matrix that maps mesurement to state)
   -    R  (Process/Observational Noise, noise comming from the motion model)
   -    Q  (Measurement Noise, noise comming from the object detection model)

 Q and R covariance matrices are assumed to be independent and normally distributed.

y(t)=y(to) - <del>ahmed </del> 

# Tracking with constant velocity (linear observation model)
#### y<sup>''</sup> (t)= 0 (acceleration a)  (constant Velocity model) 
#### y<sup>'</sup>(t)=y<sup>'</sup>(to) - <del>a(t-to)</del> 
#### y(t)= y(to) + y<sup>'</sup>(to) (t-to) - <del>a/2 (t-to)^2 </del> 
#### State X is (nx1), where n=8 (dimensions) =[X,Y,A,H,Vx,Vy,Va,Vh]
####  Measurement  Z vector is [X,Y,A,H] (Object Detection Model)
   -    X (bounding box center position along the x axis)
   -    Y (bounding box center position along the y axis)
   -    A (Aspect Ratio of the bounding box calculaed by Width/Height)
   -    H (Height of the bounding box)
   -    V<sub>x</sub>, V<sub>y</sub>, V<sub>a</sub>,V<sub>h</sub> are the rate of change of the above variables (velocity) respectively.

 



