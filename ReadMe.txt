"""KalmanFilter.ipynb


Original file is located at
    https://colab.research.google.com/drive/1V7u2cbebeHw5ARE0haBm7M4z7FjvpY93

# Kalman Filter

A Discrete Kalman Filter Class,representing each person in a camera frame.

Estimates the state vector X of a discrete-time
controlled process that is governed by the
linear stochastic difference equation.

---


x = A x_(t-1) + B_t(Matrix that maps actions to states) U_t (Actions Taken) + e_t (Error only uncertainity) (R_t)


---


C map state to observation/measurment.
z (measurement) = C_t x_t + alpha_t(Error) (Q_T)


---



E_t and alpha_t are assumed to be independent and normally distributed
with covariance Qt and Rt

For the Tracking Problem,
<br>
**Our state X is (nx1), where n=8 (dimensions) =[X,Y,A,H,Vx,Vy,Va,Vh]**
<br>
X (bounding box center position along the x axis)
<br>
y (bounding box center position along the y axis)
<br>
A (Aspect Ratio of the bounding box calculaed by Width/Height)
<br>
H (Height of the bounding box)
<br>
Vx, Vy, Va,Vh are the rate of change of the above variables (velocity) respectively.
<br>

we assumed constant velocity (linear observation model)
<br>
**Our observation vector is [X,Y,A,H]**

y**(t)= 0  (constant Velocity model) <br>
y*(t)=y*(to) - <del> g(t-to)</del> <br>
y(t)= y(to) + y*(to) (t-to) - <del>g/2 (t-to)^2 </del> <br>
