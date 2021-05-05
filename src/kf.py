import numpy as np
## Domain Specific Kalman Filter Class
"""
A Discrete Kalman Filter Class,representing each person in a camera frame.

Estimates the state vector X of a discrete-time
controlled process that is governed by the
linear stochastic difference equation.
X=  [X, Y, A, H, Vx, Vy, Va, Vh] Time increment of 1
   A is NxN from that Maps from previous state to next state
   A=  [1, 0, 0, 0, 1,  0,  0,  0,  //For X
        0, 1, 0, 0, 0,  1,  0,  0,  //for Y
        0, 0, 1, 0, 0,  0,  1,  0,  //for A
        0, 0, 0, 1, 0,  0,  0,  1,  //for H
        0, 0, 0, 0, 1,  0,  0,  0,  //for Vx
        0, 0, 0, 0, 0,  1,  0,  0,  //for Vy
        0, 0, 0, 0, 0,  0,  1,  0,  //for Va
        0, 0, 0, 0, 0,  0,  0,  1]  //for Vh
    B=0
    Z=[X, Y, A, H]
    C that maps state to observation
    4XN
    C= [1, 0, 0, 0, 0,  0,  0,  0,  //For X
        0, 1, 0, 0, 0,  0,  0,  0,  //for Y
        0, 0, 1, 0, 0,  0,  0,  0,  //for A
        0, 0, 0, 1, 0,  0,  0,  0]  //for H
    R Observation Noise
    Q Measurment  Noise
"""
class KalmanFilter():

    def __init__(self):
        self.A = np.array([[1, 0, 0, 0, 1, 0, 0, 0],
                           [0, 1, 0, 0, 0, 1, 0, 0],
                           [0, 0, 1, 0, 0, 0, 1, 0],
                           [0, 0, 0, 1, 0, 0, 0, 1],
                           [0, 0, 0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1]])

        self.C = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0, 0, 0]])

        self.stdP = 0.05
        self.stdV = 1 / 160

    def calculateMeanAndCovInitial(self, measurement):
        mu = np.concatenate((measurement, np.zeros_like(measurement)))
        cov = self.calculateCovariance([2 * self.stdP ,  # Initial covariance
                                        2 * self.stdP ,
                                        1e-2,
                                        2 * self.stdP ,
                                        10 * self.stdV ,
                                        10 * self.stdV ,
                                        1e-5,
                                        10 * self.stdV])
        return mu, cov

    def calculateCovariance(self, standardDev):
        return np.diag(np.square(standardDev)).astype(np.float32)

    def predict(self, oldMu, oldCov):
        # Applying motion model to get observations
        newMu = np.dot(self.A, oldMu)  # +np.dot(self.B,actiontaken)
        stdArr = [self.stdP * oldMu[3],
                  self.stdP * oldMu[3],  # As a function of previous height
                  1e-2,
                  self.stdP * oldMu[3],
                  self.stdV * oldMu[3],
                  self.stdV * oldMu[3],
                  1e-5,
                  self.stdV * oldMu[3]]
        R = self.calculateCovariance(stdArr)
        newCov = np.dot(np.dot(self.A, oldCov), self.A.T) + R
        return newMu, newCov  # Predicted Mu

    def correct(self, muP, covP, measurement):
        qStd = [self.stdP ,
                self.stdP ,
                1e-1,
                self.stdP ]
        Q = self.calculateCovariance(qStd)
        kalmanGain = np.dot(np.dot(covP, self.C.T), np.linalg.inv(np.dot(np.dot(self.C, covP), self.C.T) + Q))
        correctedMu = muP + np.dot(kalmanGain, measurement.reshape(4,1) - np.dot(self.C, muP))
        correctedCov = covP - np.dot(np.dot(kalmanGain, self.C), covP)
        return correctedMu, correctedCov