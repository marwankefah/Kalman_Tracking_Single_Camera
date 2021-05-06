
"""## Each Tracked Kalman Person Class"""

class KalmanPerson():

  def __init__(self, mean, covariance, uncertaintyCount, maxMisses, name='', id=-1, cameraId=-1):
    # box1 -- object with coordinates (box1_x1, box1_y1, box1_x2, box_1_y2)
    self.name = name
    self.id = id
    self.cameraId = cameraId
    self.mean = mean
    self.covariance = covariance
    self.uncertaintyCount = uncertaintyCount
    self.maxMisses = maxMisses
    self.age = 1
    self.hits = 1
    self.consecutivePredictionCount = 0
    self.state = "U"  # Uncertain about it "C"==confirmed "D"== deleted

  def predict(self, kalmanFilter):
    self.mean, self.covariance = kalmanFilter.predict(self.mean, self.covariance)
    self.age += 1
    self.consecutivePredictionCount += 1

  def getMean(self):
    return self.mean.copy()

  def correct(self, kalmanFilter, measurement):
    self.mean, self.covariance = kalmanFilter.correct(self.mean, self.covariance, measurement)
    self.consecutivePredictionCount = 0
    self.hits += 1
    ##i am uncertain about it untill i hit the target more than once
    if self.state == "U" and self.hits >= self.uncertaintyCount:
      self.state = "C"
    if self.state =="M":
        self.state = "C"

  # at some frames some tracks may not be detected or matched *Occuleded*
  def markUnmatched(self):
    if self.state == "U":
      self.state = "D"
    elif self.consecutivePredictionCount > self.maxMisses:
      self.state = "D"
    elif self.state=="C":
      self.state="M"

  def getState(self):
    return self.state
