from scipy.optimize import linear_sum_assignment
from .person import KalmanPerson
from .helpers import *
from .kf import KalmanFilter

"""## Main Tracking Class"""
class KalmanTracking():
    def __init__(self, IOUThreshold=0.5, removeTrackAfternFramesThres=20, uncertaintyCount=2):

        self.trackedPeople = []
        self.frameCount = 0
        self.IOUThreshold = IOUThreshold
        self.removeTrackAfternFramesThres = removeTrackAfternFramesThres
        self.uncertaintyCount = uncertaintyCount
        self.kalmanFilter = KalmanFilter()
        self.id = 0

    # Hussein understand this
    def matchDetectionsNaiveApproach(self, detections):
        # IOU Matrix #TODO check here
        if len(self.trackedPeople) == 0:
            return np.empty(shape=(0, 2), dtype=int), np.arange(len(self.trackedPeople)), np.arange(len(detections))
        self.predictAll()
        trackedCoordinates = [t.getMean() for t in self.trackedPeople]

        costMatrix = iou_vectorized(
            KalmanMeasuresTobbox(np.asarray(trackedCoordinates, dtype=np.float32).reshape(-1, 8)), detections)
        completeMatching = np.empty(shape=(0, 2), dtype=int)
        if min(costMatrix.shape) > 0:
            passedThreshols = (costMatrix > self.IOUThreshold).astype(np.int32)
            if np.sum(passedThreshols, axis=0).max() == 1 and np.sum(passedThreshols, axis=1).max() == 1:
                completeMatching = np.stack(np.where(passedThreshols), axis=1)
            else:
                x, y = linear_sum_assignment(-costMatrix)
                completeMatching = np.array(list(zip(x, y)))
        self.completeMatching = completeMatching
        # April 4 cont
        unmatchedDetByIOU = np.delete(np.arange(len(detections)), completeMatching[:, 1])
        unmatchedTracksByIOU = np.delete(np.arange(len(trackedCoordinates)), completeMatching[:, 0])

        finalMatchedList = np.empty((0, 2), dtype=int)

        for trackI, detI in completeMatching:
            if costMatrix[trackI][detI] >= self.IOUThreshold:
                # add to final list
                finalMatchedList = np.append(finalMatchedList, [[trackI, detI]], axis=0)
            else:
                # add to unmatched list
                unmatchedDetByIOU = np.append(unmatchedDetByIOU, [detI], axis=0)
                unmatchedTracksByIOU = np.append(unmatchedTracksByIOU, [trackI], axis=0)

        return finalMatchedList, unmatchedTracksByIOU, unmatchedDetByIOU

    def predictAll(self):
        for person in self.trackedPeople:
            person.predict(self.kalmanFilter)

    def correctAll(self, detections):
        allMatches, allUnmatched, unmatchedDet = self.matchDetectionsNaiveApproach(detections)
        for i, j in allMatches:
            self.trackedPeople[i].correct(self.kalmanFilter, bboxToKalmanMeasures(detections[j]))
        for i in allUnmatched:
            self.trackedPeople[i].markUnmatched()

        for i in unmatchedDet:
            newMean, newCovariance = self.kalmanFilter.calculateMeanAndCovInitial(bboxToKalmanMeasures(detections[i]))
            id = -1
            name = ""
            cameraId = -1
            self.trackedPeople.append(KalmanPerson(newMean,
                                                   newCovariance,
                                                   self.uncertaintyCount,
                                                   self.removeTrackAfternFramesThres,
                                                   name=name, id=self.id, cameraId=cameraId))

            self.id += 1

        self.trackedPeople = [person for person in self.trackedPeople if person.getState() != "D"]
    def match(self,detections,state):
        self.correctAll(detections)
        trackedCoordinates = [t.getMean()[:4] for t in self.trackedPeople if t.getState() == state]
        trackedIds = [t.id + 1 for t in self.trackedPeople if t.getState() == state]
        correctedDetNpArr = KalmanMeasuresTobbox(np.asarray(trackedCoordinates, dtype=np.float32).reshape(-1, 4))
        return np.concatenate((correctedDetNpArr, np.array(trackedIds).reshape(-1, 1)), axis=1) if len(
            trackedIds) != 0 else []
