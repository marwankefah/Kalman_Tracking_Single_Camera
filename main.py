# -*- coding: utf-8 -*-


from src.tracking import KalmanTracking
tracker = KalmanTracking(IOUThreshold=0.3, removeTrackAfternFramesThres=20, uncertaintyCount=2)

##LOOP OVER FRAMES

    #Get Detections From Deetection ModEL

    #tracker.correctAll(detections)

    ## Get confirmed Tracks and their relative Ids

    #trackedCoordinates = [t.getMean()[:4] for t in tracker.trackedPeople if t.getState() == "C"]
    #trackedIds = [t.id for t in tracker.trackedPeople if t.getState() == "C"]




