# -*- coding: utf-8 -*-


from src.tracking import KalmanTracking
tracker = KalmanTracking(IOUThreshold=0.3, removeTrackAfternFramesThres=20, uncertaintyCount=2)
##LOOP OVER FRAMES

    #detections= Get detections From Deetection ModEL

    #trackers=tracker.match(detections,state="C")




