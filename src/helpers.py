
import numpy as np

"""##From BBOX Mean to Kalman Measures (Mean) (Vectorized)"""
def bboxToKalmanMeasures(bbox):

    # box1 -- object with coordinates (box1_x1, box1_y1, box1_x2, box_1_y2)
    (box1_x1, box1_y1, box1_x2, box1_y2) = bbox
    width = box1_x2 - box1_x1
    height = box1_y2 - box1_y1
    centerX = box1_x1 + width / 2
    centerY = box1_y1 + height / 2
    aspectRatio = width / height
    scale = width * height
    return np.array([centerX, centerY, aspectRatio, height]).reshape(4, 1)

"""##From Kalman Mean to BBOX (Vectorized)"""
def KalmanMeasuresTobbox(measures):
    #width = aspectRatio * height
    height=measures[...,3]
    width=measures[...,2]*height
    box1_x1=measures[...,0]-width/2
    box1_y1=measures[...,1]-height/2
    box1_x2=box1_x1+width
    box1_y2=box1_y1+height

    return np.stack((box1_x1,box1_y1,box1_x2,box1_y2),axis=1)


"""##IOU Multiple Boxes Vectorized """
def iou_vectorized(boxes1, boxes2):
    """Implement the intersection over union (IoU) between all boxes1 and all boxes2
    
    Arguments:
    boxes1 -- a numpy array [[xi1,yi1,xi2,yi2],...],
    boxes2 -- a numpy array [[xi1,yi1,xi2,yi2],...],
    """
    boxes1 = np.expand_dims(boxes1, 1)
    boxes2 = np.expand_dims(boxes2, 0)
    xi1 = np.maximum(boxes1[..., 0], boxes2[..., 0])
    yi1 = np.maximum(boxes1[..., 1], boxes2[..., 1])
    xi2 = np.minimum(boxes1[..., 2], boxes2[..., 2])
    yi2 = np.minimum(boxes1[..., 3], boxes2[..., 3])
    inter_width = np.maximum(xi2 - xi1, 0)
    inter_height = np.maximum(yi2 - yi1, 0)
    inter_area = inter_width * inter_height
    # Calculate the Union area by using Formula: Union(A,B) = A + B - Inter(A,B)
    box1_areas = (boxes1[..., 2] - boxes1[..., 0]) * (boxes1[..., 3] - boxes1[..., 1])
    box2_areas = (boxes2[..., 2] - boxes2[..., 0]) * (boxes2[..., 3] - boxes2[..., 1])
    union_area = box1_areas + box2_areas - inter_area

    # compute the IoU
    iou = inter_area / union_area
    return iou

"""#Intersection Over Union From Scratch

##IOU between two boxes
"""
def iou(box1, box2):
    """Implement the intersection over union (IoU) between box1 and box2
    
    Arguments:
    box1 -- first box, list object with coordinates (box1_x1, box1_y1, box1_x2, box_1_y2)
    box2 -- second box, list object with coordinates (box2_x1, box2_y1, box2_x2, box2_y2)
    """

    # Assign variable names to coordinates for clarity
    (box1_x1, box1_y1, box1_x2, box1_y2) = box1
    (box2_x1, box2_y1, box2_x2, box2_y2) = box2

    # Calculate the (yi1, xi1, yi2, xi2) coordinates of the intersection of box1 and box2. Calculate its Area.
    ### START CODE HERE ### (≈ 7 lines)
    xi1 = max(box1_x1, box2_x1)
    yi1 = max(box1_y1, box2_y1)
    xi2 = min(box1_x2, box2_x2)
    yi2 = min(box1_y2, box2_y2)
    inter_width = max(xi2 - xi1, 0)
    inter_height = max(yi2 - yi1, 0)
    inter_area = inter_width * inter_height
    ### END CODE HERE ###    

    # Calculate the Union area by using Formula: Union(A,B) = A + B - Inter(A,B)
    box1_area = (box1_x2 - box1_x1) * (box1_y2 - box1_y1)
    box2_area = (box2_x2 - box2_x1) * (box2_y2 - box2_y1)
    union_area = box1_area + box2_area - inter_area

    # compute the IoU
    iou = inter_area / union_area

    return iou


def squarify(matrix, value):
    maxPad = max(matrix.shape[1], matrix.shape[0])
    return np.pad(matrix, ((0, maxPad - matrix.shape[0]), (0, maxPad - matrix.shape[1])), mode='constant',
                  constant_values=value)