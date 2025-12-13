#!/usr/bin/env python3
import cv2
import numpy as np

def find_circles(image, tuning_params):

    blur = 5 

    x_min = tuning_params["x_min"]
    x_max = tuning_params["x_max"] 
    y_min = tuning_params["y_min"]
    y_max = tuning_params["y_max"]

    search_windown = [x_min, y_min, x_max, y_max]

    working_image = cv2.blur(image,(blur,blur))

    if search_windown is None: search_windown = [0.0, 0.0, 1.0, 1.0]
    search_windown_px = convert_rec_prec_to_pixel(search_windown,image)

    working_image = cv2.cvtColor(working_image, cv2.COLOR_BGR2HSV)

    # HSV threshold if mix < pixel_value < max return 255(White) else 0(Black)
    thresh_min = (tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"])
    thresh_max = (tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"])
    working_image = cv2.inRange(working_image, thresh_min, thresh_max)

    #After HSV threshold, there are noise in image so dialate and erode will remove it
    working_image = cv2.dilate(working_image, None, iterations=2)
    working_image = cv2.erode(working_image, None, iterations=2)

    tuning_image = cv2.bitwise_and(image,image,mask=working_image)
    working_image = apply_search_window(working_image, search_windown)

    working_image = 255 - working_image

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0
    params.maxThreshold = 100
        
    # Filter by Area, 30<Area<20000 pixel
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 20000
        
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
        
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5
        
    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(parameters=params)

    keypoints = detector.detect(working_image)
    size_min_px = tuning_params['sz_min']*working_image.shape[1]/100.0
    size_max_px = tuning_params['sz_max']*working_image.shape[1]/100.0
    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]

    line_color = (0,0,255)

    out_image = cv2.drawKeypoints(image, keypoints, np.array([]), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    out_image = draw_window2(out_image, search_windown_px)

    tuning_image = cv2.drawKeypoints(tuning_image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    tuning_image = draw_window2(tuning_image, search_windown_px)


    keypoints_normalised = [normalize_keypoint(working_image, k) for k in keypoints]

    return keypoints_normalised, out_image, tuning_image

def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    """
    Docstring for apply_search_window
    
    :param image: Input image
    :param window_adim: x_min, y_mix, x_max, y_max of ROI windown
    """
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px =  int(cols*window_adim[0]/100)
    y_min_px =  int(rows*window_adim[1]/100)
    x_max_px =  int(cols*window_adim[2]/100)
    y_max_px =  int(rows*window_adim[3]/100)

    mask = np.zeros(image.shape, np.uint8)
    mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]

    return mask

#Draw search windown to visualize for people
def draw_window2(image, rectangle_px, color=(255,0,0), line=5):
    return cv2.rectangle(image, (rectangle_px[0], rectangle_px[1], rectangle_px[2], rectangle_px[3]), color=color, thickness=line)


def convert_rec_prec_to_pixel(rect_perc, image):
    """
    Docstring for convert_rec_prec_to_pixel
    
    :param rect_perc: x_min, y_mix, x_max, y_max
    x_min*height/100 -> x_0
    y_min*width/100 -> y_0
    Return: x_0, y_0, x_1, y_1 to draw rectangle
    """
    rows = image.shape[0]
    cols = image.shape[1]
    

    scale = [cols, rows, cols, rows]
    
    return [int(a*b/100) for a,b in zip(rect_perc,scale)]

def normalize_keypoint(cv_image, kp):
    rows = float(cv_image.shape[0])
    cols = float(cv_image.shape[1])

    center_x = 0.5*cols
    center_y = 0.5*rows
    x = (kp.pt[0] - center_x)/center_x
    y = (kp.pt[1] - center_y)/center_y

    return cv2.KeyPoint(x,y, kp.size/cv_image.shape[1])


def create_tuning_window(initial_values):
    cv2.namedWindow("Tuning", 0)
    cv2.createTrackbar("x_min","Tuning",initial_values['x_min'],100,no_op)
    cv2.createTrackbar("x_max","Tuning",initial_values['x_max'],100,no_op)
    cv2.createTrackbar("y_min","Tuning",initial_values['y_min'],100,no_op)
    cv2.createTrackbar("y_max","Tuning",initial_values['y_max'],100,no_op)
    cv2.createTrackbar("h_min","Tuning",initial_values['h_min'],180,no_op)
    cv2.createTrackbar("h_max","Tuning",initial_values['h_max'],180,no_op)
    cv2.createTrackbar("s_min","Tuning",initial_values['s_min'],255,no_op)
    cv2.createTrackbar("s_max","Tuning",initial_values['s_max'],255,no_op)
    cv2.createTrackbar("v_min","Tuning",initial_values['v_min'],255,no_op)
    cv2.createTrackbar("v_max","Tuning",initial_values['v_max'],255,no_op)
    cv2.createTrackbar("sz_min","Tuning",initial_values['sz_min'],100,no_op)
    cv2.createTrackbar("sz_max","Tuning",initial_values['sz_max'],100,no_op)

def get_tuning_params():
    trackbar_names = ["x_min","x_max","y_min","y_max","h_min","h_max","s_min","s_max","v_min","v_max","sz_min","sz_max"]
    return {key:cv2.getTrackbarPos(key, "Tuning") for key in trackbar_names}


def wait_on_gui():
    cv2.waitKey(2)


def no_op(x):
    pass

