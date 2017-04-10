import cv2
import numpy as np

#create VideoCapture object
cap = cv2.VideoCapture("test1.avi")

#create blob detector parameter object
params = cv2.SimpleBlobDetector_Params()
#filter by size
params.filterByArea = True
params.minArea = 100
#filter by circularity
params.filterByCircularity = True
params.minCircularity = 0.5
#filter by convexity
params.filterByConvexity = True
params.minConvexity = 0.87
#filter by inertia
params.filterByInertia = True
params.minInertiaRatio = 0.35
#create blob detector with params object
blob_detect = cv2.SimpleBlobDetector(params)

while cap.isOpened():
    #get the current frame
    ret, frame = cap.read()

    #break if frame not read
    if ret == False:
        break

    #__process the image to see only the cardboard

    #__BEGIN HSV METHOD__
    #convert to hsv
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #extract h, s, v channels
    #h, s, v = cv2.split(hsv)
    #threshold the h channel
    #_, h_thresh = cv2.threshold(h, 200, 255, 0)

    #threshold the s channel
    #_, s_thresh = cv2.threshold(s, 150, 255, 0)

    #threshold the v channel
    #_, v_thresh = cv2.threshold(v, 240, 255, 0)

    #combine the channels back together again
    #hsv_thresh = cv2.merge([h_thresh, s_thresh, v_thresh])
    #__END HSV METHOD__

    #__BEGIN PLAIN THRESHOLD METHOD__
    #convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #threshold
    _, gray_thresh = cv2.threshold(gray, 230, 255, 1)
    #erode to kill the noise
    #kernel = np.zeros((11,11),np.uint8)
    #gray_erode = cv2.erode(gray_thresh,kernel,iterations = 1)

    #detect blobs and get keypoints
    keypoints = blob_detect.detect(gray_thresh)

    #draw keypoints
    img_w_keypoints = cv2.drawKeypoints(frame,keypoints,np.array([]),(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #__END PLAIN THRESHOLD METHOD__

    #display the image
    cv2.imshow('cardboard', img_w_keypoints)
    cv2.imshow('original', gray_thresh)

    #quit
    if cv2.waitKey(67) & 0xFF == ord('q'):  #67 ms ~= 15 fps
        break


#cleanup
cap.release()
cv2.destroyAllWindows()
