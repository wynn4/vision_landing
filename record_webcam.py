import numpy as np
import cv2

cap = cv2.VideoCapture(1)

# Define the codec and create VideoWriter object
#fourcc = cv2.cv.CV_FOURCC(*'DIVX')
#out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
#out = cv2.VideoWriter('output.avi', -1, 15.0, (640,480))    #15fps

#Create VideoWriter object
fps = 15
fourcc = cv2.cv.CV_FOURCC('x', 'v', 'i', 'd')
out = cv2.VideoWriter()
success = out.open('test2.avi',fourcc,fps,(640,480),True)
print success

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        #frame = cv2.flip(frame,0)

        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print 'failed to receive frame'
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
