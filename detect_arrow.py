# import the necessary packages
from arrowDetector import arrowDetector
import argparse
import imutils
import cv2


def get_arrow(cap):
    # Capture frame 
    ret, frame = cap.read()

    # Mirror the frame
    frame = cv2.flip(frame, 1)
    frame = frame[:,:]

    # Convert to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(frame, width=500)
    ratio = frame.shape[0] / float(resized.shape[0])

    # convert the resized image to grayscale, blur it slightly,
    # and threshold it
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11 ,2)

    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh, cv2.RETR_LIST,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # get object to fit it to a pre-set shape if possible using cv2.approxPolyDP()
    sd = arrowDetector()

    # loop over the contours
    for c in cnts:
        # fit it to a pre-set shape if possible using cv2.approxPolyDP()
        vertices = sd.detect(c)

        # check if has same number of edges as an arrow
        if(len(vertices) == 7):
            # compute moments of contour
            M = cv2.moments(c)

            # check if it fits size requirements
            if(M["m00"] < 3500):
                continue
            else:

                #compute centre of contour
                cX = int((M["m10"] / M["m00"]))
                cY = int((M["m01"] / M["m00"]))

# if centre is white AND centre of arrow is not out of bounds (not a false detection)
                if(gray_frame[int(cY*ratio)][int(cX*ratio)] >= 150 and cY <= 300 and cY >= 100):
                    c = c.astype("int")
                    cv2.drawContours(thresh, [c], 0, (0,0,0), 5)
                    print(cX,cY)

                # arrow is beside robot as area is big
                if(M["m00"] > 9500):
                    position = 1

                # arrow is one grid away, determine position of arrow
                else:
                    
                    # get arrow positioning based on centre of contour
                    if(cX < 150):
                        position = 0
                    elif(cX < 350):
                        position = 1
                    else:
                        position = 2
                
                # annotation for display
                cv2.putText(thresh, "ARROW at {}".format(position), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 0), 2)

                print("ARROW FOUND")
                return position
    # Display the resulting frame
    cv2.imshow("Image", thresh)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return 9
    return 9
# When everything done, release the capture
cv2.destroyAllWindows()

"""
Modified by James Tan on 2/4/2019 for MDP Group 13 AY1718S2
"""
