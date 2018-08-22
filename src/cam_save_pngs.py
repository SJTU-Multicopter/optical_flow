import cv2
import numpy as np

cap = cv2.VideoCapture(1)
path = "/home/clarence/Desktop/Video/"
counter = 0

while(1):
    # get a frame
    ret, frame = cap.read()
    # show a frame
    cv2.imshow("capture", frame)
    cv2.imwrite(path + str(counter) + ".png", frame)
    counter += 1

    if cv2.waitKey(50) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows() 
