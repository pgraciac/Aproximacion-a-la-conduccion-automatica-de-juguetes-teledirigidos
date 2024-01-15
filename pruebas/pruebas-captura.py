import cv2
import time

cap = cv2.VideoCapture(1)
cap.read()
cap.read()
ret, frame = cap.read()
if ret:
    cv2.imshow('frame', frame)
    cv2.waitKey(0)
else:
    print("no se ha podido")
cap.release()
cv2.destroyAllWindows()