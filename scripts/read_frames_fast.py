from imutils.video import WebcamVideoStream
import numpy as np
import argparse
import imutils
import time
import cv2

print("[INFO] starting video thread...")
wvs = WebcamVideoStream().start()
time.sleep(1.0)


# loop over frames from the video file stream
while True:
	frame = wvs.read()
	#frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	#frame = np.dstack([frame, frame, frame])
	cv2.imshow("Frame", frame)
	cv2.waitKey(1)
	fps.update()

# do a bit of cleanup
cv2.destroyAllWindows()
wvs.stop()
