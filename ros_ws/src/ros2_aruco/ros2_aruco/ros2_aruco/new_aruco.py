import cv2
from cv2 import aruco
import numpy as np

class MarkerDetectionSystem:
    def __init__(self):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.detected_markers = {}

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if ids is not None:
            for i in range(len(ids)):
                M = cv2.moments(corners[i][0])
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    marker_info = {
                        'ID': ids[i],
                        'Centroid': (cX, cY),
                        'Corners': corners[i][0]
                    }
                    self.detected_markers[ids[i][0]] = marker_info
        return self.detected_markers
def main():
    cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER) #change that to the gstream pipeline   
    mds = MarkerDetectionSystem()

    while True:
        ret, frame = cap.read()

        detected_markers = mds.detect_markers(frame)
        mds.draw_markers(frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()