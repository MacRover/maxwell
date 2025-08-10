import cv2
from cv2 import aruco
import numpy as np

class MarkerDetectionSystem:
    def __init__(self):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.detected_markers = []

    def detect_markers(self, frame):
        self.detected_markers = []

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
                    self.detected_markers.append(marker_info)
        return self.detected_markers

def main():
    
    cap = cv2.VideoCapture('udpsrc port=40627 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" !   rtpjitterbuffer latency=50 !   rtph265depay !   h265parse !   queue max-size-buffers=20 max-size-time=0 max-size-bytes=0 leaky=downstream !   avdec_h265 !   videoconvert ! appsink', cv2.CAP_GSTREAMER) #change that to the gstream pipeline   
    mds = MarkerDetectionSystem()
    try:
        while True:
            ret, frame = cap.read()

            detected_markers = mds.detect_markers(frame)
            # print(detected_markers)

            for item in detected_markers:
                # print(type(item['Corners'][1][1]))
                cv2.rectangle(frame, (int(item['Corners'][0][0]), int(item['Corners'][0][1])), (int(item['Corners'][2][0]), int(item['Corners'][2][1])), (0, 255, 0), 5)
                cv2.putText(frame, str(item["ID"][0]), (int(item['Centroid'][0]), int(item['Centroid'][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 5)
            
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()