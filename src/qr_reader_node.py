#!/usr/bin/env python3

import threading
import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

try:
    from cv_bridge import CvBridge, CvBridgeError
    import cv2
    _OPENCV_AVAILABLE = True
except ImportError:
    _OPENCV_AVAILABLE = False
    CvBridge = None
    cv2 = None

class QRReader:
    def __init__(self):
        rospy.init_node("qr_reader_node")
        self._lock = threading.Lock()
        self._bridge = CvBridge() if _OPENCV_AVAILABLE else None

        self._latest_image = None

        if _OPENCV_AVAILABLE:
            self._qr_detector = cv2.QRCodeDetector()

        image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        
        self._sub = rospy.Subscriber(image_topic, Image, self._image_cb, queue_size=1)
 
        self._srv = rospy.Service("/qr_reader_node/scan", Trigger, self._handle_scan)

        rospy.loginfo(f"QR Okuyucu Hazır. Topic: {image_topic}")

    def _image_cb(self, msg: Image):
        if not _OPENCV_AVAILABLE:
            return
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._latest_image = cv_image
        except Exception as exc:
            pass

    def _handle_scan(self, _req: TriggerRequest) -> TriggerResponse:
        timeout = 5.0 
        start_time = time.time()
        rospy.loginfo("QR taranıyor...")

        found_qr = None

        while (time.time() - start_time) < timeout:

            img_to_scan = None
            with self._lock:
                if self._latest_image is not None:
                    img_to_scan = self._latest_image.copy()

            if img_to_scan is not None:
                try:
 
                    gray = cv2.cvtColor(img_to_scan, cv2.COLOR_BGR2GRAY)
                    data, bbox, _ = self._qr_detector.detectAndDecode(gray)
                    
                    if data:
                        found_qr = data
                        break 
                except Exception as e:
                    pass
            time.sleep(0.1)

        if found_qr:
            rospy.loginfo(f"BULUNDU: {found_qr}")
            return TriggerResponse(success=True, message=found_qr)
        else:
            rospy.logwarn("QR bulunamadı.")
            return TriggerResponse(success=False, message="TIMEOUT")

def main():
    QRReader()
    rospy.spin()

if __name__ == "__main__":
    main()