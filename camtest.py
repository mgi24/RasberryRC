
import cv2
from picamera2 import Picamera2



def main():
    picam2 = Picamera2()
    # Request BGR output from the OV5647 to match OpenCV's default BGR color order
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        raw={"size": (picam2.sensor_resolution)})
    picam2.configure(config)
    picam2.start()
    cv2.namedWindow("Picamera2", cv2.WINDOW_AUTOSIZE)
    try:
        # create control sliders
        cv2.createTrackbar("AutoExposure (1=yes)", "Picamera2", 1, 1, lambda v: None)
        cv2.createTrackbar("Exposure (us)", "Picamera2", 10000, 1000000, lambda v: None)
        cv2.createTrackbar("AnalogueGain x100", "Picamera2", 100, 1600, lambda v: None)

        while True:
            frame = picam2.capture_array()
            # read sliders
            ae_on = cv2.getTrackbarPos("AutoExposure (1=yes)", "Picamera2") == 1
            exposure_us = max(1, cv2.getTrackbarPos("Exposure (us)", "Picamera2"))
            gain_val = max(100, cv2.getTrackbarPos("AnalogueGain x100", "Picamera2")) / 100.0

            # apply controls: enable auto exposure if AE slider set, otherwise apply manual exposure+gain
            if ae_on:
                picam2.set_controls({"AeEnable": True})
            else:
                picam2.set_controls({"AeEnable": False, "ExposureTime": int(exposure_us), "AnalogueGain": float(gain_val)})

            cv2.imshow("Picamera2", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()