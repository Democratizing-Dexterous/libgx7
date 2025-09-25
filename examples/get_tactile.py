from hardware.ser import TactileSerial
import cv2
import time


tactile = TactileSerial(port="/dev/ttyACM0")

tactile.init()

tactile.start()

while True:

    print("Tactile 1 All Forces:", tactile.all_forces1)
    print("Tactile 2 All Forces:", tactile.all_forces2)
    cv2.imshow("Tactile Image", tactile.tactile_image)
    cv2.waitKey(1)
    time.sleep(0.01)
