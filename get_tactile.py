from hardware.ser import TactileSerial
import cv2
import time


tactile = TactileSerial(port="/dev/ttyACM0")

tactile.init()

while True:
    data = tactile.read_forces()
    if data["valid"]:
        print("Tactile 1 All Forces:", data["tac_allforces1"])
        print("Tactile 2 All Forces:", data["tac_allforces2"])
        cv2.imshow("Tactile Image", data["tac_img"])
        cv2.waitKey(1)
    time.sleep(0.01)
