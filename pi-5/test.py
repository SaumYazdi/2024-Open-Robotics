from modules.settings import get_path
import numpy as np
import cv2

with open(get_path("test.txt"), "rb") as f:
    data = f.read()

    x = np.arange(640*320).reshape(640, 320)
    print(x.shape)

    im = np.frombuffer(data, dtype=np.uint16)
    im.reshape((320, 320))

    cv2.imshow("test", im)
    cv2.waitKey(5000)