import cv2
import numpy as np


img=cv2.imread("./3/to_crop1.jpg", 0)
print(img.shape)
#img=img[482:535, 945:975]
#img=img[475:535, 845:900]
#img=img[480:530, 825:868]
#img=img[480:530, 1050:1096]
img=img[360:425, 380:420]
cv2.imwrite("./3/3_0.1667_0.475_1.5708_0_1.5708.jpg", img)

