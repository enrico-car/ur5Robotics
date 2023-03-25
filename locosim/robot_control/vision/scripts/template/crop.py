import cv2
import numpy as np


img=cv2.imread("./0/to_crop.jpg", 0)
print(img.shape)
#img=img[482:535, 945:975]
#img=img[475:535, 845:900]
#img=img[480:530, 825:868]
#img=img[480:530, 1050:1096]
img=img[78:110, 385:414]
cv2.imwrite("./0/0_0.8333_0.475_1.5708_0_1.9635.jpg", img)
