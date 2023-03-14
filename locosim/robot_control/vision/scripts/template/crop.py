import cv2
import numpy as np


img=cv2.imread("/home/annachiara/template/6/to_crop4.jpg", 0)
print(img.shape)
#img=img[482:535, 945:975]
#img=img[475:535, 845:900]
#img=img[480:530, 825:868]
#img=img[480:530, 1050:1096]
img=img[70:133, 389:412]
cv2.imwrite("/home/annachiara/template/6/6_0.75_0.475_4.71238898038469_0_4.71238898038469.jpg", img)
