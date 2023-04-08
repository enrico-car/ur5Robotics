import numpy as np
import cv2

img = cv2.imread("/home/carro/yolo5_images/0.5_0.475/7_0.5_0.475_0_0_0.0.jpg", 0) #riempi, 0 per grayscale
template = cv2.imread("/home/carro/yolo5_images/0.1667_0.475/7_0.1667_0.475_0_0_0.0.jpg", 0) #riempi

methods = [cv2.TM_CCOEFF, cv2.TM_CCOEFF_NORMED, cv2.TM_CCORR, cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED] #all possible methods
#try with all the methods

# for m in methods:
#     img2 = img.copy()
#     result = cv2.matchTemplate(img2, template, m) #dim is (W-w+1, H-h+1), upper is value of base img
#     min_val, max_val, min_loc, max_loc =cv2.minMaxLoc(result)
#     if m in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]: #take min_val
#         location=min_loc
#     else:
#         location=max_loc

#     bottom_right = (location[0]+w, location[1]+ h) 
#     cv2.rectangle(img2, location, bottom_right, 255, 5)
#     cv2.imshow("Match", img2)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

img_h, img_w = img.shape
print(img.shape)

template = cv2.resize(template, (img_w-1, img_h-1))
print(template.shape)

method = cv2.TM_CCORR_NORMED
result = cv2.matchTemplate(img, template, method)
print(result)

