import cv2
import os

path=os.path.join(os.path.expanduser("~"), "yolo5_images", "10", "10_0.25_0.3125_0_0_0.0.jpg")

if __name__== "__main__":
    img=cv2.imread(path, 1)
    img=img[400:1000, 560:1350]
    h, w, c= img.shape
    for i in range(0, h):
        for j in range (0,w):
            r,g,b= img[i][j]
            if (r==179 or r<=96):
                img[i][j]=(137, 137, 137)
    cv2.imshow("color_changed", img)
    cv2.waitKey(0)
