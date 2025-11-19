import detect_and_count
import cv2 as cv

img=detect_and_count.take_photo()
cv.rectangle(img, (0,0), (1280,400),(0,0,0),-1)


detect_and_count.show_img(img)
cv.waitKey(0)
cv.destroyAllWindows()