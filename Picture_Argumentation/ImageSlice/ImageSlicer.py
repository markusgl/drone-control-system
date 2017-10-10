import cv2

import numpy as np
import cv2
import math
import numpy as np


ori_filename = 'Desert.jpg'

new_filename = 'jordan-airball.jpg'


print("Reading and displaying file: ", ori_filename)

ori_img = cv2.imread(ori_filename)
print(ori_img.shape)

imgWidth=ori_img.shape[1]
imgHeight=ori_img.shape[0]
partWidth=math.floor(imgWidth/5)

cv2.imshow(ori_filename,ori_img)
bereiche= np.empty(5)

start=0
end=partWidth
counter=0
for bereich in bereiche:
    crop_img = ori_img[0:imgHeight, start:end] # Crop from x, y, w, h -> 100, 200, 300, 400
    # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
    cv2.imshow("cropped", crop_img)
    cv2.imwrite('new'+str(counter)+".jpg",crop_img)
    start=end+1
    end=end+partWidth
    counter=counter+1
print(end)
cv2.waitKey(0)

cv2.destroyAllWindows()

