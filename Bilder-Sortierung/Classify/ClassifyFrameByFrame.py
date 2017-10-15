import numpy as np
import cv2
import ctypes
from ClassifiyImages import Classify

user32 = ctypes.windll.user32
screen_res = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)
classifier= Classify("D:/tf/Inception_10000_6600Images/output_labels.txt", "D:/tf/Inception_10000_6600Images/output_graph.pb", 'DecodeJpeg/contents:0','final_result:0')
count=0

cap = cv2.VideoCapture("D:\Download\Seil am Fels\Seil am Fels/IMG_1939.m4v")
while(cap.isOpened()):
    ret, frame = cap.read()

    count = count + 1
    rope_image = 'image' + str(count) + '.jpeg'
    print("Received an image: " + rope_image)

    cv2.imwrite("D:/tmp/Tobi03/"+rope_image, frame)
    ropePosition = classifier.classifyAImage("D:/tmp/Tobi03/"+rope_image)
    print(ropePosition)

    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()