import cv2

import numpy as np
import cv2
import math
import numpy as np
import json
import os
import sys

PY3K = sys.version_info >= (3,)
if PY3K:
    from tkinter import filedialog as fd
else:
    from Tkinter import tkFileDialog as fd

class ImageSlicer:

    def __init__(self):
        self.jsonPath = fd.askopenfilename()
        self.jsonData = []
        for line in open(self.jsonPath, 'r'):
            self.jsonData.append(json.loads(line))
        self.imageParts=int(input('In wieviele Bereiche soll ein Bild unterteilt werden: '))


    def cropImages(self):
        counter=0
        for picture in self.jsonData:
            ori_filename = picture['filename']
            ori_img = cv2.imread(ori_filename)
            print(ori_img.shape)

            imgWidth = ori_img.shape[1]
            imgHeight = ori_img.shape[0]
            partWidth = math.floor(imgWidth / self.imageParts)
            start=0
            end=partWidth
            partCounter=0
            while partCounter<=self.imageParts-1:
                partCounter+=1
                crop_img = ori_img[0:imgHeight, start:end] # Crop from x, y, w, h -> 100, 200, 300, 400
                # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
                ropePos=self.getRopePos(picture['click-Positions'])
                pat=self.checkPos(ropePos,start,end,counter)
                cv2.imwrite(pat,crop_img)
                start=end+1
                end=end+partWidth
                counter+=1
                print(pat)

    def getRopePos(self, clickPos):
        if not clickPos:
            print("kann nicht sortiert werden")

        elif len(clickPos) == 2:
            firstClick = clickPos[0]
            secondClick = clickPos[1]

            if firstClick[1] < secondClick[1]:
                upperClickPos = firstClick
                lowerClickPos = secondClick
            else:
                upperClickPos = secondClick
                lowerClickPos = firstClick
        return upperClickPos[0]

    def checkPos(self, ropePos, start, end, pictureCounter):
        path="D:/tmp/"
        if ropePos>= start and ropePos <=end:
            return path+'Rope/RopePic-'+str(pictureCounter)+'.jpg'
        else:
            return path + 'noRope/noRopePic-' + str(pictureCounter) + '.jpg'






    # ori_filename = 'Desert.jpg'
    #
    # new_filename = 'jordan-airball.jpg'
    #
    #
    # print("Reading and displaying file: ", ori_filename)
    #
    # ori_img = cv2.imread(ori_filename)
    # print(ori_img.shape)
    #
    # imgWidth=ori_img.shape[1]
    # imgHeight=ori_img.shape[0]
    # partWidth=math.floor(imgWidth/5)
    #
    # cv2.imshow(ori_filename,ori_img)
    # bereiche= np.empty(5)
    #
    # start=0
    # end=partWidth
    # counter=0
    # for bereich in bereiche:
    #     crop_img = ori_img[0:imgHeight, start:end] # Crop from x, y, w, h -> 100, 200, 300, 400
    #     # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
    #     cv2.imshow("cropped", crop_img)
    #     cv2.imwrite('new'+str(counter)+".jpg",crop_img)
    #     start=end+1
    #     end=end+partWidth
    #     counter=counter+1
    # print(end)
    # cv2.waitKey(0)
    #
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    test =  ImageSlicer()
    test.cropImages()
