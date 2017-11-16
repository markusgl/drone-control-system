import cv2

import numpy as np
import cv2
import math
import numpy as np
import json
import os
import sys
import random

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
                crop_img = ori_img[0:partWidth, start:end] # Crop from x, y, w, h -> 100, 200, 300, 400
                # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
                ropePos=self.getRopePos(picture['click-Positions'])
                path=self.checkPos(ropePos,start,end,counter)
                cv2.imwrite(path,crop_img)
                start=end+1
                end=end+partWidth
                counter+=1
                print(path)

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

    def sliceOnlyRope(self):
        for picture in self.jsonData:
            ori_filename = picture['filename']
            ori_img = cv2.imread(ori_filename)
            try:
                print(ori_img.shape)

                imgWidth = ori_img.shape[1]
                imgHeight = ori_img.shape[0]
                ropePos = self.getRopePos(picture['click-Positions'])
                bereich= random.randint(20,108)
                bereich2= random.randint(30,98)
                crop_img1 = ori_img[0:128,ropePos - bereich2:ropePos + (128-bereich2)]
                crop_img2 = ori_img[128:256, ropePos - bereich:ropePos + (128-bereich)]
                cv2.imwrite('D:/tmp/128-128/rope/1-'+os.path.basename(picture['filename']), crop_img1)
                cv2.imwrite('D:/tmp/128-128/rope/2-' + os.path.basename(picture['filename']), crop_img2)
            except:
                print("fehler")

    def sliceNoRope(self):
        files= os.listdir('D:/tmp/NoRope')
        for picture in files:
            ori_img = cv2.imread('D:/tmp/NoRope/'+picture)
            print(ori_img.shape)
            imgWidth = ori_img.shape[1]
            imgHeight = ori_img.shape[0]
            partWidth = math.floor(imgWidth / self.imageParts)
            start=0
            end=partWidth
            while end < imgWidth:
                counter=0
                crop_img = ori_img[0:partWidth, start:end]
                start=end
                end=end+partWidth
                path="D:/tmp/128-128/norope" + str(counter)+'_'+picture
                cv2.imwrite(path, crop_img)
                counter+=1

if __name__ == '__main__':
    test =  ImageSlicer()
    #test.cropImages()
    #test.sliceOnlyRope()
    test.sliceNoRope()
