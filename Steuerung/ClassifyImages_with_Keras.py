"""
Klasse zum Testen des Netzes, macht im Prinzip das gleiche wie Classify_and_output.
Beispielhafte Verwendung am Ende des Skripts
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import time
import cv2
import math
from keras.models import load_model
import numpy as np

class Classify:

    def __init__(self, graph_directory):
        start = time.time()
        os.environ['TF_CPP_MIN_LOG_LEVEL']='2' # turn down log-level to suppress insignificant warning messages
        self.top_predictions=5
        self.model=self.__load_graph(graph_directory)
        self.__compileModel()
        print('Init/Load Grapg, Start Session elapsed time (sec): %s' % (time.time() - start))


    def __compileModel(self):
        self.model.compile(loss='binary_crossentropy',
                      optimizer='rmsprop',
                      metrics=['accuracy'])

    def __load_graph(self, filename):
       return load_model(filename)

    def __load_image(self,image):
        return cv2.imread(image)

    """
    arg: binary image file
    return: list
    """
    def __reshapeAndPredict(self, image):

        cv2.imshow('image', image)
        img = cv2.resize(image, (159, 160))
        img = np.reshape(img, [1, 159, 160, 3])
        classes = self.model.predict_classes(img)
        return classes




    def __slice(self, Image):
        starttime=time.time()
        ori_img = cv2.imread(Image)
        imgWidth = ori_img.shape[1]
        imgHeight = ori_img.shape[0]
        stopper= False
        partWidth=159
        start = 0
        end = partWidth
        counter=0
        arr=[]
        while not stopper:
            if end >imgWidth:
                stopper=True
                end=imgWidth
                start=end-partWidth
            crop_img = ori_img[0:160, start:end]
            start+=70
            end=start+partWidth
            img = np.reshape(crop_img, [1, 159, 160, 3])
            counter+=1
            arr.append(img)
        print('time for cropping (sec): %s' % (time.time() - starttime))
        return arr


    def classifyAImage(self, imagePath):
        start = time.time()
        slicedImagearray= self.__slice(imagePath)
        position=self.__getRopePosition(self.__predict(slicedImagearray))
        print('classify image elapsed time (sec): %s' % (time.time() - start))
        return position

    def __predict(self,ImgArray):
        arr=[]
        start = time.time()
        formerPrediction=0
        for img in ImgArray:
            classes = self.model.predict_classes(img)
            arr.append(classes[0][0])

            #early stop if two in following images
            #was a rope predicted
            if formerPrediction + classes[0][0] == 2:
                break
            else:
                formerPrediction=classes[0][0]

        print('time for prediction (sec): %s' % (time.time() - start))
        return arr

    def __getRopePosition(self,PredictedArray):
        indices=[i for i, x in enumerate(PredictedArray) if x == 1]

        if len(indices) ==0:
            #for noRope predicted
            return -1
        else:
            positionrelativ= ((sum(indices)/len(indices))/10)*100
            return positionrelativ



if __name__ == '__main__':

    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify('../models/TestModel_input159-160.h5')
    classifier.classifyAImageAtOnce('bild.jpg')



