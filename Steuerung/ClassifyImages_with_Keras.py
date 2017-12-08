"""
Klasse zum Testen des Netzes, macht im Prinzip das gleiche wie Classify_and_output.
Beispielhafte Verwendung am Ende des Skripts
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from keras.applications.mobilenet import MobileNet
import os
import time
import cv2
import math
from keras.models import load_model
import numpy as np
import keras

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
        from keras.utils.generic_utils import CustomObjectScope
        with CustomObjectScope({'relu6': keras.applications.mobilenet.relu6,'DepthwiseConv2D': keras.applications.mobilenet.DepthwiseConv2D}):
            model = load_model(filename)
        return model

    def __slice(self, Image, first):

        ori_img = Image
        imgWidth = ori_img.shape[1]
        imgHeight = ori_img.shape[0]
        if first:
            startHeight = 0
        else:
            startHeight = int(round(imgHeight / 2) - 64)
        partWidth=128
        start = 0
        end = partWidth
        croppedImages=[]
        while end < imgWidth:

            crop_img = ori_img[startHeight:startHeight+128, start:end]
        
            start=end-64
            end=start+partWidth
            img = np.reshape(crop_img, [1, 128, 128, 3])
            croppedImages.append(img)

        return croppedImages

    def classifyAImage(self, imagePath):
        norRopeFound=True
        counter=0
        while norRopeFound:
            slicedImagearray= self.__slice(imagePath,
                                           norRopeFound)
            position=self.__getRopePosition(self.__predict(slicedImagearray))
            if position != -1  or counter==2:
                return position
            elif position ==-1:
                counter+=1

        return position

    def __predict(self,ImgArray):
        predictionArray=[]
        binaryPrediction = []
        ImagesAsTensor = np.reshape(np.asarray(ImgArray), [len(ImgArray),128,128,3])
        predictionArray= self.model.predict(ImagesAsTensor , batch_size=len(ImgArray))

        for val in predictionArray:
            if min(val) > 0.8:
                binaryPrediction.append(1)
            else:
                binaryPrediction.append(0)

        return binaryPrediction

    def __getRopePosition(self,PredictedArray):
        indices=[i for i, x in enumerate(PredictedArray) if x == 1]
        print(PredictedArray)
        if len(indices) ==0:
            #for noRope predicted
            return -1
        else:
            positionrelativ= (sum(indices) / len(indices))/len(PredictedArray)
            return positionrelativ



if __name__ == '__main__':

    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify('BinaryRopeDetection-06-0.00.hdf5')
    img= cv2.imread('bild.jpg')
    pos= classifier.classifyAImage(img)
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (20, 400)
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 2
    cv2.putText(img, 'Klasse: ' + str(pos) + "%", bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
    cv2.imshow('frame', img)
    cv2.waitKey(0)


