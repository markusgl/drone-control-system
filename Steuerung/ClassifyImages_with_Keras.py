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

    def __load_image(self,image):
        return cv2.imread(image)

    """
    arg: binary image file
    return: list
    """
    def __reshapeAndPredict(self, image):

        cv2.imshow('image', image)
        img = cv2.resize(image, (128, 128))
        img = np.reshape(img, [1, 128, 128, 3])
        classes = self.model.predict_classes(img)
        return classes

    def __slice(self, Image):
        starttime=time.time()
        ori_img = cv2.imread(Image)
        imgWidth = ori_img.shape[1]
        imgHeight = ori_img.shape[0]
        stopper= False
        partWidth=128
        start = 0
        end = partWidth
        counter=0
        arr=[]
        while not stopper:
            if end >imgWidth:
                stopper=True
                end=imgWidth
                start=end-partWidth
            crop_img = ori_img[0:128, start:end]
            cv2.imwrite("bild"+str(counter)+".jpg", crop_img)
            start=end+20
            end=start+partWidth
            img = np.reshape(crop_img, [1, 128, 128, 3])
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
            classes = np.argmax( self.model.predict(img))
            arr.append(classes)

            #early stop if two in following images
            #was a rope predicted
            # if formerPrediction + classes[0][0] == 2:
            #     break
            # else:
            #     formerPrediction=classes[0][0]

        print('time for prediction (sec): %s' % (time.time() - start))
        return arr

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
    classifier = Classify('PreTrained49-0.01.hdf5')
    pos= classifier.classifyAImage('bild.jpg')
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (20, 400)
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 2
    frame= cv2.imread('bild.jpg')
    cv2.putText(frame, 'Klasse: ' + str(pos) + "%", bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
    cv2.imshow('frame', frame)
    cv2.waitKey(0)


