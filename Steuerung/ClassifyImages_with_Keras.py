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

previous_pos = 0
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
        
            start=end
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

        return np.argmax(predictionArray)
        # for val in predictionArray:
        #     if min(val) > 0.8:
        #         binaryPrediction.append(1)
        #     else:
        #         binaryPrediction.append(0)
        #
        # return binaryPrediction

    def __getRopePosition(self,PredictedArray):
        indices=[i for i, x in enumerate(PredictedArray) if x == 1]
        print(PredictedArray)

        global previous_pos
        #print("Previous rope position: " + str(previous_pos))

        if previous_pos == 1: #links
            search_area = [1, 2, 3, 4, 5]
        elif previous_pos == 2: #halblinks
            search_area = [2, 1, 3, 4, 5]
        elif previous_pos == 3: #mitte
            search_area = [3, 2, 4, 1, 5]
        elif previous_pos == 4: #halbrechts
            search_area = [4, 3, 5, 2, 1]
        elif previous_pos == 5: #rechts
            search_area = [5, 4, 3, 2, 1]
        #elif previous_pos == 6: #top
            # TODO
        else:
            search_area = [3, 2, 4, 1, 5] #mitte

        for pos in search_area:
            for i in self.__direction_to_predictarray(pos):
                if PredictedArray[i] == 1:
                    ropepos = pos
                    previous_pos = pos
                    #print("Ropepos: " + str(ropepos))
                    return ropepos

        ropepos = 7
        previous_pos = ropepos
        return ropepos

    def __direction_to_predictarray(self, previous_pred):
        if previous_pred == 1: #links
            return [0,1]
        if previous_pred == 2: #halblinks
            return [2,3]
        if previous_pred == 3: #mitte
            return [4,5,6,7]
        if previous_pred == 4: #halbrechts
            return [8,9]
        if previous_pred == 5: #rechts
            return [10,11]

def direction_to_number(self, arg):
    options = {1: "links",
               2: "halblinks",
               3: "mitte",
               4: "halbrechts",
               5: "rechts",
               6: "top",
               7: "kein Seil",
    }
    return options.get(arg, "nothing")


if __name__ == '__main__':
    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify('/Users/mgl/dev/tf_models/HD5/BinaryRopeDetection-06-0.00.hdf5')
    img= cv2.imread('bild.jpg')
    pos= classifier.classifyAImage(img)
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (20, 400)
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 2
    cv2.putText(img, 'Klasse: ' + str(direction_to_number(pos)), bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
    cv2.imshow('frame', img)
    cv2.waitKey(0)


