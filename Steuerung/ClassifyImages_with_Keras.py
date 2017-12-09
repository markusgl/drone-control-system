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

previous_pos = 3
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


    def __reshapeAndPredict(self, image):
        """
        arg: binary image file
        return: list
        """
        cv2.imshow('image', image)
        img = cv2.resize(image, (128, 128))
        img = np.reshape(img, [1, 128, 128, 3])
        classes = self.model.predict_classes(img)
        return classes

    def __slice(self, Image):
        starttime=time.time()
        ori_img = Image#cv2.imread(Image)
        imgWidth = ori_img.shape[1]
        imgHeight = ori_img.shape[0]
        stopper= False
        partWidth=128
        start = 0
        end = partWidth
        counter=0
        arr=[]
        while end < imgWidth:
            # if end >imgWidth:
            #     stopper=True
            #     end=imgWidth
            #     start=end-partWidth
            crop_img = ori_img[0:128, start:end]
            #cv2.imwrite("bild"+str(counter)+".jpg", crop_img)
            start=end-64
            end=start+partWidth
            img = np.reshape(crop_img, [1, 128, 128, 3])
            counter+=1
            arr.append(img)
        #print('time for cropping (sec): %s' % (time.time() - starttime))
        return arr

    def classifyAImage(self, imagePath):
        #start = time.time()
        slicedImagearray= self.__slice(imagePath)
        position=self.__getRopePosition(self.__predict(slicedImagearray))
        #position = self.__predict(slicedImagearray)
        #print('classify image elapsed time (sec): %s' % (time.time() - start))
        return self.__number_to_direction(position)

    def __predict(self,ImgArray):
        arr=[]
        #start = time.time()
        formerPrediction=0

        test2 = np.reshape(np.asarray(ImgArray), [len(ImgArray),128,128,3])
        start = time.time()
        arr= self.model.predict(test2, batch_size=len(ImgArray))
        arr2 =[]
        for val in arr:
            if min(val) > 0.8:
                #print(min(val))
                arr2.append(1)
            else:
                arr2.append(0)
            #arr2.append(int(round(min(val))))
        #print('time for prediction (sec): %s' % (time.time() - start))
        '''
        newArr=[]
        for elem in arr:
            newArr.append(np.argmax(elem))
        return newArr
        '''
        # for img in ImgArray:
        #     classes = np.argmax( self.model.predict(img))
        #     arr.append(classes)
        #
        #     # early stop if two in following images
        #     # was a rope predicted
        #     if formerPrediction + classes == 2:
        #         break
        #     else:
        #         formerPrediction=classes

        #print('time for prediction (sec): %s' % (time.time() - start))
        return arr2


    def __getRopePosition(self,PredictedArray):
        indices=[i for i, x in enumerate(PredictedArray) if x == 1]
        print(PredictedArray)
        global previous_pos
        print("Previous rope position: " + str(previous_pos))

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
            for i in self.direction_to_predictarray(pos):
                if PredictedArray[i] == 1:
                    ropepos = pos
                    previous_pos = pos
                    print("Ropepos: " + str(ropepos))
                    return ropepos

        ropepos = 7
        previous_pos = ropepos
        return ropepos

        #else:
        #positionrelativ = (sum(indices) / len(indices))/len(PredictedArray)
        #return positionrelativ

    # maps tensorflow classes to integer values
    def __number_to_direction(self, arg):
        options = {1: "links",
                   2: "halblinks",
                   3: "mitte",
                   4: "halbrechts",
                   5: "rechts",
                   6: "top",
                   7: "kein Seil",
        }
        return options.get(arg, "nothing")


    def direction_to_predictarray(self, previous_pred):
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


if __name__ == '__main__':

    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify('/Users/mgl/dev/tf_models/HD5/BinaryRopeDetection-06-0.00.hdf5')
    img= cv2.imread('/Users/mgl/tools/IT-Projekt_vids/frames/image160.jpeg')
    pos= classifier.classifyAImage(img)
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (20, 400)
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 2
    frame= cv2.imread('/Users/mgl/tools/IT-Projekt_vids/frames/image300.jpeg')
    cv2.putText(frame, 'Klasse: ' + str(pos) + "%", bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
    cv2.putText(frame, 'Klasse: ' + str(pos), bottomLeftCornerOfText,
                font, fontScale, fontColor, lineType)
    cv2.imshow('frame', frame)
    cv2.waitKey(0)


