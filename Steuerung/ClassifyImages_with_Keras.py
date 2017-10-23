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
    def __getRopePosition(self, image):

        cv2.imshow('image', image)
        img = cv2.resize(image, (159, 160))
        img = np.reshape(img, [1, 159, 160, 3])
        classes = self.model.predict_classes(img)
        return classes

    """
      Call this Funtion from your Class.
      return: integer value of tf class
      """
    def classifyAImage(self, imagePath):
        start = time.time()
        images= self.__sliceImage(imagePath,5)
        co=1
        predictedClass=7
        for image in images:
            image= self.__load_image(image)
            start = time.time()
            if self.__getRopePosition(image):
                predictedClass=co
            else:
                co+=1

        print('classify image elapsed time (sec): %s' % (time.time() - start))
        return predictedClass



    def __sliceImage(self, image, slices):
        ori_img = cv2.imread(image)
        imgWidth = ori_img.shape[1]
        imgHeight = ori_img.shape[0]
        partWidth = math.floor(imgWidth / slices)
        start = 0
        end = partWidth
        partCounter = 0
        croptImages=[]
        while partCounter <= slices - 1:
                partCounter += 1
                crop_img = ori_img[0:partWidth, start:end]
                start=end
                end+=partWidth
                cv2.imwrite("part"+str(partCounter)+".jpg", crop_img)
                croptImages.append("part"+str(partCounter)+".jpg")
        return croptImages

if __name__ == '__main__':

    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify('/models/TestModel_input159-160.h5')
    directory= 'G:/test/'
    pictureArray =  os.listdir(directory)

    #For-Schleife geht alle in einem Ordner gefunden Dateien durch.
    for image in pictureArray:
        #Aufruf einer Methode des oben erzeugten Objekts
        dir_number = classifier.classifyAImage(directory+image)
        print(directory+image)
        print("Prediction: " + dir_number)
        print()


