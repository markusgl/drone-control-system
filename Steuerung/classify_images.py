#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
from keras.models import load_model
import numpy as np
import keras
import tensorflow as tf


class Classify:

    def __init__(self, graph_directory):
        start = time.time()
        os.environ['TF_CPP_MIN_LOG_LEVEL']='2' # turn down log-level to suppress insignificant warning messages
        self.top_predictions=5
        self.model = self.__load_graph(graph_directory)
        self.graph = tf.get_default_graph()
        self.counter=0
        print('Init/Load Grapg, Start Session elapsed time (sec): %s' % (time.time() - start))
        self.no_rope_counter = 0
        self.prediction_array = []
        self.prev_positions = []

    def __load_graph(self, filename):
        from keras.utils.generic_utils import CustomObjectScope
        with CustomObjectScope({'relu6': keras.applications.mobilenet.relu6,
                                'DepthwiseConv2D': keras.applications.mobilenet.DepthwiseConv2D}):
            model = load_model(filename)
        return model

    def __slice(self, image, start_height):
        img_width = image.shape[1]
        partWidth = 128
        start_width = 0
        end_width = partWidth
        cropped_images = []

        while end_width < img_width:
            crop_img = image[start_height:start_height + 128, start_width:end_width]
            start_width = end_width
            end_width = start_width+partWidth

            #Glaub das reshape hier is unnötig?!
            #cropped_images.append(np.reshape(crop_img, [1, 128, 128, 3]))
            cropped_images.append(crop_img)
        return cropped_images

    def classify_image(self, image_path):
        """
        - Try to detect rope in top most image slice
        - If no rope is detected, try to find rope in the middle image slice
        :param image_path:
        :return: 0 to 4: rope position left to right
                -1: no rope found
        """
        with self.graph.as_default():
            start_height = 0

            for i in range(2):
                sliced_image_array = self.__slice(image_path, start_height)
                position = self.__predict(sliced_image_array)
                if position > -1:
                    self.no_rope_counter = 0
                    #self.prev_position = position
                    #cv2.imshow("rope", sliced_image_array[position])
                    return position
                start_height = 64

            self.no_rope_counter += 1
            print("no_rope_counter: " + str(self.no_rope_counter))
            if self.no_rope_counter > 10:
                position = 5 #top
                #self.prev_position = position

            return position

    def __predict(self, img_array):
        """
        :param img_array: array of images
        :return: 0 to 4: rope position left to right
                -1: no rope found
        """
        images_as_tensor = np.reshape(np.asarray(img_array), [len(img_array), 128, 128, 3])
        self.prediction_array = self.model.predict(images_as_tensor, batch_size=len(img_array))
        #self.__save_cropped_images(img_array, self.prediction_array)

        orig_prediction_array = self.prediction_array
        for index in self.prev_positions:
            self.prediction_array[index] *= 1.5

        self.prev_positions = np.argwhere(orig_prediction_array == np.amax(orig_prediction_array) )

        #norope threshold
        if max(self.prediction_array) < 0.9:  # TODO - Wert evtl. anpassen -> Praxis
            #print("no rope found")
            return -1
        print(self.prediction_array)

        return np.argmax(self.prediction_array)

    def __save_cropped_images(self, img_array, prediction_array):

        dir= 'D:\\tmp\\'
        file = str(self.counter) + '.jpg'
        for i, img in enumerate(img_array):
            self.counter+=1
            if prediction_array[i] >= 0.85:
                subdir='rope'
            else:
                subdir='noRope'
            img=np.reshape(img, [ 128, 128, 3])
            path= os.path.join(dir,subdir,file)
            cv2.imwrite(path,img)



if __name__ == '__main__':
    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify('..Models/BinaryRopeDetection-06-0.00.hdf5')
    img = cv2.imread('bild.jpg')
    img = cv2.resize(img, (0, 0), fx=1, fy=1)
    pos = classifier.classify_image(img)
    font = cv2.FONT_HERSHEY_SIMPLEX

    middleLeftCornerOfText = (20, 280)
    bottomLeftCornerOfText = (20, 300)
    fontScale = 1
    fontColor = (255, 255, 255)
    lineType = 2
    cv2.putText(img, 'Klasse: ' + str(pos), middleLeftCornerOfText, font, fontScale, fontColor, lineType)

    np.set_printoptions(precision=2)
    cv2.putText(img, 'Array: ' + str(classifier.prediction_array).replace('\n', ''), bottomLeftCornerOfText, font, 0.6, fontColor, 1)
    cv2.imshow('frame', img)
    cv2.waitKey(0)

