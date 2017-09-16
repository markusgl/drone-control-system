"""
Klasse zum Testen des Netzes, macht im Prinzip das gleiche wie Classify_and_output.
Beispielhafte Verwendung am Ende des Skripts
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
import os
import logging
import time

class Classify:

    def __init__(self,label_directory, graph_directory, input_layer, output_layer):
        start = time.time()
        os.environ['TF_CPP_MIN_LOG_LEVEL']='2' # turn down log-level to suppress insignificant warning messages
        self.input_layer=input_layer
        self.top_predictions=5

        #Session wird im Konstruktor bereits erstellt.
        self.session= self.startSession()
        self.__load_graph(graph_directory)
        self.labels=self.__load_labels(label_directory)
        self.output_layer=self.__getOutputLayer(output_layer, self.session)
        print('Init/Load Grapg, Start Session elapsed time (sec): %s' % (time.time() - start))

    def __load_image(self,filename):
      return tf.gfile.FastGFile(filename, 'rb').read()

    def __load_labels(self, filename):
      return [line.rstrip() for line in tf.gfile.GFile(filename)]

    def __load_graph(self, filename):
      with tf.gfile.FastGFile(filename, 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        tf.import_graph_def(graph_def, name='')

    # maps tensorflow classes to integer values
    def __direction_to_number(self, arg):
        options = {"links" : 1,
                   "halblinks" : 2,
                   "mitte" : 3,
                   "halbrechts" : 4,
                   "rechts" : 5,
                   "top" : 6,
                   "norope" : 7,
        }
        return options.get(arg, "nothing")

    """
    arg: binary image file
    return: integer value of tf class
    """
    def __getRopePosition(self, image,Session, outputLayer, labels):

        tmp = 0
        direction = ""
        predictions, = Session.run(outputLayer, {self.input_layer: image})

        # Sort to show labels in order of confidence
        top_k = predictions.argsort()[-self.top_predictions:][::-1]
        for node_id in top_k:
            human_string = labels[node_id]
            score = predictions[node_id]

            if (tmp < score):
                tmp = score
                direction = human_string

        direction_number = self.__direction_to_number(direction)
        return direction_number


    """
    arg:Outputlayer to Load, Session-Object
    return: integer value of tf class
    """
    def __getOutputLayer(self, OutputLayer, Session):
        softmax_tensor = Session.graph.get_tensor_by_name(OutputLayer)
        return softmax_tensor

    def startSession(self):
        with tf.Session() as sess:
            return sess

    """
      Call this Funtion from your Class.
      return: integer value of tf class
      """
    def classifyAImage(self, imagePath):
        image= self.__load_image(imagePath)
        start = time.time()
        predictedClass = self.__getRopePosition(image, self.session, self.output_layer,self.labels)
        print('classify image elapsed time (sec): %s' % (time.time() - start))
        return predictedClass


if __name__ == '__main__':

    #Objekterzeugung mit Kontruktoraufruf
    classifier = Classify("/tf/30000_InceptionV3/output_labels.txt","/tf/30000_InceptionV3/output_graph.pb",
                            'DecodeJpeg/contents:0','final_result:0')
    directory= '/tmp/ValidationSet/'
    pictureArray =  os.listdir(directory)

    #For-Schleife geht alle in einem Ordner gefunden Dateien durch.
    for image in pictureArray:
        #Aufruf einer Methode des oben erzeugten Objekts
        dir_number = classifier.classifyAImage(directory+image)
        print(directory+image)
        print("Prediction: %i " % dir_number)
        print()


