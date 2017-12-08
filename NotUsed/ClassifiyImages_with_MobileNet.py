"""
Klasse zum Testen des Netzes, macht im Prinzip das gleiche wie Classify_and_output.
Beispielhafte Verwendung am Ende des Skripts
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
import os
import time
import cv2
import math

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
        return self.__read_tensor_from_image_file( filename, input_height=128,
                                           input_width=128,
                                           input_mean=0,
                                           input_std=225)

    def __load_labels(self, filename):
      return [line.rstrip() for line in tf.gfile.GFile(filename)]

    def __load_graph(self, filename):
      with tf.gfile.FastGFile(filename, 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        tf.import_graph_def(graph_def, name='')

    def __getRopePosition(self, image,Session, outputLayer, labels):

        predictions, = Session.run(outputLayer, {self.input_layer: image})

        # Sort to show labels in order of confidence
        top_k = predictions.argsort()[-self.top_predictions:][::-1]

        for node_id in top_k:
          human_string = labels[node_id]
          score = predictions[node_id]
          #print('%s (score = %.5f)' % (human_string, score))
        #print()
        if labels[top_k[0]]=='rope':
            direction=True
        else:
            direction = False

        return  direction

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
        images= self.__sliceImage(imagePath,5)
        co=0
        predictionArray=[]
        for image in images:
            image= self.__load_image(image)
            start = time.time()
            predictionArray.append(self.__getRopePosition(image, self.session, self.output_layer,self.labels))

        indices = [i for i, x in enumerate(predictionArray) if x == 1]
        print(predictionArray)
        if len(indices) == 0:
                # for noRope predicted
            return -1
        else:
            positionrelativ = (sum(indices) / len(indices))/len(predictionArray)
            return positionrelativ


    def __read_tensor_from_image_file(self,file_name, input_height=128, input_width=128, input_mean=0, input_std=255):

        input_name = "file_reader"
        output_name = "normalized"
        file_reader = tf.read_file(file_name, input_name)

        image_reader = tf.image.decode_jpeg(file_reader, channels=3,
                                                name='jpeg_reader')
        float_caster = tf.cast(image_reader, tf.float32)
        dims_expander = tf.expand_dims(float_caster, 0);
        resized = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
        normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
        start = time.time()
        result = self.session.run(normalized)
        #print('load tensor: %s' % (time.time() - start))
        return result

    def __sliceImage(self, image, slices):
        start = time.time()
        ori_img = cv2.imread(image)
        imgWidth = ori_img.shape[1]
        imgHeight = ori_img.shape[0]

        #partWidth = math.floor(imgWidth / slices)
        partWidth=128
        start = 0
        end = partWidth
        partCounter = 0
        croptImages=[]
        while end<imgWidth:
                partCounter += 1
                crop_img = ori_img[0:partWidth, start:end]
                cv2.imwrite("part"+str(partCounter)+".jpg", crop_img)
                croptImages.append("part"+str(partCounter)+".jpg")
                start=end
                end=end+partWidth
        print('slice image: %s' % (time.time() - start))
        print(len(croptImages))
        return croptImages

if __name__ == '__main__':

    input_height = 128
    input_width = 128
    input_mean = 0
    input_std = 255

    classifier = Classify("D:/tf/output_labels.txt","D:/tf/output_graph.pb",
                            'input:0','final_result:0')

    directory= 'D:/tmp/Neuer Ordner/'
    pictureArray =  os.listdir(directory)

    #For-Schleife geht alle in einem Ordner gefunden Dateien durch.
    for image in pictureArray:
        #Aufruf einer Methode des oben erzeugten Objekts
        print(classifier.classifyAImage(directory+image))

        print()


