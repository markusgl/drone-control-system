"""
https://codelabs.developers.google.com/codelabs/tensorflow-for-poets

MG:
This script is only used for testing purposes
Go to ../Steuerung/Rnn.py for the production-level script
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
import os
import logging
import time

""" 
Setup environment (e.g. tf model, path to image files)
"""
os.environ['TF_CPP_MIN_LOG_LEVEL']='2' # turn down log-level to suppress insignificant warning messages
label_directory="/tf/30000_InceptionV3/output_labels.txt"
graph_directory="/tf/30000_InceptionV3/output_graph.pb"
input_layer='DecodeJpeg/contents:0'
output_layer= 'final_result:0'
top_predictions=5
pictureArray=[]
picture_directory='/tmp\ValidationSet/'

def load_image(filename):
  """Read in the image_data to be classified."""
  return tf.gfile.FastGFile(picture_directory+'/'+filename, 'rb').read()

def load_labels(filename):
  return [line.rstrip() for line in tf.gfile.GFile(filename)]

def load_graph(filename):
  """Unpersists graph from file as default graph."""
  with tf.gfile.FastGFile(filename, 'rb') as f:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
    tf.import_graph_def(graph_def, name='')

# maps tensorflow classes to integer values
def direction_to_number(arg):
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
def getRopePosition(image,Session, outputLayer, labels): # TODO speed up the image classification --> Done!

    tmp = 0
    direction = ""
    predictions, = Session.run(outputLayer, {input_layer: image})

    # Sort to show labels in order of confidence
    top_k = predictions.argsort()[-top_predictions:][::-1]
    for node_id in top_k:
        human_string = labels[node_id]
        score = predictions[node_id]

        if (tmp < score):
            tmp = score
            direction = human_string

    direction_number=direction_to_number(direction)
    return direction_number


"""
arg:Outputlayer to Load, Session-Object
return: integer value of tf class
"""
def getOutputLayer(OutputLayer, Session):
    softmax_tensor = Session.graph.get_tensor_by_name(OutputLayer)
    return softmax_tensor

def startSession():
    with tf.Session() as sess:
        return sess

def getAllPictures(Path):
    return os.listdir(Path)


# test the classification
def main(argv):

    pictureArray = getAllPictures(picture_directory)
    # load tf labels
    labels = load_labels(label_directory)
    start = time.time()
    # load tf graph, which is stored in the default session
    load_graph(graph_directory)
    print('load_graph elapsed time (sec): %s ' % (time.time() - start))

    #Load Graph once
    # start = time.time()
    session = startSession()
    #print(' elapsed time (sec): %s' % (time.time() - start))

    #Load outputLayer once
    #start = time.time()
    outputTensor= getOutputLayer(output_layer, session)
    #print('getOutputLayer elapsed time (sec): %s' % (time.time() - start))

    # test classification with single image files

    for image in pictureArray:

        start = time.time()
        image_data = load_image(image)
        print('load_image elapsed time (sec): %s' % (time.time() - start))

        start = time.time()
        dir_number=getRopePosition(image_data, session,outputTensor, labels)
        print("Prediction: %i " % dir_number)
        print('classify image elapsed time (sec): %s' % (time.time() - start))
        print()




# what is going on here?
#-> das sorgt dafür dass wenn es sich hier um eine Klasse oder ein in einem Projekt
# verwendetes Skript handelt, man dieses auch sozusagen einzeln testen kann. Es wird quasi eine
# Main-Methode in diesem Skript ausgeführt und nicht beim starten die Main des Projekts.
# Zumindest versteh ich das so...
if __name__ == '__main__':
  tf.app.run(main=main)
