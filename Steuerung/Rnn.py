"""
https://codelabs.developers.google.com/codelabs/tensorflow-for-poets
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf
import os
import logging

os.environ['TF_CPP_MIN_LOG_LEVEL']='2' # turn down loglevel to suppress warning messages
label_directory="models/retrained_labels.txt"
graph_directory="models/retrained_graph.pb"
input_layer='DecodeJpeg/contents:0'
output_layer= 'final_result:0'
top_predictions=5
pictureArray=[]
picture_directory='tmp/ValidationSet_debug'

def load_image(filename):
  """Read in the image_data to be classified."""
  return tf.gfile.FastGFile(picture_directory+'/'+filename, 'rb').read()

def load_labels(filename):
  return [line.rstrip() for line in tf.gfile.GFile(filename)]

def load_graph():
  """Unpersists graph from file as default graph."""
  with tf.gfile.FastGFile(graph_directory, 'rb') as f:
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

# arg: binary image file
# return: integer value of tf class
def getRopePosition(image): #TODO speed up the image classification
    with tf.Session() as sess:
        softmax_tensor = sess.graph.get_tensor_by_name(output_layer)
        labels = load_labels(label_directory)
        tmp = 0
        direction = ""
        predictions, = sess.run(softmax_tensor, {input_layer: image})

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

