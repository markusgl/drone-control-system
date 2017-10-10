"""
https://codelabs.developers.google.com/codelabs/tensorflow-for-poets
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import sys

import tensorflow as tf
import os


label_directory="/tf/10000_InceptionV3_Test2/output_labels.txt"
graph_directory="/tf/10000_InceptionV3_Test2/output_graph.pb"
input_layer='DecodeJpeg/contents:0'
output_layer= 'final_result:0'
top_predictions=5
pictureArray=[]
picture_directory='/tmp/ValidationSet/'

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


def run_graph(arrayofPictures, labels, input_layer_name, output_layer_name,
              num_top_predictions):
  with tf.Session() as sess:

    softmax_tensor = sess.graph.get_tensor_by_name(output_layer_name)
    for image in arrayofPictures:
        print(image)
        image_data = load_image(image)
        predictions, = sess.run(softmax_tensor, {input_layer_name: image_data})

        # Sort to show labels in order of confidence
        top_k = predictions.argsort()[-num_top_predictions:][::-1]
        for node_id in top_k:
          human_string = labels[node_id]
          score = predictions[node_id]
          print('%s (score = %.5f)' % (human_string, score))
        print()

        # human_string = labels[top_k[0]]
        # score = predictions[top_k[0]]
        # print('%s (score = %.5f)' % (human_string, score))
        #print()
    return 0


def main(argv):

  for file in os.listdir(picture_directory):
       print(file)
       pictureArray.append(file)

  # load labels
  labels = load_labels(label_directory)

  # load graph, which is stored in the default session
  load_graph(graph_directory)

  run_graph(pictureArray, labels, input_layer, output_layer,
            top_predictions)


if __name__ == '__main__':
  tf.app.run(main=main)
