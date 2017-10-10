
"""
NOTE: To learn to use this file and retrain.py, please see:
https://codelabs.developers.google.com/codelabs/tensorflow-for-poets
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import sys
import os
import  struct

import tensorflow as tf

label_directory="/tf/5000_MobileNet_1.0_244/output_labels.txt"
graph_directory="/tf/5000_MobileNet_1.0_244/output_graph.pb"
input_layer='input:0'
output_layer= 'final_result:0'
top_predictions=5
pictureArray=[]
picture_directory='/tmp/ValidationSet/'

def load_image(filename):
  return read_tensor_from_image_file(picture_directory + filename,
                                     input_height=224,
                                     input_width=224,
                                     input_mean=0,
                                     input_std=225)

def load_labels(filename):
  """Read in labels, one label per line."""
  return [line.rstrip() for line in tf.gfile.GFile(filename)]


def load_graph(filename):
  """Unpersists graph from file as default graph."""
  with tf.gfile.FastGFile(filename, 'rb') as f:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(f.read())
    tf.import_graph_def(graph_def, name='')


def run_graph(arrayofPictures, labels, input_layer_name, output_layer_name, num_top_predictions):
  with tf.Session() as sess:

    softmax_tensor = sess.graph.get_tensor_by_name(output_layer_name)
    for image in arrayofPictures:
        print(image)
        image_data = load_image(image)

        predictions, = sess.run(softmax_tensor, {input_layer_name: image_data})

        top_k = predictions.argsort()[-num_top_predictions:][::-1]
        for node_id in top_k:
          human_string = labels[node_id]
          score = predictions[node_id]
          print('%s (score = %.5f)' % (human_string, score))
        print()

        # human_string = labels[top_k[0]]
        # score = predictions[top_k[0]]
        # print('%s (score = %.5f)' % (human_string, score))
        print()

    return 0

def read_tensor_from_image_file(file_name, input_height=299, input_width=299, input_mean=0, input_std=255):

  input_name = "file_reader"
  output_name = "normalized"
  file_reader = tf.read_file(file_name, input_name)
  if file_name.endswith(".png"):
    image_reader = tf.image.decode_png(file_reader, channels = 3,
                                       name='png_reader')
  elif file_name.endswith(".gif"):
    image_reader = tf.squeeze(tf.image.decode_gif(file_reader,
                                                  name='gif_reader'))
  elif file_name.endswith(".bmp"):
    image_reader = tf.image.decode_bmp(file_reader, name='bmp_reader')
  else:
    image_reader = tf.image.decode_jpeg(file_reader, channels = 3,
                                        name='jpeg_reader')
  float_caster = tf.cast(image_reader, tf.float32)
  dims_expander = tf.expand_dims(float_caster, 0);
  resized = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
  normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
  sess = tf.Session()
  result = sess.run(normalized)

  return result

def main(argv):

  for file in os.listdir(picture_directory):
       print(file)
       pictureArray.append(file)

  # load labels
  labels = load_labels(label_directory)

  # load graph, which is stored in the default session
  load_graph(graph_directory)

  run_graph(pictureArray, labels, input_layer, output_layer, top_predictions)

if __name__ == '__main__':
  tf.app.run(main=main)
  input_height = 244
  input_width = 244
  input_mean = 0
  input_std = 255
