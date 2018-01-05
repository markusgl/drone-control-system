# Filtervisualisierung analog zu https://github.com/keras-team/keras/blob/master/examples/conv_filter_visualization.py
from __future__ import print_function
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D, Activation, Dropout, Flatten, Dense
import h5py
from scipy.misc import imsave
import numpy as np
import time
from keras import backend as K
from keras.models import load_model
from keras.utils.generic_utils import CustomObjectScope
from keras.applications import mobilenet

# Zu setzende Parameter für das Netz
eingabe_width = 128
eingabe_height = 128
name_der_schicht = 'conv_dw_7'
anzahl_filter_auf_schicht = 25 # kann auch kleiner als tatsächliche Anzahl sein
anzahl_ausgabe_bilder = 5 # z.B. 5 = 5*5 Matrix Bilder
anzahl_gradient_durchlauefe = 20 # Eine Art Intensität

K.set_learning_phase(0)

# Laden des Models
#model = load_model('Selbstgestelt-E4-.hdf5')
with CustomObjectScope({'relu6': mobilenet.relu6, 'DepthwiseConv2D': mobilenet.DepthwiseConv2D}):
    #model = load_model('RopePrediction-01-0.03.hdf5')
	model = load_model('MobileNet-Loss_0.01_Batch_32_ACC_0.098_8EP.hdf5')

# util function to convert a tensor into a valid image
def deprocess_image(x):
    # normalize tensor: center on 0., ensure std is 0.1
    x -= x.mean()
    x /= (x.std() + K.epsilon())
    x *= 0.1

    # clip to [0, 1]
    x += 0.5
    x = np.clip(x, 0, 1)

    # convert to RGB array
    x *= 255
    if K.image_data_format() == 'channels_first':
        x = x.transpose((1, 2, 0))
    x = np.clip(x, 0, 255).astype('uint8')
    return x

# Alle Schichten ausgeben um Namen der Schichten zu erkennen ...
model.summary()

eingabe_img = model.input

# get the symbolic outputs of each "key" layer (we gave them unique names).
layer_dict = dict([(layer.name, layer) for layer in model.layers[1:]])

def normalize(x):
    # utility function to normalize a tensor by its L2 norm
    return x / (K.sqrt(K.mean(K.square(x))) + K.epsilon())

kept_filters = []

for filter_index in range(anzahl_filter_auf_schicht):
    print('Processing filter %d' % filter_index)
    start_time = time.time()

    # we build a loss function that maximizes the activation
    # of the nth filter of the layer considered
    layer_output = layer_dict[name_der_schicht].output
    if K.image_data_format() == 'channels_first':
        loss = K.mean(layer_output[:, filter_index, :, :])
    else:
        loss = K.mean(layer_output[:, :, :, filter_index])

    # we compute the gradient of the input picture wrt this loss
    grads = K.gradients(loss, eingabe_img)[0]

    # normalization trick: we normalize the gradient
    grads = normalize(grads)

    # this function returns the loss and grads given the input picture
    iterate = K.function([eingabe_img], [loss, grads])

    # step size for gradient ascent
    step = 1.

    # we start from a gray image with some random noise
    if K.image_data_format() == 'channels_first':
        eingabe_img_data = np.random.random((1, 3, eingabe_width, eingabe_height))
    else:
        eingabe_img_data = np.random.random((1, eingabe_width, eingabe_height, 3))
    eingabe_img_data = (eingabe_img_data - 0.5) * anzahl_gradient_durchlauefe + 128

    # we run gradient ascent for 20 steps
    for i in range(anzahl_gradient_durchlauefe):
        loss_value, grads_value = iterate([eingabe_img_data])
        eingabe_img_data += grads_value * step

        print('Current loss value:', loss_value)
        # Auskommentiert: Es sollen auch Filter berücksichtigt werden, auf denen "nicht viel" zu sehen ist
		#if loss_value <= 0.:
            # some filters get stuck to 0, we can skip them
            #break

    # decode the resulting input image
    # Aus gleichem Grund auskommentiert
	#if loss_value > 0:
    img = deprocess_image(eingabe_img_data[0])
    kept_filters.append((img, loss_value))
    end_time = time.time()
    print('Filter %d processed in %ds' % (filter_index, end_time - start_time))

# the filters that have the highest loss are assumed to be better-looking.
# we will only keep the top 64 filters.
kept_filters.sort(key=lambda x: x[1], reverse=True)
kept_filters = kept_filters[:anzahl_ausgabe_bilder * anzahl_ausgabe_bilder]

# Größe des gesamten Bildes zusammenbauen
margin = 5
width = anzahl_ausgabe_bilder * eingabe_width + (anzahl_ausgabe_bilder - 1) * margin
height = anzahl_ausgabe_bilder * eingabe_height + (anzahl_ausgabe_bilder - 1) * margin
stitched_filters = np.zeros((width, height, 3))

# fill the picture with our saved filters
for i in range(anzahl_ausgabe_bilder):
    for j in range(anzahl_ausgabe_bilder):
        img, loss = kept_filters[i * anzahl_ausgabe_bilder + j]
        stitched_filters[(eingabe_width + margin) * i: (eingabe_width + margin) * i + eingabe_width,
                         (eingabe_height + margin) * j: (eingabe_height + margin) * j + eingabe_height, :] = img

# save the result to disk
imsave('stitched_filters_%dx%d.png' % (anzahl_ausgabe_bilder, anzahl_ausgabe_bilder), stitched_filters)