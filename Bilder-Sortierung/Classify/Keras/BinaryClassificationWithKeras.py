from keras.preprocessing.image import ImageDataGenerator
from keras import callbacks
from Binary_Classification_Model import BinaryClassificationModel


#dimensions of our images.
img_width, img_height = 159, 160

train_data_dir = 'D:/tmp\Binär/train_mit_weniger'
validation_data_dir = 'D:/tmp\Binär/validate - Kopie'

# used to rescale the pixel values from [0, 255] to [0, 1] interval
datagen = ImageDataGenerator(rescale=1./255,
                             rotation_range=25,
                             horizontal_flip=True,
                             vertical_flip=True
                             )

# automagically retrieve images and their classes for train and validation sets
train_generator = datagen.flow_from_directory(
        train_data_dir,
        target_size=(img_width, img_height),
        batch_size=32,
        class_mode='binary')

validation_generator = datagen.flow_from_directory(
        validation_data_dir,
        target_size=(img_width, img_height),
        batch_size=32,
        class_mode='binary')

modelCreater= BinaryClassificationModel()
model= modelCreater.createModel(img_width,img_height)

model.compile(loss='binary_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

model.summary()

nb_epoch = 25
nb_train_samples = 10347
nb_validation_samples = 2079

model_path='../../../models/test.h5'

#Save the model after each epoch if the validation loss improved.
save_best = callbacks.ModelCheckpoint(model_path, monitor='val_acc', verbose=1,
                                     save_best_only=True, mode='auto')

#stop training if the validation loss doesn't improve for 5 consecutive epochs.
early_stop = callbacks.EarlyStopping(monitor='val_loss', min_delta=0, patience=5,
                                     verbose=0, mode='auto')

callbacks_list = [save_best, early_stop]

model.fit_generator(
        train_generator,
        samples_per_epoch=nb_train_samples,
        nb_epoch=nb_epoch,
        validation_data=validation_generator,
        nb_val_samples=nb_validation_samples,
        callbacks=callbacks_list)

model.save('../../../models/test-End.h5')

