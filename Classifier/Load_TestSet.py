import cv2
import os
import numpy as np
from sklearn.model_selection import train_test_split

class DataLoader:

    def __init__(self):
        self.images= []
        self.labels=[]

    # Load the data
    def load_data(self, data_dir, img_Size_X, img_size_Y):
        directories = [d for d in os.listdir(data_dir)
                   if os.path.isdir(os.path.join(data_dir, d))]


        category = 0
        for d in directories:
            file_names = []
            label_dir = os.path.join(data_dir, d)
            for path, subdirs, files in os.walk(label_dir):
                for name in files:
                    if name.endswith(".jpg") or name.endswith(".png"):
                        os.path.join(path, name)
                        file_names.append(os.path.join(path, name))

            for f in file_names:
                img = cv2.imread(f)
                imresize = cv2.resize(img, (img_Size_X, img_size_Y))
                self.images.append(imresize)
                self.labels.append(category)

            category += 1
        return self.images, self.labels

    def printImageAndLabelArray(self):
        print(images, labels)

    def cross_validate(self,ImgArr, LabelArray, testSize=0.2):
        X_train, X_test, y_train, y_test = train_test_split(ImgArr , LabelArray, test_size=testSize, random_state=0)
        return X_train, X_test, y_train, y_test

    def normalizeData(self, ImagArray,LabelArray):
        imageArray= np.array(ImagArray).astype('float32')
        imageArray= imageArray / 255
        labelArray= np.array(LabelArray)
        return imageArray, labelArray

    def shuffel_data(self, img_array, label_array):
        from sklearn.utils import shuffle
        return shuffle(img_array, label_array, random_state=4)



if __name__ == "__main__":
    data_dir = "D:\\tmp\\128-128\\train"
    testset= DataLoader()
    images, labels = testset.load_data(data_dir, 128,128)
    testset.normalizeData(images, labels)
