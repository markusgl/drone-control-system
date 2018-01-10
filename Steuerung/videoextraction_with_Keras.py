#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import time
from classify_images import Classify

class Videoextractor:
    def __init__(self):
        #self.classifier = Classify('Trained99-0.01.hdf5')
        self.classifier = Classify('../models/Selbstgestelt-E4-.hdf5')

    def __argmax_to_direction(self, arg):
        options = {0: "links",
                   1: "halblinks",
                   2: "mitte",
                   3: "halbrechts",
                   4: "rechts",
                   -1: "kein Seil"
                   }
        return options.get(arg, "nothing")


    def show_probability(self, frame, prediction_array):
        pos=24
        for probability in prediction_array:
            cv2.putText(frame, str(probability).replace('\n', ''),
                        (pos, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,  (0,0,255), 1)
            pos+=128
        return frame

    def createVideo(self):
        cap = cv2.VideoCapture('output8.avi')

        font = cv2.FONT_HERSHEY_SIMPLEX
        middleLeftText = (20,300)
        bottomLeftCornerOfText = (20,335)
        fontScale = 1
        fontColor = (0,0,255)
        lineType = 2

        while(cap.isOpened()):
            ret, frame = cap.read()

            #cv2.imwrite('bild.jpg',frame)
            frame = cv2.resize(frame, (0,0), fx=0.75,fy=0.75)

            start=time.time()
            pos = self.classifier.classify_image(frame)
            print(pos)
            end = time.time()-start
            print("time for classifying: " + str(end))
            if pos == 5:
                print("Top reached")
                break
            cv2.putText(frame, 'Klasse: ' + str(self.__argmax_to_direction(pos)), middleLeftText, font, fontScale, fontColor, lineType)
            np.set_printoptions(precision=2, suppress=True)
            #cv2.putText(frame, 'Array: ' + str(self.classifier.prediction_array).replace('\n', ''), bottomLeftCornerOfText, font, 0.6, fontColor, 1)
            frame= self.show_probability(frame, self.classifier.prediction_array)
            cv2.putText(frame, 'No rope counter: ' + str(self.classifier.no_rope_counter).replace('\n', ''), bottomLeftCornerOfText, font, 0.6, fontColor, 1)
            #print("Received position from classifer: " + str(pos))
            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

def main():
    extractor = Videoextractor()
    extractor.createVideo()

if __name__ == "__main__":
    main()
