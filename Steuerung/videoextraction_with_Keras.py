import numpy as np
import cv2
import time
from classify_images import Classify

class Videoextractor:
    def __init__(self):
        #self.classifier = Classify('Trained99-0.01.hdf5')
        self.classifier = Classify('F:/IT-Projekt/tf_models/HD5/BinaryRopeDetection-06-0.00.hdf5')

    def __argmax_to_direction(self, arg):
        options = {0: "links",
                   1: "halblinks",
                   2: "mitte",
                   3: "halbrechts",
                   4: "rechts",
                   -1: "kein Seil"
                   }
        return options.get(arg, "nothing")

    def createVideo(self):
        cap = cv2.VideoCapture('F:/IT-Projekt/Videos_vong_Drohne/output.mpg')

        font = cv2.FONT_HERSHEY_SIMPLEX
        middleLeftText = (20,280)
        bottomLeftCornerOfText = (20,310)
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2

        while(cap.isOpened()):
            ret, frame = cap.read()

            cv2.imwrite('bild.jpg',frame)
            frame = cv2.resize(frame, (0,0), fx=0.75,fy=0.75)
            #classification_text = self.direction_to_number(self.classifier.classifyAImage('bild.jpg'))
            #pos=self.classifier.classifyAImage(frame)
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
            cv2.putText(frame, 'Array: ' + str(self.classifier.prediction_array).replace('\n', ''), bottomLeftCornerOfText, font, 0.6, fontColor, 1)
            cv2.putText(frame, 'No rope counter: ' + str(self.classifier.no_rope_counter).replace('\n', ''), (20,335), font, 0.6, fontColor, 1)
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
