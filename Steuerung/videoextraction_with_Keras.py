import numpy as np
import cv2

from ClassifyImages_with_Keras import Classify

class Videoextractor:
    def __init__(self):
        self.classifier = Classify('Trained99-0.01.hdf5')
        #self.classifier = Classify('BinaryRopeDetection-06-0.00.hdf5')

    def __direction_to_number(self, arg):
        options = {1: "links",
                   2: "halblinks",
                   3: "mitte",
                   4: "halbrechts",
                   5: "rechts",
                   6: "top",
                   7: "kein Seil",
                   }
        return options.get(arg, "nothing")

    def createVideo(self):
        cap = cv2.VideoCapture('output2.mpg')

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (20,400)
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2

        while(cap.isOpened()):
            ret, frame = cap.read()

            cv2.imwrite('bild.jpg',frame)
            #classification_text = self.direction_to_number(self.classifier.classifyAImage('bild.jpg'))
            pos=self.classifier.classifyAImage(frame)
            cv2.putText(frame, 'Klasse: ' + str(self.__direction_to_number(pos)), bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
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