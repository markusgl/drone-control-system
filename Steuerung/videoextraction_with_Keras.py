import numpy as np
import cv2

from ClassifyImages_with_Keras import Classify

class Videoextractor:
    def __init__(self):
        #self.classifier = Classify('Trained99-0.01.hdf5')
        self.classifier = Classify('/Users/mgl/dev/tf_models/HD5/BinaryRopeDetection-17-0.00.hdf5')

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
        cap = cv2.VideoCapture('/Users/mgl/IT-Projekt/IT-Projekt_vids/output3.mpg')

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (20,230)
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2
        no_rope_counter = 0

        while(cap.isOpened()):
            ret, frame = cap.read()

            cv2.imwrite('bild.jpg',frame)
            frame = cv2.resize(frame, (0,0), fx=0.75,fy=0.75)
            #classification_text = self.direction_to_number(self.classifier.classifyAImage('bild.jpg'))
            #pos=self.classifier.classifyAImage(frame)
            pos = self.classifier.classifyAImage(frame)
            print(pos)
            if pos == 6:
                print("Top reached")
                break
            cv2.putText(frame, 'Klasse: ' + str(self.__argmax_to_direction(pos)), bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
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