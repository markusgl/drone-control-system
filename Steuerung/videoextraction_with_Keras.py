import numpy as np
import cv2

from ClassifyImages_with_Keras import Classify

class Videoextractor:
    def __init__(self):
        self.classifier = Classify('../models/TestModel_input159-160.h5')

    def createVideo(self):
        cap = cv2.VideoCapture('D:/ropeVids/output7.mpg')

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (20,400)
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2

        while(cap.isOpened()):
            ret, frame = cap.read()

            cv2.imwrite('bild.jpg',frame)
            #classification_text = self.direction_to_number(self.classifier.classifyAImage('bild.jpg'))
            pos=self.classifier.classifyAImage('bild.jpg')
            cv2.putText(frame, 'Klasse: ' + str(pos)+"%", bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
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