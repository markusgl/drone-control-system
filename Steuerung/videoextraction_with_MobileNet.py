import numpy as np
import cv2

from ClassifiyImages_with_MobileNet import Classify

class Videoextractor:
    def __init__(self):
        #self.classifier = Classify("../../Inception_10000_6600Images/output_labels.txt", "../../Inception_10000_6600Images/output_graph.pb", 'DecodeJpeg/contents:0', 'final_result:0')
        #self.classifier = Classify("../../Inception_5000_3Classes/output_labels.txt", "../../Inception_5000_3Classes/output_graph.pb", 'DecodeJpeg/contents:0', 'final_result:0')
        self.classifier = Classify("..\models\output_labels.txt", "..\models\output_graph.pb", 'input:0', 'final_result:0')

    # maps tensorflow classes to integer values
    def direction_to_number(self, arg):
        options = {1 : "links",
                   2 : "halblinks",
                   3 : "mitte",
                   4 : "halbrechts",
                   5 : "rechts",
                   6 : "top",
                   7 : "kein Seil",
                   }
        return options.get(arg, "nothing")


    def createVideo(self):
        cap = cv2.VideoCapture('output5.mpg')

        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (20,400)
        fontScale = 1
        fontColor = (255,255,255)
        lineType = 2

        while(cap.isOpened()):
            ret, frame = cap.read()

            cv2.imwrite('bild.jpg',frame)
            #classification_text = self.direction_to_number(self.classifier.classifyAImage('bild.jpg'))
            classification_text = str(self.classifier.classifyAImage('bild.jpg'))
            cv2.putText(frame, 'Klasse: ' + classification_text, bottomLeftCornerOfText, font, fontScale, fontColor, lineType)
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