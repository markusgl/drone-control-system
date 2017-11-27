import cv2
import math
from Picture_Argumentation.ImageStitching.stitchit import ImageStitcher


def stitch_wall_images():
    images = []
    '''
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-30.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-31.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-32.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-33.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-34.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-35.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-36.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-37.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-38.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-39.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-40.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-41.jpg"))
    '''

    # bottom to top order
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-41.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-40.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-39.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-38.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-37.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-36.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-35.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-34.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-33.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-32.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-31.jpg"))
    images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-30.jpg"))

    stitcher = ImageStitcher()
    stitcher.stitch_images(images)

def stitch_panorama_example():
    images = []
    # Example Panorama
    #images.append(cv2.imread("F:\\Stitchpics\\panorama_vert1.png"))
    #images.append(cv2.imread("F:\\Stitchpics\\panorama_vert2.png"))
    images.append(cv2.imread("F:\\Stitchpics\\tmp2\\panorama2.png"))
    images.append(cv2.imread("F:\\Stitchpics\\tmp2\\panorama1.png"))
    #images.append(cv2.imread("F:\\Stitchpics\\tmp2\\panorama2.png"))

    #filelist = './images/Dachelwand-right-30.jpg', './images/Dachelwand-right-31.jpg', './images/Dachelwand-right-32.jpg', './images/Dachelwand-right-33.jpg'
    #images = np.array([np.array(Image.open(fname)) for fname in filelist])

    stitcher = ImageStitcher()
    stitcher.stitch_images(images)


def crop_images():
    ori_img = cv2.imread("F:\\Stitchpics\\panorama1.png")
    imgWidth = ori_img.shape[1]
    imgHeight = ori_img.shape[0]
    imageParts = 3
    partHeight = math.floor(imgHeight / imageParts)

    start = 0
    end = partHeight
    partCounter = 0
    while partCounter <= imageParts - 1:
        partCounter += 1
        crop_img = ori_img[start:end, 0:imgWidth]  # Crop from x, y, w, h -> 100, 200, 300, 400
        # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
        path = "F:\\Stitchpics\\tmp2\\panorama" + str(partCounter) + ".png"
        cv2.imwrite(path, crop_img)
        start = end - int(partHeight / 10)
        print(start)
        end = end + partHeight
        print(end)
        print(path)

def video_extraction():
    count = 0
    cap = cv2.VideoCapture("F:\\Videos_vong_Drohne\\Dachelwand1.mp4")
    while (cap.isOpened()):
        ret, frame = cap.read()

        count += 1
        if count % 20 == 0:
            rope_image = 'image' + str(count) + '.jpeg'
            print("Received an image: " + rope_image)

            cv2.imwrite("F:\\Videos_vong_Drohne\\frames\\" + rope_image, frame)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


#stitch_panorama_example()
#stitch_wall_images()
video_extraction()
