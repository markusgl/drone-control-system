import cv2
from Picture_Augmentation.ImageStitching.stitchit import ImageStitcher

# load images
images = []
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
#filelist = './images/Dachelwand-right-30.jpg', './images/Dachelwand-right-31.jpg', './images/Dachelwand-right-32.jpg', './images/Dachelwand-right-33.jpg'
#images = np.array([np.array(Image.open(fname)) for fname in filelist])

stitcher = ImageStitcher()
stitcher.stitch_images(images)
