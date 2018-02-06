import imutils
import cv2
import numpy as np
import os
import threading

class Stitcher(threading.Thread):

    def __init__(self, images):
        threading.Thread.__init__(self)
        # determine if we are using OpenCV v3.X
        self.isv3 = imutils.is_cv3()
        self.images = images

    def run(self):
        result = self.stitch_images(self.images)
        cv2.imshow("Result", imutils.resize(result, width=600))
        cv2.waitKey(0)

    def stitch_images(self, video):
        # bottom to top order
        frames = self._video_extraction(video)
        stitched_img = frames[1]
        if stitched_img is not None:
            print("Start stitching images - this may take some time...")
            for x in range(1, len(frames)):
                stitched_img = self.__stitch([imutils.resize(stitched_img, width=800), imutils.resize(images[x], width=800)])
        else:
            print("No images loaded.")

        return stitched_img

    def _video_extraction(self, video):
        count = 0
        cap = cv2.VideoCapture(video)
        frames = []
        while (cap.isOpened()):
            ret, frame = cap.read()

            count += 1
            if count % 20 == 0:
                rope_image = 'image' + str(count) + '.jpeg'
                print("Received an image: " + rope_image)

                frames.append(frame)

        return frames

    def __stitch(self, images, ratio=0.75, reprojThresh=4.0):
        (kpsA, featuresA) = self._detectAndDescribe(images[0])
        (kpsB, featuresB) = self._detectAndDescribe(images[1])
        M = self._matchKeypoints(kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh)

        if M is None:
            return None

        # apply a perspective warp to stitch the images
        (matches, H, status) = M

        # naive approach
        result = cv2.warpPerspective(images[0], H, (images[0].shape[1], images[0].shape[0] + int(images[1].shape[0]/4))) #width, height(sum)
        result[0:images[1].shape[0], 0:images[1].shape[1]] = images[1]

        ''' ******* mathematical approach - not implemented yet ********* '''
        '''
        # compute pairwise homographies
        homographies = []
        for i in range(0, len(images)+1):
            for j in range(i+1, len(images)):
                (kpsA, featuresA) = self._detectAndDescribe(images[i])
                (kpsB, featuresB) = self._detectAndDescribe(images[j])
                (matches, H, status) = self._matchKeypoints(kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh)
                #print("Homography I_" + str(i) + str(j) +" = " + str(H))
                homographies.append(H)

        # select one anchor image
        Identity = homographies[0]
        result = cv2.warpPerspective(images[0], homographies[0],(images[0].shape[1], images[0].shape[0] + images[1].shape[0]))
        for x in range(1, len(images)):
            result = cv2.warpPerspective(result, homographies[x], (images[0].shape[1]+int(images[1].shape[0]), images[0].shape[0] + images[1].shape[0]))

        #TODO warp images bottom up
        #print("img0 shape 0: " + str(images[0].shape[0])) #height
        #print("img0 shape 1: " + str(images[0].shape[1])) #width
        #print("img1 shape 0: " + str(images[1].shape[0])) #height
        #print("img1 shape 1: " + str(images[1].shape[1])) #width
        '''

        return result

    def _matchKeypoints(self, kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh):
        # compute the raw matches and initialize the list of actual matches
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []

        # loop over the raw matches
        for m in rawMatches:
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                matches.append((m[0].trainIdx, m[0].queryIdx))

        # computing a homography requires at least 4 matches
        if len(matches) > 4:
            ptsA = np.float32([kpsA[i] for (_, i) in matches])
            ptsB = np.float32([kpsB[i] for (i, _) in matches])

            # compute the homography between the two sets of points
            (H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC, reprojThresh)

            # return the matches along with the homograpy matrix
            # and status of each matched point
            return (matches, H, status)

        # otherwise, no homograpy could be computed
        print("Less than 4 matches found. No homography could be computed")
        return None

    def _detectAndDescribe(self, image):
        # convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # OpenCV 3.X
        if self.isv3:
            # detect and extract features from the image
            descriptor = cv2.xfeatures2d.SIFT_create()
            (kps, features) = descriptor.detectAndCompute(image, None)

        # OpenCV 2.4.X
        else:
            # detect keypoints in the image
            detector = cv2.FeatureDetector_create("SIFT")
            kps = detector.detect(gray)

            # extract features from the image
            extractor = cv2.DescriptorExtractor_create("SIFT")
            (kps, features) = extractor.compute(gray, kps)

        # convert the keypoints from KeyPoint objects to NumPy arrays
        kps = np.float32([kp.pt for kp in kps])

        return (kps, features)


    #TODO delete for production environment - only necessary for testing purposes
    def _drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
        """
        draws keypoint_matches in result
        """
        # initialize the output visualization image
        (hA, wA) = imageA.shape[:2]
        (hB, wB) = imageB.shape[:2]
        vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
        vis[0:hA, 0:wA] = imageA
        vis[0:hB, wA:] = imageB

        # loop over the matches
        for ((trainIdx, queryIdx), s) in zip(matches, status):
            # only process the match if the keypoint was successfully
            # matched
            if s == 1:
                # draw the match
                ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
                ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
                cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

        # return the visualization
        return vis

if __name__ == '__main__':

    images = [] # bottom to top order
    path = "F:\\Videos_vong_Drohne\\frames\\test1"
    for root, dirs, file_names in os.walk(path, topdown=False):
        for file_name in file_names:
            images.append(cv2.imread(os.path.join(root,file_name)))
            print(os.path.join(root,file_name))

    thread1 = Stitcher(images)
    thread1.start()
    print("Tue etwas anderes...")
    #stitched_img = newStitcher.stitch(images, showMatches = True) #this works well for two images
