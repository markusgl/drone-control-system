import imutils
import cv2
import numpy as np

class NewStitcher:
    def __init__(self):
        # determine if we are using OpenCV v3.X
        self.isv3 = imutils.is_cv3()

    def stitch(self, images, ratio=0.75, reprojThresh=4.0, showMatches=False):
        # unpack the images, then detect keypoints and extract
        # local invariant descriptors from them
        #(imageB, imageA) = images
        #(kpsA, featuresA) = self._detectAndDescribe(imageA)
        #(kpsB, featuresB) = self._detectAndDescribe(imageB)

        # match features between the two images
        #M = self._matchKeypoints(kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh)

        #TODO compute pairwise homographies
        homographies = []
        #print(len(images))
        for i in range(0, len(images)):
            for j in range(i+1, len(images)):
                (kpsA, featuresA) = self._detectAndDescribe(images[i])
                (kpsB, featuresB) = self._detectAndDescribe(images[j])
                (matches, H, status) = self._matchKeypoints(kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh)
                print("Homography I_" + str(i) + str(j) +" = " + str(H))
                homographies.append(H)

        # select one anchor image
        Identity = homographies[0]

        #TODO warp images bottom up
        print("img0 shape 0: " + str(images[0].shape[0])) #height
        print("img0 shape 1: " + str(images[0].shape[1])) #width
        print("img1 shape 0: " + str(images[1].shape[0])) #height
        print("img1 shape 1: " + str(images[1].shape[1])) #width
        result = cv2.warpPerspective(images[0], homographies[0], (images[0].shape[1], images[0].shape[0] + images[1].shape[0]))
        #print(len(result[0:images[1].shape[1]]))
        result[0:images[1].shape[0], 0:images[1].shape[1]] = images[1]  # not sure whats going on here
        #result[0:images[1].shape[0], 0:images[1].shape[1]] = images[1] # not sure whats going on here
        # result[0:imageB.shape[0], 0:imageB.shape[1]] = imageB

        cv2.imshow("Result", cv2.resize(result, (800,600), interpolation = cv2.INTER_LINEAR))
        cv2.waitKey(0)


    def _matchKeypoints(self, kpsA, kpsB, featuresA, featuresB, ratio, reprojThresh):
        # compute the raw matches and initialize the list of actual
        # matches
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []

        # loop over the raw matches
        for m in rawMatches:
            # ensure the distance is within a certain ratio of each
            # other (i.e. Lowe's ratio test)
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                matches.append((m[0].trainIdx, m[0].queryIdx))

        # computing a homography requires at least 4 matches
        if len(matches) > 4:
            # construct the two sets of points
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

        # check to see if we are using OpenCV 3.X
        if self.isv3:
            # detect and extract features from the image
            descriptor = cv2.xfeatures2d.SIFT_create()
            (kps, features) = descriptor.detectAndCompute(image, None)

        # otherwise, we are using OpenCV 2.4.X
        else:
            # detect keypoints in the image
            detector = cv2.FeatureDetector_create("SIFT")
            kps = detector.detect(gray)

            # extract features from the image
            extractor = cv2.DescriptorExtractor_create("SIFT")
            (kps, features) = extractor.compute(gray, kps)

        # convert the keypoints from KeyPoint objects to NumPy
        # arrays
        kps = np.float32([kp.pt for kp in kps])

        # return a tuple of keypoints and features
        return (kps, features)


    #TODO delete later - only necessary for testing purposes
    def _drawMatches(self, imageA, imageB, kpsA, kpsB, matches, status):
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
    newStitcher = NewStitcher()
    images = []
    images.append(cv2.imread("F:\\Stitchpics\\panorama_vert1.png"))
    images.append(cv2.imread("F:\\Stitchpics\\panorama_vert2.png"))
    #images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-30.jpg"))
    #images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-31.jpg"))
    #images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-32.jpg"))
    #images.append(cv2.imread("F:\\Stitchpics\\Dachelwand-right-33.jpg"))
    newStitcher.stitch(images, showMatches = True)
