#!/usr/bin/env python
from tl_classifier import TLClassifier
import cv2

IMG_COLS = 200
IMG_ROWS = 66

def pre_processing(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    height, width, channels = img.shape
    crop_img = img[int(height/4):height-25, 0:width]
    scale_img = cv2.resize(crop_img, (IMG_COLS, IMG_ROWS))
    return scale_img

if __name__ == '__main__':
    print('==========')
    print('UNKNOWN: 4')
    print('GREEN:   2')
    print('YELLOW:  1')
    print('RED:     0')
    print('==========')
    light_classifier = TLClassifier('model.h5')
    cv_image = cv2.imread('training_data/cv_image.png')
    cv_image = pre_processing(cv_image)
    state = light_classifier.get_classification(cv_image)
    print('Get state: ' + str(state))

