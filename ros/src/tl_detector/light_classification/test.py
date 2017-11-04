#!/usr/bin/env python
from tl_classifier import TLClassifier
import cv2

if __name__ == '__main__':
    print('==========')
    print('UNKNOWN: 4')
    print('GREEN:   2')
    print('YELLOW:  1')
    print('RED:     0')
    print('==========')
    light_classifier = TLClassifier('model.h5')

    cv_image = cv2.imread('training_data/cv_image_unknown.png')
    state = light_classifier.get_classification(cv_image)
    print('Get state 4: ' + str(state))
    cv_image = cv2.imread('training_data/cv_image_green.png')
    state = light_classifier.get_classification(cv_image)
    print('Get state 2: ' + str(state))
    cv_image = cv2.imread('training_data/cv_image_yellow.png')
    state = light_classifier.get_classification(cv_image)
    print('Get state 1: ' + str(state))
    cv_image = cv2.imread('training_data/cv_image_red.png')
    state = light_classifier.get_classification(cv_image)
    print('Get state 0: ' + str(state))

