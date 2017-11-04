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
    cv_image = cv2.imread('training_data/cv_image.png')
    state = light_classifier.get_classification(cv_image)
    print('Get state: ' + str(state))

