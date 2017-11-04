from styx_msgs.msg import TrafficLight
from keras.models import load_model
import h5py
from keras import __version__ as keras_version
import cv2
import tensorflow as tf
import numpy as np

IMG_COLS = 160
IMG_ROWS = 120

class TLClassifier(object):
    def __init__(self, model_path):
        #TODO load classifier
        global keras_version
        self.model = None
        # check that model Keras version is same as local Keras version
        f = h5py.File(model_path, mode='r')
        model_version = f.attrs.get('keras_version')
        keras_version = str(keras_version).encode('utf8')

        if model_version != keras_version:
            print('\033[91m' +
                  '[ERROR] You are using Keras version ' + keras_version +
                  ', but the model was built using ' + model_version +
                  '\033[00m')
        else:
            self.model = load_model(model_path)
            self.model._make_predict_function() 
            self.graph = tf.get_default_graph()

    def pre_processing(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        scale_img = cv2.resize(img, (IMG_COLS, IMG_ROWS))
        return scale_img

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        state = TrafficLight.UNKNOWN
        if self.model:
            image = self.pre_processing(image)
            with self.graph.as_default():
                result = self.model.predict(image[None, :, :, :], batch_size=1)
                state = np.argmax(result)
                if state == 3:
                    state = 4
        return state

