from styx_msgs.msg import TrafficLight
from keras.models import load_model
import h5py
from keras import __version__ as keras_version
import cv2
import tensorflow as tf

IMG_COLS = 200
IMG_ROWS = 66

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
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        height, width, channels = img.shape
        crop_img = img[int(height/4):height-25, 0:width]
        scale_img = cv2.resize(crop_img, (IMG_COLS, IMG_ROWS))
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
                state = int(self.model.predict(image[None, :, :, :], batch_size=1))
        return state

