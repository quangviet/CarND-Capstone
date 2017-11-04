from styx_msgs.msg import TrafficLight
from keras.models import load_model
import h5py
from keras import __version__ as keras_version

model = None

class TLClassifier(object):
    def __init__(self, model_path):
        #TODO load classifier
        global model
        global keras_version
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
            model = load_model(model_path)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        global model
        state = TrafficLight.UNKNOWN
        if model:
            state = int(model.predict(image[None, :, :, :], batch_size=1))
        return state

