from styx_msgs.msg import TrafficLight

import timeit
import tensorflow as tf
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile('frozen_inference_graph.pb', 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name = '')

        self.graph = detection_graph 
        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        pred = 0
        highest_score = 0

        # Definite input and output Tensors for detection_graph
        image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.graph.get_tensor_by_name('num_detections:0')

        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        #start_time = timeit.default_timer()
        (boxes, scores, classes, num) = self.sess.run([detection_boxes, detection_scores, detection_classes, num_detections],feed_dict={image_tensor: image_np_expanded})
        for i in range(len(classes[0])):
            if scores[0][i] > highest_score:
                highest_score = scores[0][i]
                pred = classes[0][i]
        #rospy.logerr("Predict " +str(pred) + " Elasped: " + str(timeit.default_timer() - start_time))

        if pred == 1:
                return TrafficLight.RED
        elif pred == 2:
                return TrafficLight.YELLOW
        elif pred == 3:
                return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
