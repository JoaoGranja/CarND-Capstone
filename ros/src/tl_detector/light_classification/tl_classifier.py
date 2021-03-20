from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import os
import rospy
#from object_detection.utils import ops as utils_ops
#from object_detection.utils import label_map_util

class TLClassifier(object):
    def __init__(self):
        
        SSD_GRAPH_FILE = 'light_classification/model/frozen_inference_graph.pb'
        
        self.detection_graph = self.load_graph(SSD_GRAPH_FILE)
        
        self.confidence_cutoff = 0.5

        rospy.logwarn("TL Detector - Finish classifier initialization")
        #self.tensor_dict['detection_masks'] = self.detection_graph.get_tensor_by_name('detection_masks:0')

        ## The following processing is only for single image
        #detection_boxes = tf.squeeze(self.tensor_dict['detection_boxes'], [0])
        #detection_masks = tf.squeeze(self.tensor_dict['detection_masks'], [0])
        ## Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
        #real_num_detection = tf.cast(self.tensor_dict['num_detections'][0], tf.int32)
        #detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
        #detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
        #detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(detection_masks, detection_boxes, image.shape[0], image.shape[1])
        #detection_masks_reframed = tf.cast(tf.greater(detection_masks_reframed, 0.5), tf.uint8)
        # Follow the convention by adding back the batch dimension
        #self.tensor_dict['detection_masks'] = tf.expand_dims(detection_masks_reframed, 0)


    def filter_boxes(self, min_score, output_dict):
        """Return boxes with a confidence >= `min_score`"""
        n = len(output_dict['detection_classes'])
        if n > 1:
            idxs = []
            for i in range(n):
                if output_dict['detection_scores'][i] >= min_score:
                    idxs.append(i)

            for key, items in output_dict.items():
                #rospy.logwarn("TL Detector - key {}".format(key))
                output_dict[key] = items[idxs]

        return output_dict

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def run_inference_for_single_image(self, image):
        # Convert image to numpy
        #image_np = self.load_image_into_numpy_expanded(image)
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), axis=0)
        
        #rospy.logwarn("TL Detector - camera image {}".format(image_np))
        
        with tf.Session(graph=self.detection_graph) as sess:
            image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')    
            tensor_dict = {}
            tensor_dict['num_detections'] = self.detection_graph.get_tensor_by_name('num_detections:0')
            tensor_dict['detection_boxes'] = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            tensor_dict['detection_scores'] = self.detection_graph.get_tensor_by_name('detection_scores:0')
            tensor_dict['detection_classes'] = self.detection_graph.get_tensor_by_name('detection_classes:0')
            
            # Run inference
            output_dict = sess.run(tensor_dict, feed_dict={image_tensor: image_np})

            # all outputs are float32 numpy arrays, so convert types as appropriate
            output_dict['num_detections'] = int(output_dict['num_detections'][0])
            output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
            output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
            output_dict['detection_scores'] = output_dict['detection_scores'][0]
            #if 'detection_masks' in output_dict:
            #    output_dict['detection_masks'] = output_dict['detection_masks'][0]

            # Filter boxes with a confidence score less than `confidence_cutoff`
            #output_dict = self.filter_boxes(self.confidence_cutoff, output_dict)

            return output_dict


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        # Actual detection.
        output_dict = self.run_inference_for_single_image(image)
        
        traffic_light = TrafficLight.UNKNOWN
        if output_dict['num_detections'] > 0:
            rospy.logwarn("TL Detector - Detection dict {}".format(output_dict))
            if output_dict['detection_classes'][0].astype(np.uint8) == 1:
                traffic_light = TrafficLight.GREEN 
            elif output_dict['detection_classes'][0].astype(np.uint8) == 2:
                traffic_light = TrafficLight.RED
            elif output_dict['detection_classes'][0].astype(np.uint8) == 3:
                traffic_light = TrafficLight.YELLOW
            
        return traffic_light
