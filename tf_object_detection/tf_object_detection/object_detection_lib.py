#!/usr/bin/env python
# Object Detection with TensorFlow and TensorFlow object detection API
# We will be using a pre-trained model from TensorFlow detection model zoo
# See https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

import numpy as np
import tensorflow as tf
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

class ObjectDetection:

    def __init__(self, path, confidence): # path will be to the models/research/object_detection directory
        utils_ops.tf = tf.compat.v1
        tf.gfile = tf.io.gfile
               
        # Pre-trained model name
        MODEL_NAME = 'ssd_mobilenet_v2_320x320_coco17_tpu-8'
        MODEL_DIR = path + '/' + MODEL_NAME + '/saved_model'
        PATH_TO_LABELS = path + '/data/' + 'mscoco_label_map.pbtxt'

        # Load pre-trained model
        self.model = tf.saved_model.load(MODEL_DIR)
        # Load the label map. Label maps map indices to category names
        self.__category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)
        # Store the confidence level
        self.__confidence = confidence


    def run_inference_for_single_image(self, image):
        image = np.asarray(image)
        # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
        input_tensor = tf.convert_to_tensor(image)
        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis,...]

        # Run inference
        model_fn = self.model.signatures['serving_default']
        output_dict = model_fn(input_tensor)

        # All outputs are batches tensors.
        # Convert to numpy arrays, and take index [0] to remove the batch dimension.
        # We're only interested in the first num_detections.
        num_detections = int(output_dict.pop('num_detections'))
        output_dict = {key:value[0, :num_detections].numpy() 
                        for key,value in output_dict.items()}
        output_dict['num_detections'] = num_detections
        output_dict['detection_classes'] = output_dict['detection_classes'].astype(np.int64)
        # Handle models with masks:
        if 'detection_masks' in output_dict:
            # Reframe the the bbox mask to the image size.
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                    output_dict['detection_masks'], output_dict['detection_boxes'],
                    image.shape[0], image.shape[1])      
            detection_masks_reframed = tf.cast(detection_masks_reframed > 0.5,tf.uint8)
            output_dict['detection_masks_reframed'] = detection_masks_reframed.numpy()
        return output_dict


    # This class function will be called from outside to scan the supplied img.
    # if objects are detected it will adjust the supplied image by drawing boxes areound the objects
    # The img parameter is an OpenCV image
    def scan_for_objects(self, image_np):
        # Actual detection.
        output_dict = self.run_inference_for_single_image(image_np)

        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            self.__category_index,
            instance_masks=output_dict.get('detection_masks_reframed', None),
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=self.__confidence)

        # Return a list of object names detected
        # detected_list = []
        detected_list = ""
        total_detections = output_dict['num_detections']
        if total_detections > 0:
            for detection in range(0, total_detections):
                if output_dict['detection_scores'][detection] > self.__confidence:
                    category = output_dict['detection_classes'][detection]
                    # detected_list.insert(0,self.__category_index[category]['name'])
                    detected_list += "-" + self.__category_index[category]['name']
        return detected_list
