#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow

import os
import sys
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String , Header
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from mapping_msgs.msg import BoundingBox, BoundingBoxes, ObjectCount, BoxesAndClouds
import message_filters as mf
# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from rosgraph_msgs.msg import Clock

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.4

######### Set model here ############
#MODEL_NAME =  'ssd_resnet101_v1_fpn_shared_box_predictor_oid_512x512_sync_2019_01_20'
MODEL_NAME = 'faster_rcnn_inception_resnet_v2_atrous_oid_v4_2018_12_12'
#MODEL_NAME = 'faster_rcnn_inception_resnet_v2_atrous_lowproposals_oid_2018_01_28'

# By default models are stored in data/models/
MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','models' , MODEL_NAME)
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'open_images.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 600

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.compat.v2.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
config = tf.compat.v1.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION
config.gpu_options.allow_growth = True

# Detection

class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("bounding_boxes", BoundingBoxes, queue_size=1)
        self.final_pub = rospy.Publisher("cloud_and_boxes", BoxesAndClouds, queue_size=1)
        self.bridge = CvBridge()
        
        #self.use_compressed_image =  rospy.get_param('compressed', 'False')
        self.use_compressed_image = True
        if (self.use_compressed_image == True):
            
            self.image_sub = mf.Subscriber('image',  CompressedImage)
        else:    
            self.image_sub = mf.Subscriber('image', Image)
        #self.cloud_sub = mf.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rect_points', Image)
        self.cloud_sub = mf.Subscriber('points', PointCloud2)
        print("Compressed: ", self.use_compressed_image)
        ts = mf.ApproximateTimeSynchronizer([self.image_sub, self.cloud_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_cb)
        #self.image_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2**24)
        #self.cloud_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.sess = tf.compat.v1.Session(graph=detection_graph,config=config)
        print(self.sess)

    def image_cb(self, data, cloud):
        #print(data)
        finalPubMsg = BoxesAndClouds()
        objArray = BoundingBoxes()
        
        try:
            if (self.use_compressed_image == True):
                np_arr = np.fromstring(data.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = np.asarray(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        (boxes, scores, classes, num_detections) = self.sess.run([boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})
        objects=vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2)

        objArray.bounding_boxes =[]
        objArray.header=data.header
        objArray.image_header=data.header
        object_count=1

        for i in range(len(objects)):
            #print("BOUNDING", objects[i][2])
            object_count+=1
            objArray.bounding_boxes.append(self.object_predict(objects[i],data.header,image_np,cv_image))

        self.object_pub.publish(objArray)
        finalPubMsg.bounding_boxes = objArray
        finalPubMsg.point_cloud = cloud
        self.final_pub.publish(finalPubMsg)

        img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        self.image_pub.publish(image_out)


    def object_predict(self,object_data, header, image_np,image):
        image_height,image_width,channels = image.shape
        obj=BoundingBox()
        obj.probability = object_data[1]
        #print("BOUNDING", object_data[2][2])
        
        center_x = int((object_data[2][1] + object_data[2][3])*image_height/2)
        center_y = int((object_data[2][0] + object_data[2][2])*image_width/2)
        size_y = int((object_data[2][2]-object_data[2][0])*image_height)
        size_x = int((object_data[2][3]-object_data[2][1] )*image_width)
        
        obj.ymin = int(object_data[2][0] *image_height)
        obj.xmin = int(object_data[2][1] *image_width)
        obj.ymax = int(object_data[2][2] *image_height)
        obj.xmax = int(object_data[2][3] *image_width)
        obj.id = object_data[0]
        obj.Class = object_data[3]
	print(obj.Class)
        
        return obj

def main(args):
    rospy.init_node('detector_node')
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)
