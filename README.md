
Object-Oriented Semantic Mapping
This repository contains the accompanying code for the paper Online Object-Oriented Semantic Mapping and Map Updating by N. Dengler and T. Zaenker and F. Verdoja and M. Bennewitz accepted at EMCR, 2021. you can find the paper at https://arxiv.org/pdf/2011.06895.pdf

# How to use

## Get the Neural Net:
1. download pretrained neural net `"faster_rcnn_inception_resnet_v2_atrous_oidv4"` from `https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md`
2. place it in `tensorflow_object_detector/data/models/`
3. change camera and pointcloud topic in `tensorflow_object_detector/launch/object_detection.launch`
4. To use it tensorflow has to be installed follow step 3 of installation readme.md in the `tensorflow_object_detector` folder

## Prerequisites for mapping:
1. the map of the environment has to be loaded
2. amcl or other localizer has to run

## important commands:
1. build the repo in release mode: `catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build --profile release`
1. run tensorflow:   
    `source ~/tensorflow/bin/activate && roslaunch tensorflow_object_detector object_detect.launch`
2. run mapping:     
    `rosrun semmapping mapping _camera_info:=YOUR/CAMERA/INFO/TOPIC`


## setable Params:
Some Params ar configureable with a rqt configure server. You can change the default value in /cfg/Params.cfg and change the value with `rosrun rqt_reconfigure rqt_reconfigure`.
important Params are:
1. queue_size: size of Queue which is used for the object likelihood. If the que limit is reaced the oldest value will be poped for a new one
2. certainty_thresh: If the obj certainty is under the threshold, the obj will be removed
3. queue_thresh: Only remove obj if Queue size > queue_thresh
4. view_min: obj cen which are < view_min will not be removed
5. view_max: obj cen which are > view_max will not be removed
 
## Citation:
Please cite this work as:

@InProceedings{Dengler21ecmr,
  author =	 {N. Dengler and T. Zaenker and F. Verdoja and M. Bennewitz},
  title =	 {Online Object-Oriented Semantic Mapping and Map Updating},
  booktitle =	 {Proc.~of the European Conference on Mobile Robots (ECMR)},
  year =         2021
}

