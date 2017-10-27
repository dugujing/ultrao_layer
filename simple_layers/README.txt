1. 修改turtlebot_navigation中的param/costmap_common_params.yaml
simple_layer:
  enabled:              true
  max_obstacle_height:  0.6
  origin_z:             0.0
  combination_method:   1
  track_unknown_space:  true    #true needed for disabling global path planning through unknown space
  observation_sources:  ultrasound #infrared
  ultrasound:
    data_type: ultrasound
    topic: ultrasound
    marking: true
    clearing: true
    min_obstacle_height: 0.10
    max_obstacle_height: 0.25
#  infrared:
#    data_type: infrared
#    topic: infrared
#    marking: true
#    clearing: true
#    min_obstacle_height: 0.10
#    max_obstacle_height: 0.25


2. 修改turtlebot_navigation中的param/local_costmap_params.yaml
    - {name: simple_layer,     type: "simple_layer_namespace::SimpleLayer"}


3. source 当前工作空间
  $ rospack plugins --attrib=plugin costmap_2d

4. $ roslaunch simple_layers simple_layer.launch
  launch 修改launch文件，进行tf坐标变换
