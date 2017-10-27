/*
 * Copyright 2017 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "simple_layers/simple_layer.h"
#include <pluginlib/class_list_macros.h> 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;  //致命障碍
using costmap_2d::NO_INFORMATION; //无信息 
using costmap_2d::FREE_SPACE; //空闲 

namespace simple_layer_namespace
{
  SimpleLayer::SimpleLayer() 
  {
    costmap_ = NULL;
  }
  
  SimpleLayer::~SimpleLayer() 
  {
    if (dsrv_)
      delete dsrv_;
  }
  
  void SimpleLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_), g_nh;
    //test cloud2
    cloud2pub_t_ = nh.advertise<sensor_msgs::PointCloud2>("test_cloud2",10);
    
    rolling_window_ = layered_costmap_->isRolling();
    
    std::string global_frame_ = layered_costmap_->getGlobalFrameID();
    std::cout << "global_frame-------" << global_frame_<< std::endl;
    
    //是否需要将未知空间在全局路径规划中禁用
    bool track_unknown_space;
    nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
    if (track_unknown_space)
      default_value_ = NO_INFORMATION;
    else
      default_value_ = FREE_SPACE;
    
    //重置大小
    CostmapLayer::matchSize(); 
    current_ = true;
    
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);
    
    //获取超声波和红外传感器的名称
    std::string topics_string;
    nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());
    
    ros::NodeHandle prefix_nh;
    const std::string tf_prefix = tf::getPrefixParam(prefix_nh);
    
    //我们可以使用stringstream将获取到的参数按照空格分割
    std::stringstream ss(topics_string);
    std::string source;
    while(ss >> source)
    {
      //传感器话题,数据类型
      std::string topic, data_type;
      double observation_keep_time, expected_update_rate;
      double min_obstacle_height, max_obstacle_height;
      bool clearing, marking;
      
      ros::NodeHandle source_node(nh, source);
      source_node.param("topic", topic, source);
      source_node.param("data_type", data_type, std::string(""));
      source_node.param("observation_persistence", observation_keep_time, 0.0);
      source_node.param("expected_update_rate", expected_update_rate, 0.0);
      source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
      source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
      source_node.param("clearing", clearing, false);
      source_node.param("marking", marking, true);

      ROS_INFO("simple layer param:-------topic: %s, data_type: %s", topic.c_str(), data_type.c_str());
      
      if (!(data_type == "ultrasound" || data_type == "infrared"))
      {
	ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
	throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
      }
      
      std::string raytrace_range_param_name;
      std::string obstacle_range_param_name;
      
      // 得到传感器的障碍范围
      double obstacle_range = 2.5;
      if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
      {
	source_node.getParam(obstacle_range_param_name, obstacle_range);
      }
      
      //获取传感器的射线追踪范围
      double raytrace_range = 3.0;
      if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
      {
	source_node.getParam(raytrace_range_param_name, raytrace_range);
      }
      
      //ROS_INFO("Creating an observation buffer for source: %s, topic: %s, frame_id: %s", source.c_str(), topic.c_str(),sensor_frame.c_str());
      
      // 创建观察缓冲区,设置sensor_frame为空，通过传感器数据获取
      observation_buffers_.push_back(boost::shared_ptr < ObservationBuffer> (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
												   "", transform_tolerance)) );
      
      //将obversetion_buffers_的最后一个元素的引用添加到标记缓冲区
      if (marking)
	marking_buffers_.push_back(observation_buffers_.back());
      
      // 检查我们是否将这个数据缓冲区添加到清除观察缓冲区
      if (clearing)
	clearing_buffers_.push_back(observation_buffers_.back());
      
      //ROS_INFO("Created an simple buffer for source %s, topic %s, global frame: %s, ""expected update rate: %.2f, observation persistence: %.2f"source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);
      
      // 为消息主题创建回调
      if (data_type == "ultrasound")
      {
	boost::shared_ptr < message_filters::Subscriber< simple_layers::UltrasoundData> > sub(new message_filters::Subscriber<simple_layers::UltrasoundData>(g_nh, topic, 50));
	boost::shared_ptr < tf::MessageFilter<simple_layers::UltrasoundData> > filter(new tf::MessageFilter<simple_layers::UltrasoundData>(*sub, *tf_, global_frame_, 50));
	//在回调函数中将消息中的超声波数据存储在observation_buffers对应的观察缓冲区中,主要完成超声波数据转换成点云数据[包括坐标变换]
	filter->registerCallback(boost::bind(&SimpleLayer::ultrasoundCB, this, _1, observation_buffers_.back()));
	
	observation_subscribers_.push_back(sub);
	observation_notifiers_.push_back(filter);
	ROS_INFO("created the ultrasound");
	//observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
      }
      else if (data_type == "infrared")
      {
	boost::shared_ptr < message_filters::Subscriber<simple_layers::InfraredData> > sub(new message_filters::Subscriber<simple_layers::InfraredData>(g_nh, topic, 50));
	boost::shared_ptr < tf::MessageFilter<simple_layers::InfraredData> >  filter(new tf::MessageFilter<simple_layers::InfraredData>(*sub, *tf_, global_frame_, 50));
	filter->registerCallback(boost::bind(&SimpleLayer::InfraredCB, this, _1, observation_buffers_.back()));
	
	observation_subscribers_.push_back(sub);
	observation_notifiers_.push_back(filter);
	ROS_INFO("created the infrared");
      }
      
    } 
    
    std::cout << "marking_buffers_ size: " << marking_buffers_.size() << "clearing_buffers_ size: " << clearing_buffers_.size() << std::endl;
    
    dsrv_ = new dynamic_reconfigure::Server<simple_layer_namespace::SimpleLayerConfig>(nh);
    dynamic_reconfigure::Server<simple_layer_namespace::SimpleLayerConfig>::CallbackType cb = boost::bind(&SimpleLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }
  
  void SimpleLayer::ultrasoundCB ( const simple_layers::UltrasoundDataConstPtr& message, const boost::shared_ptr< ObservationBuffer >& buffer )
  {
    
    geometry_msgs::Point32 point;
    
    geometry_msgs::PointStamped point_i;
    geometry_msgs::PointStamped point_o;
    
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 cloud2;
    
    cloud.header.frame_id = message->header.frame_id;
    cloud.header.stamp = message->header.stamp;
    
    for(int i = 0; i < message->value.size(); i++)
    {
      //只能将其转换成传感器中心对应的坐标
      try
      {
	point_i.header.frame_id = message->frame_id[i];
	point_i.header.stamp = message->header.stamp;
	point_i.point.x = message->value[i].x;
	point_i.point.y = message->value[i].y;
	point_i.point.z = message->value[i].z;
	//调试原始点没有问题
	std::cout << point_i.header.frame_id << "---"
		<< "src ul:----" << point_i.point.x << "," << point_i.point.y << "," << point_i.point.z << ","<< std::endl;
	
	tf_->waitForTransform(message->header.frame_id,message->frame_id[i], ros::Time::now(), ros::Duration(0.5));
	tf_->transformPoint(message->header.frame_id, point_i, point_o);
	
	//tf_->waitForTransform("odom",message->frame_id[i], ros::Time::now(), ros::Duration(0.5));
	//tf_->transformPoint("odom", point_i, point_o);
	//ROS_INFO("ultrasound---target: %s, src: %s, header: %s", message->header.frame_id.c_str(), message->frame_id[i].c_str(),point_i.header.frame_id.c_str());
	
	point.x = point_o.point.x;
	point.y = point_o.point.y;
	point.z = point_o.point.z;
	//std::cout << "tf ul_point:---- x " << point.x << " y " << point.y << " z " << point.z << std::endl;
	
	//如何将传感器数据转换成点云数据，将其添加到数据集中
	cloud.points.push_back(point);
      }
      catch (tf::TransformException &ex)
      {
	ROS_WARN("simple ultrasound transform %s: ",ex.what());
      }
    }
    
    if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2))
    {
      ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
      return;
    }
    //test
    cloud2pub_t_.publish(cloud2);
    
    // buffer the point cloud
    buffer->lock();
    //插入一组点云并将其转换成global坐标,对改组点云完成高度范围的过滤,并对每一组点云的原点的global添加到buffer中,将障碍范围和管线追踪范围写入
    buffer->bufferCloud(cloud2);
    buffer->unlock();
    
  }
  
  
  void SimpleLayer::InfraredCB ( const simple_layers::InfraredDataConstPtr& message, const boost::shared_ptr< ObservationBuffer >& buffer )
  {
    geometry_msgs::Point32 point;
    
    geometry_msgs::PointStamped point_i;
    geometry_msgs::PointStamped point_o;
    
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 cloud2;
    
    
    cloud.header.frame_id = message->header.frame_id;
    cloud.header.stamp = message->header.stamp;
    for(int i = 0; i < message->value.size(); i++)
    {
   
      point_i.header.frame_id = message->frame_id[i];
      point_i.header.stamp = message->header.stamp;
      point_i.point.x = message->value[i].x;
      point_i.point.y = message->value[i].y;
      point_i.point.z = message->value[i].z;
      
      try
      {
	tf_->transformPoint(message->header.frame_id, point_i, point_o);
	ROS_ERROR("infrared---target: %s, src:%s", message->header.frame_id.c_str(), message->frame_id[i].c_str());
	point.x = point_o.point.x;
	point.y = point_o.point.y;
	point.z = point_o.point.z;
	
	//如何将传感器数据转换成点云数据，将其添加到数据集中
	cloud.points.push_back(point);
      }
      catch (tf::TransformException &ex)
      {
	ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
		 message->header.frame_id.c_str(), ex.what());
      }
    }
    
    if(!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2))
    {
      ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
      return;
    }  
    // buffer the point cloud
    buffer->lock();
    //插入一组点云并将其转换成global坐标,对改组点云完成高度范围的过滤,并对每一组点云的原点的global添加到buffer中,将障碍范围和管线追踪范围写入
    buffer->bufferCloud(cloud2);
    buffer->unlock();
  }
  
  
  bool SimpleLayer::getClearingObservations ( std::vector< costmap_2d::Observation >& clearing_observations )
  {
    bool current = true;
    
    // get the clearing observations
    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
    {
      clearing_buffers_[i]->lock();
      clearing_buffers_[i]->getObservations(clearing_observations);
      current = clearing_buffers_[i]->isCurrent();
      clearing_buffers_[i]->unlock();
    }
    
    return current;
  }
  
  bool SimpleLayer::getMarkingObservations ( std::vector< costmap_2d::Observation >& marking_observations )
  {
    bool current = true;
    for(unsigned int i = 0; i < marking_buffers_.size(); ++i)
    { 
      marking_buffers_[i]->lock(); 
      //将所有观察的副本推送到传入的向量的末尾
      marking_buffers_[i]->getObservations(marking_observations);
      current = marking_buffers_[i]->isCurrent();
      marking_buffers_[i]->unlock();
    }
    return current;
  }
  
  
  void SimpleLayer::updateRaytraceBounds ( double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y, double* max_x, double* max_y )
  {
    double dx = wx-ox, dy = wy-oy;
    double full_distance = hypot(dx, dy);
    double scale = std::min(1.0, range / full_distance);
    double ex = ox + dx * scale, ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
  }
  
 
  //射线追踪，确定所容纳的最大边界
  void SimpleLayer::raytraceFreespace ( const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y, double* max_x, double* max_y )
  { 
    //std::cout << "clearing_observation size -------- : " << clearing_observation.cloud_->points.size() << std::endl; 
    //不同时间的机器人原点已经被写入观察值中
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    
    pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);
    
    //将机器人在world中的坐标转换成/map坐标
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0))
    {
      ROS_WARN_THROTTLE(
	1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
			ox, oy);
      return;
    }
    
    // 我们可以预先计算出内循环之外的地图的端点，我们稍后需要这些
    double origin_x = origin_x_, origin_y = origin_y_;
    
    //map地图，计算出当前地图的大小
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;
    
    
    //将机器人原点坐标和边框四点坐标比较，扩大边框
    touch(ox, oy, min_x, min_y, max_x, max_y);
    
    //对于云中的每个点，我们想要从原点跟踪一条线，并沿着它清除障碍物
    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      double wx = cloud.points[i].x;
      double wy = cloud.points[i].y;
      
      //当前障碍物和同一时刻机器人坐标的位置关系
      double a = wx - ox;
      double b = wy - oy;
      
      //射线追踪的最小值是原点
      if (wx < origin_x)
      {
	double t = (origin_x - ox) / a;
	wx = origin_x;
	wy = oy + b * t;
      }
      if (wy < origin_y)
      {
	double t = (origin_y - oy) / b;
	wx = ox + a * t;
	wy = origin_y;
      }
      
      // 射线追踪的最小值是地图的结尾
      if (wx > map_end_x)
      {
	double t = (map_end_x - ox) / a;
	wx = map_end_x - .001;
	wy = oy + b * t;
      }
      if (wy > map_end_y)
      {
	double t = (map_end_y - oy) / b;
	wx = ox + a * t;
	wy = map_end_y - .001;
      }
      // 现在，矢量被正确缩放...我们将得到其端点的地图坐标
      
      unsigned int x1, y1;
      //将wx和wy装换成map坐标
      if (!worldToMap(wx, wy, x1, y1))
	continue;
      
      //将光线追踪范围转换成单元格范围
      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
      MarkCell marker(costmap_, FREE_SPACE);
      
      // 最后，我们可以执行我们的追踪，以清除沿线的障碍
      raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
      
      updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
  }
  
  
 //robot_x robot_y robot_yaw是机器人的坐标和旋转，通过上层应用传值操作 
  void SimpleLayer::updateBounds ( double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y )
  {	
    
    if (rolling_window_)
    {
      double new_robotx = robot_x - getSizeInMetersX() / 2;
      double new_roboty = robot_y - getSizeInMetersY() / 2;
      //更新原点，获取以米为单位的尺寸
      updateOrigin(new_robotx, new_roboty);
    }
    if (!enabled_)
      return;
    
    
    //将本层地图地图边界更新到master中,没有addExtraBounds调用就不用调用
    useExtraBounds(min_x, min_y, max_x, max_y);
    
    
    bool current = true;
    std::vector<costmap_2d::Observation> marking_observations, clearing_observations;
    
    // 获取标记观察值,只获取最新的
    current = current && getMarkingObservations(marking_observations);
    //std::cout << "marking_observations size: " << marking_observations.size() << std::endl;
    
    // 获取清空观察值
    current = current && getClearingObservations(clearing_observations);
    //std::cout << "clearing_observations size: " << clearing_observations.size()<< std::endl;
    
    // update the global current status
    current_ = current;
    
    // 光线追踪可用空间,求解bound
    for (unsigned int i = 0; i < clearing_observations.size(); ++i)
    {
      raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    }
    
    
    // 将新的障碍物置于优先级队列中，每个优先级队列以优先级为零开始
    for (std::vector<costmap_2d::Observation>::const_iterator it = marking_observations.begin(); it != marking_observations.end(); ++it)
    {
      const costmap_2d::Observation& obs = *it;
      
      const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);
      std::cout << "cloud pointw size: " << cloud.points.size() << std::endl;
      
      // 计算点云中的点距离地盘中心的距离,通过比较点云距离中心点的平方
      //在外部指定的范围
      double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;
      for (unsigned int i = 0; i < cloud.points.size(); ++i)
      {
	double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;
	
	double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
	+ (pz - obs.origin_.z) * (pz - obs.origin_.z);
	
	//如果这点太远了，我们不会考虑的
	if (sq_dist >= sq_obstacle_range)
	{
	  ROS_DEBUG("The point is too far away");
	  continue;
	}
	
	// 将选中的点转换成map框架下
	unsigned int mx, my;
	if (!worldToMap(px, py, mx, my))
	{
	  ROS_DEBUG("Computing map coords failed");
	  continue;
	}
	
	//通过二维索引，转换为一维索引
	unsigned int index = getIndex(mx, my);
	
	//将障碍写入地图中
	costmap_[index] = LETHAL_OBSTACLE;
	
	//通过buffer中的点更新边界的宽度和高度
	touch(px, py, min_x, min_y, max_x, max_y);
      }
    }
    
    
    //获取到机器人的边界
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }

  }
  
  void SimpleLayer::updateCosts ( costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j )
  {
    
    
    if(!enabled_)
      return;
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
    //覆盖地图
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
    
  }
  
  
  
  void SimpleLayer::reconfigureCB ( simple_layer_namespace::SimpleLayerConfig& config, uint32_t level )
  {
    enabled_ = config.enabled;
  }
  
void SimpleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}

void SimpleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void SimpleLayer::reset()
{
    ROS_DEBUG("Reseting range sensor layer...");
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

}
