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

#ifndef SIMPLE_LAYER_H
#define SIMPLE_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/footprint.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <dynamic_reconfigure/server.h>

//#include "simple_layers/observation_buffer.h"
#include "simple_layers/SimpleLayerConfig.h"
#include "simple_layers/UltrasoundData.h"
#include "simple_layers/InfraredData.h"
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

using namespace costmap_2d;
namespace simple_layer_namespace
{
  class SimpleLayer : public costmap_2d::CostmapLayer
  //class SimpleLayer : public costmap_2d::Layer
  {
  public:
    SimpleLayer();
    ~SimpleLayer();
    
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, 
			      double* min_x, double* min_y, double* max_z, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    
    virtual void activate();
    virtual void deactivate();
    virtual void reset();
    
    
    bool isDiscretized()
    {
      return true;
    }
    void ultrasoundCB(const simple_layers::UltrasoundDataConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);
    void InfraredCB(const simple_layers::InfraredDataConstPtr& message, const boost::shared_ptr<ObservationBuffer>& buffer);
  private:
    bool getClearingObservations(std::vector<costmap_2d::Observation>& clearing_observations);
    bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations);
    void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y);
    void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y);
    
    
  public:
    bool rolling_window_; //动态窗口
    std::vector<geometry_msgs::Point> transformed_footprint_;
    std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief 用于观察消息过滤器
    std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief 用于确保每个传感器都可以使用变换
    
    std::vector<boost::shared_ptr<ObservationBuffer> > observation_buffers_;  ///< @brief 用于存储各种传感器的观察结果
    std::vector<boost::shared_ptr<ObservationBuffer> > marking_buffers_;  ///< @brief 用于存储标记障碍物的观察缓冲区
    std::vector<boost::shared_ptr<ObservationBuffer> > clearing_buffers_;  ///< @brief 用于存储清除障碍物的观察缓冲区
    
    double mark_x_;
    double mark_y_;
    ros::Publisher cloud2pub_t_;
    dynamic_reconfigure::Server<simple_layer_namespace::SimpleLayerConfig>* dsrv_;
    
  private:
    void reconfigureCB(simple_layer_namespace::SimpleLayerConfig &config, uint32_t level);
    
  };
}

#endif // SIMPLE_LAYER_H
