//
//  Copyright 2019 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <rviz/visualization_manager.h>

#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QLineEdit>
#include <QGroupBox>
#include <QCheckBox>
#include <QLabel>
#include <QComboBox>

#include <vector>
#include <memory>

#include "../utils/waypoints.hpp"
#include "../utils/points_map.hpp"
#include "event_capture/EventCaptureStamped.h"

namespace rviz_plugins
{

template<typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts&&... params)
{
  return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
}

class MotionCreator : public rviz::Panel
{
  Q_OBJECT

public:
  MotionCreator(QWidget* parent = 0);

private:
  std::unique_ptr<Waypoints> wps_ptr_;
  std::unique_ptr<PointsMap> pm_ptr_;
  ros::NodeHandle nh_;
  ros::Subscriber ec_sub_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  //parameter
  QString save_dir_;
  //QPlainTextEdit* log_view_;

  QLineEdit *velocity_;
  QLineEdit *target_frame_;

  QPushButton* edit_button_;
  QPushButton* save_button_;
  QPushButton* reset_button_;
  QPushButton* load_button_;

  QCheckBox* check_z_;
  QLabel *check_z_label_;

  QCheckBox* reverse_;
  QCheckBox* csv_;

  QGroupBox* obj_info_group_;

  QComboBox* intp_mode_box_;
  QLabel *intp_mode_label_;

  bool prev_left_button_;
  bool prev_right_button_;

  void callback(const event_capture::EventCaptureStampedConstPtr &msg);
  void createLayout();

private Q_SLOTS:

  void update();
  void edit(bool checked);
  void load();
  void reset();
  void save();
  void check(int state);
  void check_reverse(int state);
  void change_intp_mode(const QString qstr);


};
}
