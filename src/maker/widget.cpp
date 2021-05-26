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

#include "widget.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QValidator>
#include <QTimer>
#include <QVBoxLayout>
#include <QApplication>


#include <fstream>
#include <yaml-cpp/yaml.h>
#include <chrono>

// ROS includes
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rviz_plugins
{
MotionCreator::MotionCreator(QWidget* parent)
    : tf2_listener_(tf2_buffer_)
    , prev_left_button_(false)
    , prev_right_button_(false)
{
  ec_sub_ = nh_.subscribe("/rviz/event_capture", 1, &rviz_plugins::MotionCreator::callback, this);

  save_dir_ = QDir::homePath();

  wps_ptr_ = make_unique<Waypoints>("motion_creator");

  createLayout();
}

void MotionCreator::update()
{
  // update target frame
  wps_ptr_->setTargetFrame(target_frame_->text().toUtf8().constData());
  //wps_.publishDeleteMarker();
  wps_ptr_->publishMarker();

  load_button_->setEnabled([this](){
    return edit_button_->isChecked();
  }());

  reset_button_->setEnabled([this](){
    return edit_button_->isChecked();
  }());

  save_button_->setEnabled([this](){
    return !target_frame_->text().isEmpty() && !velocity_->text().isEmpty() && wps_ptr_->getSize() >= 2
        && !edit_button_->isChecked();
  }());
}

void MotionCreator::edit(bool checked)
{
  if(checked)
  {
    target_frame_->setEnabled(true);
    obj_info_group_->setEnabled(true);
  }
  else
  {
    target_frame_->setEnabled(false);
    obj_info_group_->setEnabled(false);
  }

}

void MotionCreator::load()
{
  QString filepath = QFileDialog::getOpenFileName(this, "Load", QDir::homePath(), tr("YAML (*.yaml)"));

  if (filepath.isEmpty())
    return;

  try
  {
    YAML::Node node = YAML::LoadFile(filepath.toUtf8().constData());
    //for(const auto &n : node)
    //  ROS_INFO_STREAM(n.first << ": "  << n.second);
    target_frame_->setText(QString::fromStdString(node["target_frame"].as<std::string>()));
    velocity_->setText(QString::number(node["velocity"].as<double>()));

    std::vector<geometry_msgs::Pose> wps_intp;
    for(const auto& w : node["waypoints"])
    {
      geometry_msgs::Pose pose;
      pose.position.x = w["px"].as<double>();
      pose.position.y = w["py"].as<double>();
      pose.position.z = w["pz"].as<double>();
      pose.orientation.x = w["ox"].as<double>();
      pose.orientation.y = w["oy"].as<double>();
      pose.orientation.z = w["oz"].as<double>();
      pose.orientation.w = w["ow"].as<double>();
      wps_intp.push_back(pose);
    }

    std::vector<geometry_msgs::Pose> wps_raw;
    for(const auto& w : node["waypoints_raw"])
    {
      geometry_msgs::Pose pose;
      pose.position.x = w["px"].as<double>();
      pose.position.y = w["py"].as<double>();
      pose.position.z = w["pz"].as<double>();
      pose.orientation.x = w["ox"].as<double>();
      pose.orientation.y = w["oy"].as<double>();
      pose.orientation.z = w["oz"].as<double>();
      pose.orientation.w = w["ow"].as<double>();
      wps_raw.push_back(pose);
    }

    wps_ptr_->setTargetFrame(target_frame_->text().toUtf8().constData());
    wps_ptr_->load(wps_intp, wps_raw);
  }
  catch(YAML::Exception& e) {
    ROS_WARN_STREAM(e.what());
    //log_view_->insertPlainText(QString::fromStdString(std::string("Failure " + (std::string)e.what() + "\n")));
    //log_view_->moveCursor(QTextCursor::End);
  }
  catch(std::out_of_range &e)
  {
    ROS_WARN_STREAM(e.what());
    //log_view_->insertPlainText(QString::fromStdString(std::string("Failure " + (std::string)e.what() + "\n")));
    //log_view_->moveCursor(QTextCursor::End);
  }

  ROS_INFO("load: %s", filepath.toUtf8().constData());
  //log_view_->insertPlainText(QString::fromStdString(std::string("load: " + (std::string)filepath.toUtf8().constData() + "\n")));
  //log_view_->moveCursor(QTextCursor::End);
}

void MotionCreator::reset()
{
  auto result = QMessageBox::question(this, "Reset", "Reset ?", QMessageBox::Yes | QMessageBox::No);
  if (result == QMessageBox::Yes)
  {
    target_frame_->setText("");
    velocity_->setText("");
    wps_ptr_->reset();
  }

  //log_view_->insertPlainText(QString::fromStdString(std::string("reset \n")));
  //log_view_->moveCursor(QTextCursor::End);
}

void MotionCreator::save()
{
  QString filepath = QFileDialog::getSaveFileName(this, "Save", save_dir_, tr("YAML (*.yaml)"));

  if (filepath.isEmpty())
    return;

  filepath = [filepath](){
    std::string str = filepath.toUtf8().constData();
    if(str.find(".yaml") == std::string::npos)
      str += ".yaml";
    return QString::fromStdString(str);
  }();

  std::ofstream ofs(filepath.toUtf8().constData());

  // YAML
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // set target frame
  emitter << YAML::Key << "target_frame";
  emitter << YAML::Value << target_frame_->text().toUtf8().constData();

  // set velocity
  emitter << YAML::Key << "velocity";
  emitter << YAML::Value << velocity_->text().toUtf8().constData();
  emitter << YAML::Comment("(km/h)");

  auto wps2yaml = [&emitter](const geometry_msgs::Pose &pose){
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "px" << YAML::Value << pose.position.x;
    emitter << YAML::Key << "py" << YAML::Value << pose.position.y;
    emitter << YAML::Key << "pz" << YAML::Value << pose.position.z;
    emitter << YAML::Key << "ox" << YAML::Value << pose.orientation.x;
    emitter << YAML::Key << "oy" << YAML::Value << pose.orientation.y;
    emitter << YAML::Key << "oz" << YAML::Value << pose.orientation.z;
    emitter << YAML::Key << "ow" << YAML::Value << pose.orientation.w;
    emitter << YAML::EndMap;
  };

  // set waypoints
  emitter << YAML::Key << "waypoints";
  emitter << YAML::Value << YAML::BeginSeq;

  for(const auto &e : wps_ptr_->getWaypointsInterpolated())
    wps2yaml(e);
  emitter << YAML::EndSeq;

  // set waypoints raw
  emitter << YAML::Key << "waypoints_raw";
  emitter << YAML::Value << YAML::BeginSeq;

  for(const auto &e : wps_ptr_->getWaypointsRaw())
    wps2yaml(e);
  emitter << YAML::EndSeq;

  emitter << YAML::EndMap;
  //std::cout << emitter.c_str() << std::endl;
  ofs << emitter.c_str();
  ofs.close();


  //log_view_->insertPlainText(QString::fromStdString(std::string("saved:" + (std::string)filepath.toUtf8().constData() + "\n")));
  //log_view_->moveCursor(QTextCursor::End);

  // save as csv
  std::string filepath_csv = filepath.toUtf8().constData();
  filepath_csv +=".csv";
  std::ofstream ofs_csv(filepath_csv.c_str());
  ofs_csv << "x,y,z,yaw,velocity,change_flag" << std::endl;

  for(const auto &e : wps_ptr_->getWaypointsInterpolated())
  {
    ofs_csv << std::to_string(e.position.x) << "," << std::to_string(e.position.y) << "," << std::to_string(e.position.z) << ","  << tf2::getYaw(e.orientation) << "," << velocity_->text().toUtf8().constData() << "," << 0 << std::endl;
  }
  ofs_csv.close();
  
  save_dir_ = QFileInfo(filepath).dir().path();
}

void MotionCreator::check(int state)
{
  if(state == Qt::Unchecked)
  {
    pm_ptr_.release();
    check_z_label_->setText(": Not Initialized");
  }
  else if(state == Qt::Checked)
  {
    auto result = QMessageBox::question(this, "Subscribe /points_map", "get z from /points_map?\n already created waypoints is reinitialized.", QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      wps_ptr_->reset();

      check_z_label_->setText(": Initializing...");
      QApplication::processEvents();
      pm_ptr_.reset(new PointsMap());
      if(pm_ptr_->isSubscribed())
      {
        check_z_label_->setText(": Initialized");
      }
      else
      {
        check_z_label_->setText(": Initialize Failed");
        check_z_->setCheckState(Qt::Unchecked);
      }
    }
    else
    {
      check_z_->setCheckState(Qt::Unchecked);
    }

  }
}

void MotionCreator::check_reverse(int state)
{
  wps_ptr_->reverse();
  if(state == Qt::Unchecked)
  {
    wps_ptr_->setIsReverse(false);
  }
  else if(state == Qt::Checked)
  {
    wps_ptr_->setIsReverse(true);
  }
}

void MotionCreator::change_intp_mode(const QString qstr)
{
  ROS_INFO("mode: %s", qstr.toUtf8().constData());
  wps_ptr_->setInterpolationMode(qstr.toUtf8().constData());
}

void MotionCreator::callback(const event_capture::EventCaptureStampedConstPtr &msg)
{
  //ROS_INFO_STREAM(__FUNCTION__);
  if(!edit_button_->isChecked())
    return;

  auto ps_in = [msg]() {
    geometry_msgs::PointStamped ps;
    ps.header = msg->header;
    ps.point.x = msg->mouse.ray.origin.x;
    ps.point.y = msg->mouse.ray.origin.y;
    return ps;
  }();

  auto transformPointStamped = [this, msg, &ps_in]() -> std::pair<bool, geometry_msgs::PointStamped> {

    geometry_msgs::PointStamped ps_out;

    try
    {
      tf2_buffer_.transform(ps_in, ps_out, target_frame_->text().toUtf8().constData(), ros::Duration(0.0));

      /*ROS_INFO("in(x:%lf y:%lf z:%lf) -> out(x:%lf y:%lf z:%lf)",
               ps_in.point.x,
               ps_in.point.y,
               ps_in.point.z,
               ps_out.point.x,
               ps_out.point.y,
               ps_out.point.z);*/

      // get z value from pointcloud
      //ROS_INFO("pcl_sub: %s, check: %s", (is_pointcloud_subscribed_ != 0) ? "True" : "False", (check_z_->isChecked() != 0) ? "True": "False");
      if(pm_ptr_ != nullptr)
      {
        auto res_pair = pm_ptr_->requestPositionZ(ps_out.point);
        if(res_pair.first)
          ps_out.point.z = res_pair.second;
      }

      //ROS_INFO("end");
      return std::make_pair(true, ps_out);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_STREAM(e.what());
      // Print exception which was caught
      //log_view_->insertPlainText(QString::fromStdString(std::string("Failure " + (std::string)ex.what() + "\n")));
      //log_view_->moveCursor(QTextCursor::End);
      return std::make_pair(false, geometry_msgs::PointStamped());
    }
  };

  //ROS_INFO("shift: %s", msg->capture.shift ? "True" : "False");
  //ROS_INFO("prev left: %s", prev_left_button_ ? "True" : "False");
  //ROS_INFO("left: %s", msg->capture.left ? "True" : "False");
  //ROS_INFO("prev right: %s", prev_right_button_ ? "True" : "False");
  //ROS_INFO("right: %s", msg->capture.right ? "True" : "False");


  if (!msg->mouse.shift)
  {
    if(msg->mouse.ctrl){
      if (!prev_right_button_ && msg->mouse.right) // right down
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("erase: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_ptr_->erase(ps_out_pair.second);
      }
      //log_view_->insertPlainText(QString::fromStdString(
      //    std::string("erase: " + std::to_string(ps_out.point.x) + ", " + std::to_string(ps_out.point.y) + "\n")
      //));
      //log_view_->moveCursor(QTextCursor::End);
    }
    }else
    {
      if (!prev_right_button_ && msg->mouse.right) // right down
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("add: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_ptr_->add(ps_out_pair.second);
      }

      //log_view_->insertPlainText(QString::fromStdString(
      //    std::string("add: " + std::to_string(ps_out.point.x) + ", " + std::to_string(ps_out.point.y) + "\n")
       //   ));
      //log_view_->moveCursor(QTextCursor::End);
    }
    }
    
  }
  else
  {
    // move
    if(!prev_right_button_ && msg->mouse.right) // left down
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("select: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_ptr_->select(ps_out_pair.second);
      }
      //log_view_->insertPlainText(QString::fromStdString(
      //    std::string("move: " + std::to_string(ps_out.point.x) + ", " + std::to_string(ps_out.point.y) + ", " + std::to_string(ps_out.point.z) + "\n")
      //));
      //log_view_->moveCursor(QTextCursor::End);
    }
    else if(prev_right_button_ && msg->mouse.right) // left keep
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("move: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_ptr_->move(ps_out_pair.second);
      }

    }
    else if(prev_right_button_ && !msg->mouse.right) // left up
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("release: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_ptr_->release();
      }
    }
  }

  prev_left_button_ = msg->mouse.left;
  prev_right_button_ = msg->mouse.right;
}



void MotionCreator::createLayout()
{
  // Setup Layout

  // Target Frame
  auto target_frame_label = new QLabel(tr("TargetFrame"));
  target_frame_ = new QLineEdit;
  target_frame_->setPlaceholderText("ex. world");
  target_frame_->setEnabled(false);
  auto tf_layout = new QHBoxLayout();
  tf_layout->addWidget(target_frame_label);
  tf_layout->addWidget(target_frame_);

  // Other parameter
  auto velocity_label = new QLabel(tr("Velocity :"));
  velocity_ = new QLineEdit;
  velocity_->setPlaceholderText("[km/h]");
  velocity_->setValidator( new QDoubleValidator(0, 100, 2, this));
  auto others_layout = new QGridLayout;
  others_layout->addWidget(velocity_label, 0, 0);
  others_layout->addWidget(velocity_, 0, 1);
  auto others_group = new QGroupBox(tr("Others"));
  others_group->setLayout(others_layout);

  // parameter layout
  auto param_layout = new QGridLayout;
  param_layout->addWidget(others_group, 0, 1);

  // checkbox
  intp_mode_label_ = new QLabel("interpolation mode: ");
  intp_mode_box_ = new QComboBox();
  intp_mode_box_->addItem("linear");
  intp_mode_box_->addItem("spline");
  auto intp_mode_layout = new QHBoxLayout();
  intp_mode_layout->addWidget(intp_mode_label_);
  intp_mode_layout->addWidget(intp_mode_box_);

  check_z_label_ = new QLabel(tr(": Not Initialized"));
  check_z_ = new QCheckBox("get z from /points_map");
  auto check_z_layout = new QHBoxLayout();
  check_z_layout->addWidget(check_z_);
  check_z_layout->addWidget(check_z_label_);

  reverse_ = new QCheckBox("reverse route");
  csv_ = new QCheckBox("output *.csv");

  // button layout
  edit_button_ = new QPushButton("EDIT");
  edit_button_->setCheckable(true);

  reset_button_ = new QPushButton("RESET");
  save_button_ = new QPushButton("SAVE");
  load_button_ = new QPushButton("LOAD");

  connect(edit_button_, &QPushButton::clicked, this, &MotionCreator::edit);
  connect(load_button_, &QPushButton::clicked, this, &MotionCreator::load);
  connect(reset_button_, &QPushButton::clicked, this, &MotionCreator::reset);
  connect(save_button_, &QPushButton::clicked, this, &MotionCreator::save);
  connect(check_z_, &QCheckBox::stateChanged, this, &MotionCreator::check);
  connect(reverse_, &QCheckBox::stateChanged, this, &MotionCreator::check_reverse);
  connect(intp_mode_box_, &QComboBox::currentTextChanged, this, &MotionCreator::change_intp_mode);

  auto button_layout = new QHBoxLayout();
  button_layout->addWidget(edit_button_);
  button_layout->addWidget(load_button_);
  button_layout->addWidget(reset_button_);
  button_layout->addWidget(save_button_);

  auto obj_info_layout = new QVBoxLayout();
  obj_info_layout->addLayout(param_layout);
  obj_info_layout->addLayout(intp_mode_layout);
  obj_info_layout->addLayout(check_z_layout);
  obj_info_layout->addWidget(reverse_);
  obj_info_layout->addWidget(csv_);

  obj_info_group_ = new QGroupBox(tr("Waypoint Information"));
  obj_info_group_->setLayout(obj_info_layout);
  obj_info_group_->setEnabled(false);

  //log_view_ = new QPlainTextEdit();
  //log_view_->setReadOnly(true);


  // integrate layout
  auto vlayout = new QVBoxLayout();
  vlayout->addLayout(tf_layout);
  vlayout->addWidget(obj_info_group_);
  vlayout->addLayout(button_layout);
  //vlayout->addWidget(log_view_);
  setLayout(vlayout);

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &MotionCreator::update);
  timer->start(1000 / 30);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MotionCreator, rviz::Panel)
