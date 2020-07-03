/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once  // NOLINT(build/header_guard)

#include <ros/ros.h>
#include <QVBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QComboBox>
#include <QDialog>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <string>
#include <vector>

namespace rosdyn_gui
{

class FrictionWidgets : public QWidget
{
  Q_OBJECT

public:
  explicit FrictionWidgets(ros::NodeHandle& nh, const std::string& joint_name, QWidget *parent = 0);

  int checkedFriction();
  std::string jointName()
  {
    return m_joint_name;
  };
  void saveParam();
  void loadParam();
protected Q_SLOTS:  // NOLINT(build/include)
protected:
  std::string m_joint_name;
  std::string m_robot_name;
  ros::NodeHandle m_nh;
  std::vector<QRadioButton*> m_friction_model;
//     QSpinBox* m_vel_threshold;
//     QSpinBox* m_max_threshold;
  QVBoxLayout* m_vbox;
  std::string m_type;
  int m_max_velocity;
  double m_min_velocity;
  std::vector<double> m_coefficients;
};

class ModelTab : public QWidget
{
  Q_OBJECT

public:
  explicit ModelTab(ros::NodeHandle& nh, QWidget *parent = 0);

protected Q_SLOTS:
  void saveNewPar();
protected:
  ros::NodeHandle m_nh;
  QVBoxLayout* m_vbox;
  std::vector<FrictionWidgets*> m_components;
  QSpinBox* m_vel_threshold;
  QSpinBox* m_max_threshold;
  QPushButton* m_ok_btn;
};


}  // namespace rosdyn_gui
