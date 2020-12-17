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

#include <dyn_components.h>

#include <string>
#include <vector>

namespace rosdyn_gui
{

FrictionWidgets::FrictionWidgets(ros::NodeHandle& nh,
                                 const std::string& joint_name,
                                 QWidget* parent): QWidget(parent), m_nh(nh)
{
  if (!m_nh.getParam("/meto_cfg/urdf_name", m_robot_name))
    ROS_ERROR("No /meto_cfg/urdf_name");


  m_joint_name = joint_name;
  setWindowTitle(QString(m_joint_name.c_str()));

  m_vbox = new QVBoxLayout(this);

  m_friction_model.resize(3);
  m_friction_model.at(0) = new QRadioButton(tr("None"));
  m_friction_model.at(1) = new QRadioButton(tr("Polynomial 1°"));
  m_friction_model.at(2) = new QRadioButton(tr("Polynomial 2°"));

  m_vbox->addWidget(new QLabel(tr(m_joint_name.c_str())));
  m_vbox->addWidget(m_friction_model.at(0));
  m_vbox->addWidget(m_friction_model.at(1));
  m_vbox->addWidget(m_friction_model.at(2));
  m_vbox->addStretch(0);

  loadParam();
}

void FrictionWidgets::loadParam()
{
  m_type = "None";
  m_max_velocity = 100;
  m_min_velocity = 1.0e-6;
  m_coefficients.resize(10, 0);


  m_nh.getParam(m_robot_name + "/" + m_joint_name + "/friction/type", m_type);

  if (!m_type.compare("Polynomial1"))
  {
    m_nh.getParam(m_robot_name + "/" + m_joint_name + "/friction/constants/max_velocity", m_max_velocity);
    m_nh.getParam(m_robot_name + "/" + m_joint_name + "/friction/constants/max_velocity", m_min_velocity);
    m_nh.getParam(m_robot_name + "/" + m_joint_name + "/friction/coefficients/columb", m_coefficients.at(0));
    m_nh.getParam(m_robot_name + "/" + m_joint_name + "/friction/coefficients/viscous", m_coefficients.at(1));
    m_friction_model.at(1)->setChecked(true);
  }
  else if (!m_type.compare("Polynomial2"))
  {
    m_nh.getParam(m_robot_name + "/" + m_joint_name +
                  "/friction/constants/max_velocity", m_max_velocity);
    m_nh.getParam(m_robot_name + "/" + m_joint_name +
                  "/friction/constants/max_velocity", m_min_velocity);
    m_nh.getParam(m_robot_name + "/" + m_joint_name +
                  "/friction/coefficients/columb", m_coefficients.at(0));
    m_nh.getParam(m_robot_name + "/" + m_joint_name +
                  "/friction/coefficients/first_order_viscous", m_coefficients.at(1));
    m_nh.getParam(m_robot_name + "/" + m_joint_name +
                  "/friction/coefficients/second_order_viscous", m_coefficients.at(2));
    m_friction_model.at(2)->setChecked(true);
  }
  else
    m_friction_model.at(0)->setChecked(true);
}

void FrictionWidgets::saveParam()
{
  checkedFriction();
  m_nh.setParam(m_robot_name + "/" + m_joint_name + "/friction/type", m_type);
  m_nh.setParam(m_robot_name + "/" + m_joint_name + "/friction/constants/max_velocity", m_max_velocity);
  m_nh.setParam(m_robot_name + "/" + m_joint_name + "/friction/constants/max_velocity", m_min_velocity);

  m_nh.setParam(m_robot_name + "/" + m_joint_name + "/friction/coefficients/columb", m_coefficients.at(0));
  if (!m_type.compare("Polynomial1"))
  {
    m_nh.setParam(m_robot_name + "/" + m_joint_name +
                  "/friction/coefficients/viscous", m_coefficients.at(1));
    m_nh.deleteParam(m_robot_name + "/" + m_joint_name +
                     "/friction/coefficients/first_order_viscous");
    m_nh.deleteParam(m_robot_name + "/" + m_joint_name +
                     "/friction/coefficients/second_order_viscous");
  }
  else   if (!m_type.compare("Polynomial2"))
  {
    m_nh.deleteParam(m_robot_name + "/" + m_joint_name +
                     "/friction/coefficients/viscous");
    m_nh.setParam(m_robot_name + "/" + m_joint_name +
                  "/friction/coefficients/first_order_viscous", m_coefficients.at(1));
    m_nh.setParam(m_robot_name + "/" + m_joint_name +
                  "/friction/coefficients/second_order_viscous", m_coefficients.at(2));
  }
}

int FrictionWidgets::checkedFriction()
{
  if (m_friction_model.at(0)->isChecked())
  {
    m_type = "None";
    return 0;
  }
  else if (m_friction_model.at(0)->isChecked())
  {
    m_type = "Polynomial1";
    return 1;
  }
  else if (m_friction_model.at(0)->isChecked())
  {
    m_type = "Polynomial2";
    return 2;
  }

  return -1;
}



ModelTab::ModelTab(ros::NodeHandle& nh, QWidget* parent): QWidget(parent), m_nh(nh)
{
  std::vector<std::string> joints;
  if (!m_nh.getParam("/meto_cfg/controller_joint_names", joints))
    ROS_ERROR("no controller_joint_names");

  m_vbox = new QVBoxLayout(this);
  m_components.resize(joints.size());
  for (auto idx = 0; idx < joints.size(); idx++)
  {
    m_components.at(idx) = new FrictionWidgets(nh, joints.at(idx), this);
    m_vbox->addWidget(m_components.at(idx));
  }

  m_ok_btn = new QPushButton();
  m_ok_btn->setText("Store");
  connect(m_ok_btn, SIGNAL(clicked()), this, SLOT(saveNewPar()));
  m_vbox->addWidget(m_ok_btn);
}

void ModelTab::saveNewPar()
{
  for (rosdyn_gui::FrictionWidgets* component : m_components)
    component->saveParam();
}

}  // namespace rosdyn_gui
