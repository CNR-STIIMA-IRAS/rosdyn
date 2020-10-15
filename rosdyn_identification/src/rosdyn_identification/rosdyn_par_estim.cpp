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

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_identification/rosdyn_par_estim.h>


namespace rosdyn
{
MetoParEstim::MetoParEstim(ros::NodeHandle& nh, const std::string& robot_description, const bool& verbose):
  m_nh(nh),
  m_verbose(verbose)
{
  std::string base_frame, tool_frame;

  m_model = urdf::parseURDF(robot_description);
  if (!m_model)
    throw std::invalid_argument("PARAMETER ' " + robot_description + "' NOT FOUND");

  m_par_computed = false;
  std::vector<std::string> js_name;

  m_robot_name = m_model->getName();

  if (!m_nh.getParam(m_model->getName() + "/joint_names", js_name))
    throw std::invalid_argument("PARAMETER ' " + m_model->getName() + "/joint_names' NOT FOUND");

  if (!m_nh.getParam(m_model->getName() + "/base_link", base_frame))
    throw std::invalid_argument("PARAMETER '" + m_model->getName() + "/base_link' NOT FOUND");

  if (!m_nh.getParam(m_model->getName() + "/tool_link", tool_frame))
    throw std::invalid_argument("PARAMETER '" + m_model->getName() + "/tool_link' NOT FOUND");

  int njnt = js_name.size();

  std::vector<double> grav_stl(3);
  if (!m_nh.getParam(m_model->getName() + "/gravity", grav_stl))
    throw std::invalid_argument("PARAMETER '" + m_model->getName() + "/gravity' NOT FOUND");

  if (grav_stl.size() != 3)
    throw std::invalid_argument("PARAMETER '" + m_model->getName() + "/gravity' has wrong dimension!");
  Eigen::Vector3d grav;
  grav << grav_stl.at(0), grav_stl.at(1), grav_stl.at(2);

  rosdyn::LinkPtr root_link(new rosdyn::Link());
  root_link->fromUrdf(m_model->root_link_);

  m_chain.reset(new rosdyn::Chain(root_link, base_frame, tool_frame, grav));
  m_chain->setInputJointsName(js_name);

  m_joints_names = js_name;

  try
  {
    for (unsigned int idx = 0; idx < js_name.size(); idx++)
    {
      std::string component_type;
      if (m_nh.getParam(m_model->getName() + "/" + js_name.at(idx) + "/spring/type", component_type))
      {
        if (!component_type.compare("Ideal"))
        {
          ROS_INFO("JOINT '%s' has a spring component", js_name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::IdealSpring(js_name.at(idx), m_model->getName(), m_nh)));
        }
      }

      if (m_nh.getParam(m_model->getName() + "/" + js_name.at(idx) + "/friction/type", component_type))
      {
        if (!component_type.compare("Polynomial1"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial1 component", js_name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::FirstOrderPolynomialFriction(js_name.at(idx), m_model->getName(), m_nh)));
        }
        else if (!component_type.compare("Polynomial2"))
        {
          ROS_INFO("JOINT '%s' has a Polynomial2 component", js_name.at(idx).c_str());
          m_components.push_back(rosdyn::ComponentPtr(new rosdyn::SecondOrderPolynomialFriction(js_name.at(idx), m_model->getName(), m_nh)));
        }
      }
    }
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Exception: %s", e.what());
  }
  m_additional_parameters = 0;
  for (unsigned int idx = 0; idx < m_components.size(); idx++)
    m_additional_parameters += m_components.at(idx)->getParametersNumber();

  Eigen::VectorXd q(njnt);
  q.setZero();
  Eigen::VectorXd Dq(njnt);
  Dq.setZero();
  Eigen::VectorXd DDq(njnt);
  DDq.setZero();

  Eigen::VectorXd tau(njnt);
  tau.setZero();

  int n_joint_number = m_chain->getActiveJointsNumber();
  size_t npoint = 1000;
  int n_moveable_links = m_chain->getLinksNumber() - 1;

  Eigen::MatrixXd Phi(npoint * n_joint_number, (10 * n_moveable_links) + m_additional_parameters);
  Eigen::VectorXd Tau(npoint * n_joint_number);
  Phi.setZero();
  Tau.setZero();
  for (size_t idx = 0; idx < npoint; idx++)
  {
    q.setRandom();
    Dq.setRandom();
    DDq.setRandom();

    Tau.block(n_joint_number * idx, 0, n_joint_number, 1)  = m_chain->getJointTorque(q, Dq, DDq);
    Phi.block(n_joint_number * idx, 0, n_joint_number, 10 * n_moveable_links) = m_chain->getRegressor(q, Dq, DDq);

    unsigned int starting_column = 10 * n_moveable_links;
    for (size_t iComponent = 0; iComponent < m_components.size(); iComponent++)
    {
      Phi.block(n_joint_number * idx, starting_column, n_joint_number, m_components.at(iComponent)->getParametersNumber()) = m_components.at(iComponent)->getRegressor(q, Dq, DDq);
      starting_column += m_components.at(iComponent)->getParametersNumber();
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Phi,  Eigen::ComputeThinU | Eigen::ComputeThinV);
  int rank = 0;
  while (std::abs(svd.singularValues()(rank)) > 1e-10)
  {
    rank++;
    if (rank == (svd.matrixV().rows()))
      break;
  }

  m_T_base_to_full = svd.matrixV().block(0, 0, svd.matrixV().rows(), rank);
  if ((svd.matrixV().cols() - rank) > 0)
    m_T_null_to_full = svd.matrixV().block(0, rank, svd.matrixV().rows(),  svd.matrixV().cols() - rank);
  else
    m_T_null_to_full.resize(svd.matrixV().rows(), 0);

  loadNominalParameters();

  return;
}


MetoParEstim::MetoParEstim(ros::NodeHandle& nh, const std::string& robot_description, Eigen::MatrixXd& T, const bool& verbose):
  MetoParEstim(nh, robot_description, verbose)
{
  m_T_base_to_full = T;
}


Eigen::MatrixXd MetoParEstim::getTrajectoryRegressor(const Eigen::MatrixXd& q,
    const Eigen::MatrixXd& Dq,
    const Eigen::MatrixXd& DDq)
{
  m_par_computed = false;

  int n_joint_number = m_chain->getActiveJointsNumber();
  int npoint = q.rows();

  int n_moveable_links = m_chain->getLinksNumber() - 1;


  Eigen::MatrixXd Phi_;
  Phi_.resize(npoint * n_joint_number, (10 * n_moveable_links) + m_additional_parameters);

  for (int idx = 0; idx < npoint; idx++)
  {
    Eigen::VectorXd q_tmp, Dq_tmp, DDq_tmp;
    q_tmp   = q.row(idx);
    Dq_tmp  = Dq.row(idx);
    DDq_tmp = DDq.row(idx);

    Phi_.block(n_joint_number * idx, 0, n_joint_number, 10 * n_moveable_links) = m_chain->getRegressor(q_tmp, Dq_tmp, DDq_tmp);

    unsigned int starting_column = 10 * n_moveable_links;
    for (unsigned int iComponent = 0; iComponent < m_components.size(); iComponent++)
    {
      Phi_.block(n_joint_number * idx, starting_column, n_joint_number, m_components.at(iComponent)->getParametersNumber()) = m_components.at(iComponent)->getRegressor(q_tmp, Dq_tmp, DDq_tmp);
      starting_column += m_components.at(iComponent)->getParametersNumber();
    }

    if (npoint >= 10)
      if (idx % (npoint / 10) == 0)
      {
        int tmp_i_ = static_cast<int>(std::ceil(static_cast<double>(idx * 100.0) / static_cast<double>(npoint)));
        ROS_DEBUG("dynamics parameters loading %d ", tmp_i_);
      }
  }
  m_phi_full = Phi_;
  m_phi_base = Phi_ * m_T_base_to_full;
  m_svd_num_base_phi.compute(m_phi_base, Eigen::ComputeThinU | Eigen::ComputeThinV);
  m_svd_null.compute(m_T_null_to_full, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (m_verbose)
  {
    ROS_INFO("Phi dimension = %zu x %zu", Phi_.rows(), Phi_.cols());
    ROS_INFO("Reduced Phi dimension = %zu x %zu", m_phi_base.rows(), m_phi_base.cols());
    ROS_INFO("Reduced Phi first singular value = %e", m_svd_num_base_phi.singularValues()(0));
    ROS_INFO("Reduced Phi last singular value = %e", m_svd_num_base_phi.singularValues()(m_T_base_to_full.cols() - 1));
  }

  int rank = 0;
  while (std::abs(m_svd_num_base_phi.singularValues()(rank)) > 1e-10)
  {
    rank++;
    if (rank == (m_svd_num_base_phi.matrixV().rows()))
      break;
  }
  if (rank > m_phi_base.cols())
    ROS_WARN("rank %d, expected %zu", rank, m_phi_base.cols());

  m_condition_number = m_svd_num_base_phi.singularValues()(0) / m_svd_num_base_phi.singularValues()(m_phi_base.cols() - 1);
  return m_phi_base;
}

double MetoParEstim::getConditionNumber()
{
  return m_condition_number;
}

Eigen::MatrixXd MetoParEstim::getTrajectoryFullRegressor(const Eigen::MatrixXd& q, const Eigen::MatrixXd& Dq, const Eigen::MatrixXd& DDq)
{
  return m_phi_full;
}


Eigen::VectorXd MetoParEstim::getEstimatedParameters(const Eigen::MatrixXd& tau)
{
  if (m_verbose)
  {
    ROS_INFO("tau vector dimension = %zu x %zu", tau.rows(), tau.cols());
  }
  if (tau.size() != m_phi_base.rows())
    throw std::invalid_argument("Tau has wrong dimensions");

  Eigen::VectorXd Tau(tau.size());

  for (unsigned int idx = 0; idx < tau.rows(); idx++)
    Tau.block(tau.cols() *idx, 0, tau.cols(), 1) = tau.row(idx).transpose();




  m_base_par =  m_svd_num_base_phi.solve(Tau);

  if (m_verbose)
  {
    double error = (Tau - m_phi_base * m_base_par).array().abs().mean();
    ROS_INFO("mean error = %e", error);
  }

  Eigen::VectorXd null_par(m_T_null_to_full.cols());
  null_par.setZero();


  // m_full_par = m_T_base_to_full*m_base_par + m_T_null_to_full*null_par = m_nominal_parameters
  // m_T_null_to_full*null_par = m_nominal_parameters-m_T_base_to_full*m_base_par
  null_par = m_svd_null.solve(m_nominal_parameters - m_T_base_to_full * m_base_par);


  m_full_par = m_T_base_to_full * m_base_par + m_T_null_to_full * null_par;
  m_par_computed = true;

  return m_base_par;
}

bool MetoParEstim::saveParXml()
{
  //****************************************
  // Inertial Data
  //****************************************
  std::string username = getenv("LOGNAME");
  std::string path = std::string("/home/") + username + "/.ros/";
  std::string xml_new_ = path + m_robot_name + "_estimated.urdf";
  Eigen::VectorXd& dyn_par = m_full_par;

  int numb_of_link_param_ = 10;
  std::vector<std::string> chain_links_names_ = m_chain->getLinksName();

  urdf::ModelInterfaceSharedPtr robot_model_copy_ = m_model;

  int iLink = 0;

  for (auto itChLinkName = chain_links_names_.begin() + 1; itChLinkName != chain_links_names_.end(); itChLinkName++)
  {
    ROS_DEBUG("Extracting data from link: %s", (*itChLinkName).c_str());

    urdf::LinkSharedPtr link_;
    robot_model_copy_->getLink(*itChLinkName, link_);

    if (link_)
    {
      if (!link_->inertial)
        link_->inertial.reset<urdf::Inertial>(new urdf::Inertial());

      ROS_DEBUG("%s: mass %f, nominal %f", (*itChLinkName).c_str(), dyn_par(iLink + 0), m_nominal_parameters(iLink + 0));
      link_->inertial->mass = dyn_par(iLink + 0);

      ROS_DEBUG("%s: mass*cx %f, nominal %f", (*itChLinkName).c_str(), dyn_par(iLink + 1), m_nominal_parameters(iLink + 1));
      ROS_DEBUG("%s: mass*cy %f, nominal %f", (*itChLinkName).c_str(), dyn_par(iLink + 2), m_nominal_parameters(iLink + 2));
      ROS_DEBUG("%s: mass*cz %f, nominal %f", (*itChLinkName).c_str(), dyn_par(iLink + 3), m_nominal_parameters(iLink + 3));

      if (std::abs(link_->inertial->mass) > 1e-15)
      {
        link_->inertial->origin.position.x = dyn_par(iLink + 1) / link_->inertial->mass;
        link_->inertial->origin.position.y = dyn_par(iLink + 2) / link_->inertial->mass;
        link_->inertial->origin.position.z = dyn_par(iLink + 3) / link_->inertial->mass;
      }
      else
      {
        link_->inertial->origin.position.x = 0.0;
        link_->inertial->origin.position.y = 0.0;
        link_->inertial->origin.position.z = 0.0;
      }
      ROS_DEBUG("%s: cx %f", (*itChLinkName).c_str(), link_->inertial->origin.position.x);
      ROS_DEBUG("%s: cy %f", (*itChLinkName).c_str(), link_->inertial->origin.position.y);
      ROS_DEBUG("%s: cz %f", (*itChLinkName).c_str(), link_->inertial->origin.position.z);

      Eigen::MatrixXd I0(3, 3);
      I0(0, 0) = dyn_par(iLink + 4);

      I0(0, 1) = dyn_par(iLink + 5);
      I0(1, 0) = dyn_par(iLink + 5);

      I0(0, 2) = dyn_par(iLink + 6);
      I0(2, 0) = dyn_par(iLink + 6);

      I0(1, 1) = dyn_par(iLink + 7);

      I0(1, 2) = dyn_par(iLink + 8);
      I0(2, 1) = dyn_par(iLink + 8);

      I0(2, 2) = dyn_par(iLink + 9);


      double mass = dyn_par(iLink + 0);

      Eigen::Vector3d cog;
      cog(0) = dyn_par(iLink + 1) / mass;
      cog(1) = dyn_par(iLink + 2) / mass;
      cog(2) = dyn_par(iLink + 3) / mass;
      Eigen::MatrixXd cog_skew = rosdyn::skew(cog);


      Eigen::MatrixXd I = I0 - mass * (cog_skew * cog_skew.transpose());

      ROS_DEBUG("%s: ixx (inertia frame) %f, nominal %f", (*itChLinkName).c_str(), I0(0, 0), m_nominal_parameters(iLink + 4));
      ROS_DEBUG("%s: ixy (inertia frame) %f, nominal %f", (*itChLinkName).c_str(), I0(0, 1), m_nominal_parameters(iLink + 5));
      ROS_DEBUG("%s: ixz (inertia frame) %f, nominal %f", (*itChLinkName).c_str(), I0(0, 2), m_nominal_parameters(iLink + 6));
      ROS_DEBUG("%s: iyy (inertia frame) %f, nominal %f", (*itChLinkName).c_str(), I0(1, 1), m_nominal_parameters(iLink + 7));
      ROS_DEBUG("%s: iyz (inertia frame) %f, nominal %f", (*itChLinkName).c_str(), I0(1, 2), m_nominal_parameters(iLink + 8));
      ROS_DEBUG("%s: izz (inertia frame) %f, nominal %f", (*itChLinkName).c_str(), I0(2, 2), m_nominal_parameters(iLink + 9));

      ROS_DEBUG("%s: ixx (link frame) %f", (*itChLinkName).c_str(), I(0, 0));
      ROS_DEBUG("%s: ixy (link frame) %f", (*itChLinkName).c_str(), I(0, 1));
      ROS_DEBUG("%s: ixz (link frame) %f", (*itChLinkName).c_str(), I(0, 2));
      ROS_DEBUG("%s: iyy (link frame) %f", (*itChLinkName).c_str(), I(1, 1));
      ROS_DEBUG("%s: iyz (link frame) %f", (*itChLinkName).c_str(), I(1, 2));
      ROS_DEBUG("%s: izz (link frame) %f", (*itChLinkName).c_str(), I(2, 2));

      link_->inertial->ixx = I(0, 0);
      link_->inertial->ixy = I(0, 1);
      link_->inertial->ixz = I(0, 2);
      link_->inertial->iyy = I(1, 1);
      link_->inertial->iyz = I(1, 2);
      link_->inertial->izz = I(2, 2);

      iLink += numb_of_link_param_;
    }
    else
    {
      ROS_ERROR("Cannot find: %s", (*itChLinkName).c_str());
      return false;
    }
  }

  // save inertial data
  TiXmlDocument* xml_doc_ = urdf::exportURDF(robot_model_copy_);
  ROS_INFO("Saving URDF file: %s",  xml_new_.c_str());
  if (!xml_doc_->SaveFile(xml_new_))
    ROS_ERROR("failed saving %s", xml_new_.c_str());


  //****************************************
  // Friction Data
  //****************************************
  unsigned int iComponent = 0;
  for (rosdyn::ComponentPtr& component : m_components)
  {
    Eigen::VectorXd par = dyn_par.block(iLink + iComponent, 0, component->getParametersNumber(), 1);

    ROS_FATAL_STREAM(component->getJointName() << ": " << par.transpose());
    iComponent += component->getParametersNumber();
    if (!component->setParameters(par))
      ROS_ERROR("something wrong in component");
    else
      component->saveParameters();
  };

  std::string string_to_save_yaml =   "rosparam dump "
                                      + path + m_robot_name +
                                      + "_estimated.yaml"
                                      + " /"
                                      + m_robot_name;

  ROS_INFO("Executing command: %s", string_to_save_yaml.c_str());
  if (system(string_to_save_yaml.c_str()) != 0)
    ROS_WARN("Unable to store yaml parameters");

  return true;
}


Eigen::VectorXd MetoParEstim::getFullEstimatedParameters()
{
  if (m_par_computed)
    return m_full_par;

  ROS_ERROR("Parameters do not computed");
  Eigen::VectorXd tmp(0, 1);
  return tmp;
}


void MetoParEstim::loadNominalParameters()
{
  int n_moveable_links = m_chain->getLinksNumber() - 1;

  m_nominal_parameters.resize(10 * n_moveable_links + m_additional_parameters);
  m_nominal_parameters.setZero();
  int ic = 10 * n_moveable_links;

  m_nominal_parameters.block(0, 0, 10 * n_moveable_links, 1) = m_chain->getNominalParameters();

  for (unsigned int idx = 0; idx < m_components.size(); idx++)
  {
    int npar = m_components.at(idx)->getParametersNumber();
    m_nominal_parameters.block(ic, 0, npar, 1) = m_components.at(idx)->getNominalParameters();
    ic += npar;
  }
}



}  // namespace rosdyn
