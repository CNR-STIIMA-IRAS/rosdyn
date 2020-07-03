#ifndef rosyn_par_estim_nodelet
#define rosyn_par_estim_nodelet

#include <mutex>
#include <thread>
#include <fstream> 
#include <iostream>

#include <nodelet/nodelet.h>
#include <boost/graph/graph_concepts.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/server/simple_action_server.h>

#include <rosdyn_identification_msgs/MetoParEstimAction.h>
#include <rosdyn_identification/rosdyn_par_estim.h>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <std_srvs/Empty.h>

/*! \file meto_par_estim_interface.h
    \brief A Documented file.
    
    Details.
*/

namespace rosdyn
{
  
  //! The Nodelet inteface class to \link MetoParEstim \endlink. 
  /*!
    The class is the Nodelet interface to the base class \link MetoParEstim \endlink .    
  */
  class MetoParEstimInterfaceNodelet : public nodelet::Nodelet
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //! Initialization of the ROS infrastructure for Nodelet.
    /*!
      Virtual and must be overridden by subclass. All initialization of the ROS infrastructure must be put into this function.
    */
    virtual void onInit();
    
  protected:
    //! Flag to stop the Nodelet thread
    bool m_stop;
    
    //! Node parameters' namepace.
    std::string m_namespace;
    
    //! Temporary string that contains the binary file to be loaded
    std::string m_file_name;
    std::string m_model_name;
    
    //! Main Nodelet thread for the dynamics parameters estimation.
    std::thread m_main_thread;
    
    ros::ServiceServer m_save_model_server;

    std::shared_ptr<rosdyn::MetoParEstim> m_estimator;
    
    //! Action Server.
    /*!
      Action Server of the rosdyn_identification::MetoParEstimAction.
    */
    std::shared_ptr<actionlib::SimpleActionServer<rosdyn_identification_msgs::MetoParEstimAction>> m_meto_par_estim_as;
   
    
    //! Main function to enter in an infinite loop.
    /*!
      
    */
    void main();
    
    
    bool saveXmlCallback( std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

    //! Callback of the rosdyn_identification::MetoParEstimAction.
    /*!
      \param goal goal of the rosdyn_identification::MetoParEstimAction
    */ 
    void metoParEstimCB(  const rosdyn_identification_msgs::MetoParEstimGoalConstPtr& goal );
    
    //! A destructor of the class \link MetoParEstimInterfaceNodelet \endlink.
    /*!
    */
    ~MetoParEstimInterfaceNodelet();
    
  };

}

# endif