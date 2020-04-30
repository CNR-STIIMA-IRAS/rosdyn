#ifndef rosdyn_gui_201811200841
#define rosdyn_gui_201811200841

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>


#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <rosdyn_identification_msgs/MetoParEstimAction.h>
#include <rosdyn_identification_msgs/MetoTrjGenAction.h>
#include <moveit_planning_helper/ExecuteTrajectoryFromParamAction.h>

#include "rviz/panel.h"
#include "rviz/visualization_manager.h"

#include <QVBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QGroupBox>
#include <QComboBox>
#include <QDialog>
#include <setup_popup.h>
#include <dyn_components.h>
#include <results_popup.h>

// class QPushButton;
namespace rosdyn_gui
{
  

  
  class IdentificationGui: public rviz::Panel
  {
    
    Q_OBJECT
  public:
    
    IdentificationGui( QWidget* parent = 0 );
    virtual ~IdentificationGui();
    virtual void onInitialize();    
    /** @brief Load the given html file. */
    
    
    protected Q_SLOTS:
      
      void callback(unsigned int num_btn);
      
      void generateTrjCallback();
      void saveTrjCallback();
      void loadTrjCallback();
      
      void simulateTrjCallback();
      void executeTrjCallback();
      void newTrajectory(QString select_trj);
      
      void modelEstimationCallback();
      void saveModelCallback();
      void setupCallback();
      
  protected:  
    ros::NodeHandle m_nh;
    
    QGridLayout* m_grid_layout;
    
    QPushButton* m_generate_trajectories_btn;
    QPushButton* m_save_trj_btn;
    QPushButton* m_load_trj_btn;
    
    QPushButton* m_simulate_trajectories_btn;
    QPushButton* m_execute_trajectories_btn;
    QComboBox*   m_list_trjs_combobox;
    
    QPushButton* m_parameter_estimation_btn;
    QPushButton* m_save_model_btn;
    QPushButton* m_setup_btn;
    TabDialog* m_setup_popup;
    ResultsTabDialog* m_results_popup;

    std::shared_ptr<actionlib::SimpleActionClient<rosdyn_identification_msgs::MetoTrjGenAction>> meto_gen_ac_;
    std::shared_ptr<actionlib::SimpleActionClient<moveit_planning_helper::ExecuteTrajectoryFromParamAction>> meto_exec_ac_;
    std::shared_ptr<actionlib::SimpleActionClient<rosdyn_identification_msgs::MetoParEstimAction>> meto_estim_ac_;
    ros::ServiceClient m_save_model_client;
    QLabel* trj_label;
    
    void updateTrajectoriesList();
    void generationDoneCb(const actionlib::SimpleClientGoalState& state, const rosdyn_identification_msgs::MetoTrjGenResultConstPtr& result);
    void executionDoneCb(const actionlib::SimpleClientGoalState& state, const moveit_planning_helper::ExecuteTrajectoryFromParamResultConstPtr& result);
    void estimationDoneCb(const actionlib::SimpleClientGoalState& state, const rosdyn_identification_msgs::MetoParEstimResultConstPtr& result);
    
    
  };
  
}

#endif
