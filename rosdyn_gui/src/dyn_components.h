#ifndef rosdyn_gui_dynamic_component_201811200841
#define rosdyn_gui_dynamic_component_201811200841

#include <ros/ros.h>
#include <QVBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QGroupBox>
#include <QComboBox>
#include <QDialog>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QSpinBox>

namespace rosdyn_gui
{
  
  class FrictionWidgets : public QWidget
  {
    Q_OBJECT
    
  public:
    explicit FrictionWidgets(ros::NodeHandle& nh, const std::string& joint_name, QWidget *parent = 0);
    
    int checkedFriction();
    std::string jointName(){return m_joint_name;};
    void saveParam();
    void loadParam();
  protected Q_SLOTS:
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

  
}


#endif