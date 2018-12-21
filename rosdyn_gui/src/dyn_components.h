#ifndef rosdyn_gui_setup_popup_201811200841
#define rosdyn_gui_setup_popup_201811200841

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
  
  class DynComponents : public QWidget
  {
    Q_OBJECT
    
  public:
    explicit DynComponents(ros::NodeHandle& nh, QWidget *parent = 0);
    
    QGroupBox* m_formGroupBox;
    QGridLayout* m_grid_layout;
    ros::NodeHandle m_nh;
    
  protected Q_SLOTS:
    void changedDuration(double value);
    void changedStage2(int number);
    void changedPointStage2(int number);
    void changeTrialNumber(int number);
    void saveNewPar();
  protected:
    QPushButton* m_ok_btn;
    QPushButton* m_cancel_btn;
    double m_stage_duration;
    int m_regione_stage2;
    int m_point_per_region;
    int m_trials;
    
  };
  
  class TabDialog : public QDialog
  {
    Q_OBJECT
    
  public:
    explicit TabDialog( ros::NodeHandle& nh, QWidget *parent = 0);
    
  private:
    QTabWidget* m_tabWidget;
    ros::NodeHandle m_nh;
  };
  

}


#endif