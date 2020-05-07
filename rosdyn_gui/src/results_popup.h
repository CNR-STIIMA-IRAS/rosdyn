#ifndef rosdyn_gui_results_popup_20190405
#define rosdyn_gui_results_popup_20190405

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
#include <QProgressBar>
#include <rosdyn_identification_msgs/MetoParEstimActionResult.h>

namespace rosdyn_gui
{
  class ResultsDialog : public QWidget
  {
    Q_OBJECT

  public:
    explicit ResultsDialog(ros::NodeHandle& nh,
                            const rosdyn_identification_msgs::MetoParEstimResultConstPtr& result,
                            const unsigned int iax,
                            QWidget *parent = 0);

  private:
    QGridLayout* m_grid_layout;
    ros::NodeHandle m_nh;

  };

  class ResultsTabDialog : public QDialog
  {
    Q_OBJECT

  public:
    explicit ResultsTabDialog(ros::NodeHandle& nh,
                            const rosdyn_identification_msgs::MetoParEstimResultConstPtr& result,
                            QWidget *parent = 0);

  private:
    QTabWidget* m_tabWidget;
    std::vector<ResultsDialog*> m_tabs;
    ros::NodeHandle m_nh;

  };

}


#endif
