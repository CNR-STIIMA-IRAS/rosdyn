#include <dyn_components.h>


namespace rosdyn_gui
{


DynComponents::DynComponents( ros::NodeHandle& nh,QWidget* parent): QWidget(parent ), m_nh(nh)
{
  m_grid_layout = new QGridLayout(this);
  m_grid_layout->addWidget( new QLabel(tr("Duration of Stage 1")),        0, 0, 1, 1);
  m_grid_layout->addWidget( new QLabel(tr("Number of seeds (Stage 2)")),  1, 0, 1, 1);
  m_grid_layout->addWidget( new QLabel(tr("Points per seed  (Stage 2)")), 2, 0, 1, 1);
  m_grid_layout->addWidget( new QLabel(tr("Number of trial")),            3, 0, 1, 1);
  
  QDoubleSpinBox* duration_stage_1= new QDoubleSpinBox();
  
  double duration;
  if (m_nh.getParam("meto_cfg/opt_cfg/stage1_duration",duration))
  {
    duration_stage_1->setValue(duration);
  }
  else 
  {
    duration_stage_1->setValue(60);
    m_nh.setParam("meto_cfg/opt_cfg/stage1_duration",60);
  }
  connect(duration_stage_1, SIGNAL(valueChanged(double)),this,SLOT(changedDuration(double)));
  m_grid_layout->addWidget( duration_stage_1,0, 1, 1, 1);
  
  QSpinBox* point_stage_2= new QSpinBox();
  m_grid_layout->addWidget( point_stage_2,1, 1, 1, 1);
  int region_stage2;
  if (m_nh.getParam("meto_cfg/opt_cfg/region_stage2",region_stage2))
  {
    point_stage_2->setValue(region_stage2);
  }
  else 
  {
    point_stage_2->setValue(0);
    m_nh.setParam("meto_cfg/opt_cfg/region_stage2",0);
  }
  connect(point_stage_2, SIGNAL(valueChanged(int)),this,SLOT(changedStage2(int)));
  
  
  QSpinBox* point_per_region= new QSpinBox();
  m_grid_layout->addWidget( point_per_region,2, 1, 1, 1);
  int ipoint_per_region;
  if (m_nh.getParam("meto_cfg/opt_cfg/point_per_region",ipoint_per_region))
  {
    point_per_region->setValue(ipoint_per_region);
  }
  else 
  {
    point_per_region->setValue(0);
    m_nh.setParam("meto_cfg/opt_cfg/point_per_region",0);
  }
  connect(point_per_region, SIGNAL(valueChanged(int)),this,SLOT(changedPointStage2(int)));
  
  
  QSpinBox* n_trial= new QSpinBox();
  m_grid_layout->addWidget( n_trial,3, 1, 1, 1);
  int ntrial;
  if (m_nh.getParam("meto_cfg/opt_cfg/trials",ntrial))
  {
    n_trial->setValue(ntrial);
  }
  else 
  {
    n_trial->setValue(0);
    m_nh.setParam("meto_cfg/opt_cfg/trials",0);
  }
  connect(n_trial, SIGNAL(valueChanged(int)),this,SLOT(changeTrialNumber(int)));
  
  m_ok_btn = new QPushButton();
  m_ok_btn->setText("Ok");
  connect(m_ok_btn, SIGNAL(clicked()), this, SLOT(saveNewPar()));
  m_grid_layout->addWidget( m_ok_btn,4, 0, 1, 1);

  m_cancel_btn = new QPushButton();
  m_cancel_btn->setText("Cancel");
  //connect(m_cancel_btn, SIGNAL(clicked()), this, SLOT(generateTrjCallback()));
  m_grid_layout->addWidget( m_cancel_btn,4, 1, 1, 1);
  
  //   m_layout->addRow(new QLabel(tr("Duration of Stage 1")), new QLineEdit);
//   m_layout->addRow(new QLabel(tr("Number of seeds of Stage 2")), new QLineEdit);
//   m_formGroupBox->setLayout(m_layout);
//   adjustSize();
}


void DynComponents::saveNewPar()
{
  m_nh.setParam("meto_cfg/opt_cfg/stage1_duration",m_stage_duration);
  m_nh.setParam("meto_cfg/opt_cfg/region_stage2",m_regione_stage2);
  m_nh.setParam("meto_cfg/opt_cfg/point_per_region",m_point_per_region);
  m_nh.setParam("meto_cfg/opt_cfg/trials",m_trials);
}

void DynComponents::changedDuration(double value)
{
  m_stage_duration=value;
}
void DynComponents::changedStage2(int number)
{
  m_regione_stage2=number;
}

void DynComponents::changedPointStage2(int number)
{
  m_point_per_region=number;
}

void DynComponents::changeTrialNumber(int number)
{
  m_trials=number;
}



}