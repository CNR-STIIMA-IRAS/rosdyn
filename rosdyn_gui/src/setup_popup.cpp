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
#include <setup_popup.h>
#include <dyn_components.h>

namespace rosdyn_gui
{

TabDialog::TabDialog(ros::NodeHandle& nh, QWidget* parent): QDialog(parent), m_nh(nh)
{
  m_tabWidget = new QTabWidget(this);
  m_tabWidget ->addTab(new GenerationTab(nh, this), tr("General"));
  m_tabWidget ->addTab(new ModelTab(nh, this), tr("Friction"));
  m_tabWidget->adjustSize();
}



GenerationTab::GenerationTab(ros::NodeHandle& nh, QWidget* parent): QWidget(parent), m_nh(nh)
{
  m_grid_layout = new QGridLayout(this);
  m_grid_layout->addWidget(new QLabel(tr("Duration of Stage 1")),        0, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(tr("Number of seeds (Stage 2)")),  1, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(tr("Points per seed  (Stage 2)")), 2, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(tr("Number of trial")),            3, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(tr("Filtering frequency")),        4, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(tr("Sampling period")),            5, 0, 1, 1);

  m_ok_btn = new QPushButton();
  m_ok_btn->setText("Store");
  connect(m_ok_btn, SIGNAL(clicked()), this, SLOT(saveNewPar()));
  m_grid_layout->addWidget(m_ok_btn, 6, 0, 1, 2);


  QDoubleSpinBox* duration_stage_1 = new QDoubleSpinBox();

  if (m_nh.getParam("meto_cfg/opt_cfg/stage1_duration", m_stage_duration))
  {
    duration_stage_1->setValue(m_stage_duration);
  }
  else
  {
    m_stage_duration = 60;
    duration_stage_1->setValue(60);
    m_nh.setParam("meto_cfg/opt_cfg/stage1_duration", 60);
  }
  connect(duration_stage_1, SIGNAL(valueChanged(double)), this, SLOT(changedDuration(double)));
  m_grid_layout->addWidget(duration_stage_1, 0, 1, 1, 1);

  QSpinBox* point_stage_2 = new QSpinBox();
  m_grid_layout->addWidget(point_stage_2, 1, 1, 1, 1);
  if (m_nh.getParam("meto_cfg/opt_cfg/region_stage2", m_regione_stage2))
  {
    point_stage_2->setValue(m_regione_stage2);
  }
  else
  {
    m_regione_stage2 = 0;
    point_stage_2->setValue(0);
    m_nh.setParam("meto_cfg/opt_cfg/region_stage2", 0);
  }
  connect(point_stage_2, SIGNAL(valueChanged(int)), this, SLOT(changedStage2(int)));


  QSpinBox* point_per_region = new QSpinBox();
  m_grid_layout->addWidget(point_per_region, 2, 1, 1, 1);
  if (m_nh.getParam("meto_cfg/opt_cfg/point_per_region", m_point_per_region))
  {
    point_per_region->setValue(m_point_per_region);
  }
  else
  {
    m_point_per_region = 0;
    point_per_region->setValue(0);
    m_nh.setParam("meto_cfg/opt_cfg/point_per_region", 0);
  }
  connect(point_per_region, SIGNAL(valueChanged(int)), this, SLOT(changedPointStage2(int)));


  QSpinBox* n_trial = new QSpinBox();
  m_grid_layout->addWidget(n_trial, 3, 1, 1, 1);
  if (m_nh.getParam("meto_cfg/opt_cfg/trials", m_trials))
  {
    n_trial->setValue(m_trials);
  }
  else
  {
    m_trials = 0;
    n_trial->setValue(0);
    m_nh.setParam("meto_cfg/opt_cfg/trials", 0);
  }
  connect(n_trial, SIGNAL(valueChanged(int)), this, SLOT(changeTrialNumber(int)));

  QDoubleSpinBox* filtering_freq = new QDoubleSpinBox();
  m_grid_layout->addWidget(filtering_freq, 4, 1, 1, 1);
  if (m_nh.getParam("meto_cfg/filter/frequency", m_frequency))
  {
    filtering_freq->setValue(m_frequency);
  }
  else
  {
    m_frequency = 10;
    filtering_freq->setValue(m_frequency);
    m_nh.setParam("meto_cfg/filter/frequency", m_frequency);
  }
  connect(filtering_freq, SIGNAL(valueChanged(double)), this, SLOT(changeFrequency(double)));

  QDoubleSpinBox* sampling_period = new QDoubleSpinBox();
  sampling_period->setDecimals(4);
  m_grid_layout->addWidget(sampling_period, 5, 1, 1, 1);
  if (m_nh.getParam("meto_cfg/filter/sample_period", m_sampling_period))
  {
    sampling_period->setValue(m_sampling_period);
  }
  else
  {
    m_sampling_period = 10;
    sampling_period->setValue(m_sampling_period);
    m_nh.setParam("meto_cfg/filter/sample_period", m_sampling_period);
  }
  connect(sampling_period, SIGNAL(valueChanged(double)), this, SLOT(changeSamplingPeriod(double)));
}


void GenerationTab::saveNewPar()
{
  m_nh.setParam("meto_cfg/opt_cfg/stage1_duration", m_stage_duration);
  m_nh.setParam("meto_cfg/opt_cfg/region_stage2", m_regione_stage2);
  m_nh.setParam("meto_cfg/opt_cfg/point_per_region", m_point_per_region);
  m_nh.setParam("meto_cfg/opt_cfg/trials", m_trials);
  m_nh.setParam("meto_cfg/filter/frequency", m_frequency);
  m_nh.setParam("meto_cfg/filter/sample_period", m_sampling_period);
}


void GenerationTab::changeFrequency(double value)
{
  m_frequency = value;
}

void GenerationTab::changeSamplingPeriod(double value)
{
  m_sampling_period = value;
}


void GenerationTab::changedDuration(double value)
{
  m_stage_duration = value;
}

void GenerationTab::changedStage2(int number)
{
  m_regione_stage2 = number;
}

void GenerationTab::changedPointStage2(int number)
{
  m_point_per_region = number;
}

void GenerationTab::changeTrialNumber(int number)
{
  m_trials = number;
}



}  // namespace rosdyn_gui
