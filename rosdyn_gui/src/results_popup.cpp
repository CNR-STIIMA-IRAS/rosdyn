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

#include <results_popup.h>
#include <dyn_components.h>

#include <string>
#include <vector>

namespace rosdyn_gui
{

ResultsDialog::ResultsDialog(ros::NodeHandle& nh,
                             const rosdyn_identification_msgs::MetoParEstimResultConstPtr& result,
                             const unsigned int iax,
                             QWidget* parent):
  m_nh(nh)/*, QWidget(parent)*/
{
  std::vector<std::string> js = result->identification.joint_names;
  std::vector<double> ident_real_model_correlation = result->identification.real_model_correlation;
  std::vector<double> ident_rms_error = result->identification.rms_error;

  std::vector<double> vali_real_model_correlation = result->validation.real_model_correlation;
  std::vector<double> vali_rms_error = result->validation.rms_error;

  m_grid_layout = new QGridLayout(this);

  int ir = 0;
//  m_grid_layout->addWidget( new QLabel(tr(js.at(iax).c_str())),        ir++, 0, 1, 2);
  m_grid_layout->addWidget(new QLabel(tr("IDENTIFICATION")),        ir++, 0, 1, 2);

  m_grid_layout->addWidget(new QLabel(tr("Real/Model correlation")),        ir, 0, 1, 1);
  QProgressBar* ident_slider = new QProgressBar(this);
  ident_slider->setRange(0, 1);
  ident_slider->setValue(ident_rms_error.at(iax));
  m_grid_layout->addWidget(ident_slider,        ir, 1, 1, 1);
  ir++;
  m_grid_layout->addWidget(new QLabel(tr("RMS Error")),        ir, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(QString::number(ident_rms_error.at(iax))),        ir, 1, 1, 1);
  ir++;

  m_grid_layout->addWidget(new QLabel(tr("Validation")),        ir++, 0, 1, 2);
  m_grid_layout->addWidget(new QLabel(tr("Real/Model correlation")),        ir, 0, 1, 1);
  QProgressBar* valid_slider = new QProgressBar(this);
  valid_slider->setRange(0, 1);
  valid_slider->setValue(vali_rms_error.at(iax));
  m_grid_layout->addWidget(valid_slider,        ir, 1, 1, 1);
  ir++;
  m_grid_layout->addWidget(new QLabel(tr("RMS Error")),        ir, 0, 1, 1);
  m_grid_layout->addWidget(new QLabel(QString::number(vali_rms_error.at(iax))),        ir, 1, 1, 1);
  ir++;
}

ResultsTabDialog::ResultsTabDialog(ros::NodeHandle &nh,
                                   const rosdyn_identification_msgs::MetoParEstimResultConstPtr &result,
                                   QWidget *parent)
  : /*QDialog(parent), */m_nh(nh)
{
  std::vector<std::string> js = result->identification.joint_names;


  m_tabWidget = new QTabWidget(this);

  for (unsigned int iax = 0; iax < js.size(); iax++)
  {
    m_tabs.push_back(new ResultsDialog(nh, result, iax, this));
    m_tabs.back()->adjustSize();
    m_tabWidget ->addTab(m_tabs.back(), tr(js.at(iax).c_str()));
  }
  m_tabWidget->adjustSize();
}


}  // namespace rosdyn_gui
