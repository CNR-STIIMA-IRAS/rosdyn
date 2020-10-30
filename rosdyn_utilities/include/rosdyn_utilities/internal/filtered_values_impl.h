#ifndef ROSDYN_UTILITIES__FILTERED_VALUES__IMPL__H
#define ROSDYN_UTILITIES__FILTERED_VALUES__IMPL__H

#include <memory>
#include <rosdyn_core/spacevect_algebra.h>
#include <Eigen/Core>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <rosdyn_utilities/filtered_values.h>

namespace rosdyn
{

template<int N>
FilteredValue<N>::FilteredValue() 
: filter_active_(false), natural_frequency_(0.0), sampling_time_(0.0)
{
}

template<int N>
FilteredValue<N>::FilteredValue(const FilteredValue& cpy)
{
  filter_active_ = cpy.filter_active_;
  natural_frequency_ = cpy.natural_frequency_ ;
  sampling_time_ = cpy.sampling_time_;

  if(filter_active_)
  {
    activateFilter(cpy.raw_values_, cpy.saturation_, cpy.natural_frequency_, cpy.sampling_time_, cpy.values_);
  }
  else
  {
    values_ = cpy.values_;
  }
}

template<int N>
FilteredValue<N>& FilteredValue<N>::operator=(const FilteredValue& rhs)
{
  filter_active_ = rhs.filter_active_;
  natural_frequency_ = rhs.natural_frequency_ ;
  sampling_time_ = rhs.sampling_time_;

  if(filter_active_)
  {
    activateFilter(rhs.raw_values_, rhs.saturation_, rhs.natural_frequency_, rhs.sampling_time_, rhs.values_);
  }
  else
  {
    values_ = rhs.values_;
  }
  return *this;
}


template<int N>
void FilteredValue<N>::activateFilter ( const Eigen::Matrix<double,N,1>& dead_band
                                      , const Eigen::Matrix<double,N,1>& saturation
                                      , const double natural_frequency
                                      , const double sampling_time
                                      , const Eigen::Matrix<double,N,1>& init_value )
{
  values_ = init_value;
  banded_values_ = values_;
  raw_values_ = values_;
  dead_band_ = dead_band;
  saturation_ = saturation;
  natural_frequency_ = natural_frequency;
  sampling_time_ = sampling_time;
  filter_active_ = true;
  
  for(int i = 0; i < init_value.rows(); i++)
  {
    Eigen::VectorXd u(1); u(0) = init_value(i);
    std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> filt(
      new eigen_control_toolbox::FirstOrderLowPass( natural_frequency_,sampling_time_) );
    lpf_.push_back(filt);
    lpf_.back()->setStateFromIO(u,u);
  }
}

template<int N>
void FilteredValue<N>::deactivateFilter(  )
{
  filter_active_ = false;
}

template<int N>
const Eigen::Matrix<double,N,1>& FilteredValue<N>::value() const
{
  return values_;
}

template<int N>
Eigen::Matrix<double,N,1>& FilteredValue<N>::value()
{
  if(filter_active_)
  {
    throw std::runtime_error("The filter has been activated. The direct assignement cannot be performed. Function update() should be used.");
  }
  return values_;
}

template<int N>
const double& FilteredValue<N>::value(const size_t iAx) const
{
  return values_(iAx);
}

template<int N>
double& FilteredValue<N>::value(const size_t iAx)
{
  double* ptr_to_val = values_.data() + iAx;
  return *ptr_to_val;
}

template<int N>
FilteredValue<N>& FilteredValue<N>::update(const Eigen::Matrix<double,N,1>& new_values)
{
  assert( values_.rows() == new_values.rows() );
  if(filter_active_)
  {
    banded_values_ = new_values;
    for(int i = 0; i<new_values.size(); i++)
    {
      banded_values_[i] = new_values[i] >  dead_band_[i] ?  new_values[i] - dead_band_[i]
                        : new_values[i] < -dead_band_[i] ?  new_values[i] + dead_band_[i]
                        : 0;

      banded_values_[i] = banded_values_[i] >  saturation_[i] ?  saturation_[i]
                        : banded_values_[i] < -saturation_[i] ? -saturation_[i]
                        : banded_values_[i];
      
      values_[i] = lpf_[i]->update(banded_values_[i]);
    }
  }
  else
  {
    values_ = new_values;
  }
  raw_values_ = new_values;
  return *this;
}

template<int N>
const Eigen::Matrix<double,N,1>& FilteredValue<N>::raw() const
{
  return raw_values_;
}

template<int N>
Eigen::Matrix<double,N,1>& FilteredValue<N>::raw() 
{ 
  return raw_values_; 
}

template<int N>
const double* FilteredValue<N>::data(const size_t iAx) const
{
  if(iAx >= values_.rows() )
  {
    throw std::runtime_error("Index out of range");
  }
  return values_.data() + iAx;
}

template<int N>
double* FilteredValue<N>::data(const size_t iAx)
{
  if(iAx >= values_.rows() )
  {
    throw std::runtime_error("Index out of range");
  }
  return values_.data() + iAx;
}




/**
 * 
 * 
 * 
 * 
 */
inline
FilteredValue<-1>::FilteredValue()
  : filter_active_(false), natural_frequency_(0.0), sampling_time_(0.0)
{
}

inline 
void FilteredValue<-1>::activateFilter ( const Eigen::VectorXd& dead_band
                    , const Eigen::VectorXd& saturation
                    , const double natural_frequency
                    , const double sampling_time
                    , const Eigen::VectorXd& init_value )
{
  values_ = init_value;
  banded_values_ = values_;
  raw_values_ = values_;
  dead_band_ = dead_band;
  saturation_ = saturation;
  natural_frequency_ = natural_frequency;
  sampling_time_ = sampling_time;
  filter_active_ = true;
  
  for(int i = 0; i < init_value.rows(); i++)
  {
    Eigen::VectorXd u(1); u(0) = init_value(i);
    std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass> filt(
      new eigen_control_toolbox::FirstOrderLowPass( natural_frequency_,sampling_time_) );
    lpf_.push_back(filt);
    lpf_.back()->setStateFromIO(u,u);
  }
}

inline
void FilteredValue<-1>::resize(size_t nAx)
{
  filter_active_ = false;
  values_.resize(nAx);
  raw_values_.resize(nAx);
  banded_values_.resize(nAx);
}

inline
FilteredValue<-1>::FilteredValue(const FilteredValue<-1>& cpy)
{
  filter_active_ = cpy.filter_active_;
  natural_frequency_ = cpy.natural_frequency_ ;
  sampling_time_ = cpy.sampling_time_;

  if(filter_active_)
  {
    activateFilter(cpy.raw_values_, cpy.saturation_, cpy.natural_frequency_, cpy.sampling_time_, cpy.values_);
  }
  else
  {
    values_ = cpy.values_;
  }
}

inline
FilteredValue<-1>& FilteredValue<-1>::operator=(const FilteredValue<-1>& rhs)
{
  filter_active_ = rhs.filter_active_;
  natural_frequency_ = rhs.natural_frequency_ ;
  sampling_time_ = rhs.sampling_time_;

  if(filter_active_)
  {
    activateFilter(rhs.raw_values_, rhs.saturation_, rhs.natural_frequency_, rhs.sampling_time_, rhs.values_);
  }
  else
  {
    values_ = rhs.values_;
  }
  return *this;
}

inline
void FilteredValue<-1>::deactivateFilter(  )
{
  filter_active_ = false;
}

inline
const Eigen::VectorXd& FilteredValue<-1>::value() const
{
  return values_;
}

inline
Eigen::VectorXd& FilteredValue<-1>::value()
{
  if(filter_active_)
    throw std::runtime_error("The filter has been activated. The direct assignement cannot be performed. Function update() should be used.");
  return values_;
}

inline
double FilteredValue<-1>::value(const size_t iAx) const
{
  return values_(iAx);
}

inline
double& FilteredValue<-1>::value(const size_t iAx)
{
  double* ptr_to_val = values_.data() + iAx;
  return *ptr_to_val;
}

inline
Eigen::VectorXd& FilteredValue<-1>::update(const Eigen::VectorXd& new_values)
{
  assert( values_.rows() == new_values.rows() );
  if(filter_active_)
  {
    banded_values_ = new_values;
    for(int i = 0; i<new_values.size(); i++)
    {
      banded_values_[i] = new_values[i] >  dead_band_[i] ?  new_values[i] - dead_band_[i]
                        : new_values[i] < -dead_band_[i] ?  new_values[i] + dead_band_[i]
                        : 0;

      banded_values_[i] = banded_values_[i] >  saturation_[i] ?  saturation_[i]
                        : banded_values_[i] < -saturation_[i] ? -saturation_[i]
                        : banded_values_[i];
      
      values_[i] = lpf_[i]->update(banded_values_[i]);
    }
  }
  else
  {
    values_ = new_values;
  }
  raw_values_ = new_values;
  return filter_active_ ? values_ : raw_values_;
}

inline
const double* FilteredValue<-1>::data(const size_t iAx) const
{
  return values_.data() + iAx;
}

inline
double* FilteredValue<-1>::data(const size_t iAx)
{
  return values_.data() + iAx;
}
  

inline
const Eigen::VectorXd& FilteredValue<-1>::raw() const
{
  return raw_values_;
}

inline
Eigen::VectorXd& FilteredValue<-1>::raw() 
{ 
  return raw_values_; 
}



  
} // namespace rosdyn


#endif
