#ifndef ROSDYN_UTILITIES__FILTERED_VALUES__IMPL__H
#define ROSDYN_UTILITIES__FILTERED_VALUES__IMPL__H

#include <memory>
#include <Eigen/Core>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/eigen_common_filters.h>

#include <rosdyn_core/spacevect_algebra.h>

#include <rosdyn_utilities/filtered_values.h>

namespace rosdyn
{

template<int N, int MaxN>
FilteredValue<N,MaxN>::FilteredValue() 
: filter_active_(false), natural_frequency_(0.0), sampling_time_(0.0)
{
}


template<int N, int MaxN>
FilteredValue<N,MaxN>::FilteredValue(const FilteredValue& cpy)
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


template<int N, int MaxN>
FilteredValue<N,MaxN>& FilteredValue<N,MaxN>::operator=(const FilteredValue<N,MaxN>& rhs)
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

template<int N, int MaxN>
bool FilteredValue<N,MaxN>::activateFilter( const FilteredValue<N,MaxN>::Value& dead_band
                                          , const FilteredValue<N,MaxN>::Value& saturation
                                          , const double natural_frequency
                                          , const double sampling_time
                                          , const FilteredValue<N,MaxN>::Value& init_value )
{
  if((eigen_utils::rows(dead_band) != eigen_utils::rows(saturation))
  || (eigen_utils::rows(dead_band) != eigen_utils::rows(init_value)))
  {
    return false;
  }

  if( (N>0) && (eigen_utils::rows(values_) != eigen_utils::rows(dead_band)))
  {
    return false;
  }

  values_ = init_value;
  bonded_value_ = values_;
  raw_values_ = values_;
  dead_band_ = dead_band;
  saturation_ = saturation;
  natural_frequency_ = natural_frequency;
  sampling_time_ = sampling_time;
  filter_active_ = true;

  lpf_.init( natural_frequency_,sampling_time_, eigen_utils::rows(init_value) );
  lpf_.setStateFromLastIO(init_value, init_value);

  return true;
}



template<int N, int MaxN>
void FilteredValue<N,MaxN>::deactivateFilter(  )
{
  filter_active_ = false;
}

template<int N, int MaxN>
const typename FilteredValue<N,MaxN>::Value& FilteredValue<N,MaxN>::value() const
{
  if(filter_active_)
  {
    throw std::runtime_error("The filter has been activated. The direct access be performed. Function getUpdatedValue() should be used.");
  }
  return values_;
}

template<int N, int MaxN>
typename FilteredValue<N,MaxN>::Value& FilteredValue<N,MaxN>::value()
{
  if(filter_active_)
  {
    throw std::runtime_error("The filter has been activated. The direct assignement cannot be performed. Function update() should be used.");
  }
  return values_;
}

template<int N, int MaxN>
const typename FilteredValue<N,MaxN>::Value& FilteredValue<N,MaxN>::getUpdatedValue() const
{
  if(!filter_active_)
  {
    throw std::runtime_error("The filter is not active. The access must be done through value().");
  }
  return values_;
}


template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
const double& FilteredValue<N,MaxN>::value(const int iAx) const
{
  return values_(iAx);
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
double& FilteredValue<N,MaxN>::value(const int iAx)
{
  double* ptr_to_val = values_.data() + iAx;
  return *ptr_to_val;
}

template<int N, int MaxN>
FilteredValue<N,MaxN>& FilteredValue<N,MaxN>::update(const FilteredValue<N,MaxN>::Value& new_values)
{
  if(!filter_active_)
  {
    throw std::runtime_error("The filter is not active. The assignement must be done through value().");
  }
  assert( eigen_utils::rows(values_) == eigen_utils::rows(new_values) );
  if(filter_active_)
  {
    bonded_value_ = new_values;
    for(int i = 0; i<eigen_utils::rows(new_values); i++)
    {
      double& b = eigen_utils::at(bonded_value_,i,0);
      double  d = eigen_utils::at(dead_band_,i,0);
      double  n = eigen_utils::at(new_values,i,0);
      double  s = eigen_utils::at(saturation_,i,0);
      b = n >  d ?  n - d 
        : n < -d ?  n + d
        : 0;

      b = b >  s ?  s
        : b < -s ? -s
        : b;
    }
    values_ = lpf_.update(bonded_value_);
  }
  else
  {
    values_ = new_values;
  }
  raw_values_ = new_values;
  return *this;
}

template<int N, int MaxN>
const typename FilteredValue<N,MaxN>::Value& FilteredValue<N,MaxN>::raw() const
{
  return raw_values_;
}

template<int N, int MaxN>
typename FilteredValue<N,MaxN>::Value& FilteredValue<N,MaxN>::raw() 
{ 
  return raw_values_; 
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
const double* FilteredValue<N,MaxN>::data() const
{
  return values_.data();
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n==1, int>::type>
const double* FilteredValue<N,MaxN>::data() const
{
  return &values_;
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
double* FilteredValue<N,MaxN>::data( )
{
 return values_.data();
}


template<int N, int MaxN>
template <int n, typename std::enable_if<n==1, int>::type>
double* FilteredValue<N,MaxN>::data( )
{
 return &values_;
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
const double* FilteredValue<N,MaxN>::data(const int iAx) const
{
  if(iAx >= eigen_utils::rows(values_) )
  {
    throw std::runtime_error("Index out of range");
  }
  return values_.data() + iAx;
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
double* FilteredValue<N,MaxN>::data(const int iAx)
{
  if(iAx >= eigen_utils::rows(values_) )
  {
    throw std::runtime_error("Index out of range");
  }
  return values_.data() + iAx;
}

template<int N, int MaxN>
template <int n, typename std::enable_if<n!=1, int>::type>
inline bool FilteredValue<N,MaxN>::resize(int nAx)
{
  filter_active_ = false;
  if(!eigen_utils::resize(values_, nAx,1))
    return false;
  
  if(!eigen_utils::resize(raw_values_, nAx,1))
    return false;
  
  if(!eigen_utils::resize(bonded_value_, nAx,1))
    return false;
  
  if(!eigen_utils::resize(dead_band_, nAx,1))
    return false;
  
  if(!eigen_utils::resize(saturation_, nAx,1))
    return false;

  return true;
}




  
} // namespace rosdyn


#endif
