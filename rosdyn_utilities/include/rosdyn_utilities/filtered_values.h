#ifndef ROSDYN_UTILITIES__FILTERED_VALUES__H
#define ROSDYN_UTILITIES__FILTERED_VALUES__H

#include <memory>
#include <rosdyn_core/spacevect_algebra.h>
#include <Eigen/Core>
#include <eigen_state_space_systems/eigen_common_filters.h>

namespace rosdyn
{

/**
 * @class FilteredValue
 * @brief The class is a wrap for the templated-value and the filter provied bu eigen_common_filter. 
 * The template is designed in order to use as raw data std::vector or Eigen::VectorXd, or static dimensioned Eigen::Matrix<double,N,1>
 * The specialization for Eigen::VectorXd, Eigen:Matrix<double,6,1>, and Eigen:Matrix<double,7,1> is provided.
 */
template<int N>
class FilteredValue
{
protected:
  bool filter_active_;
  double natural_frequency_;
  double sampling_time_;

  typedef Eigen::Matrix<double,N,1> Vector;
  enum { NeedsToAlign = (sizeof(Vector)%16)==0 };

  Eigen::Matrix<double,N,1> raw_values_;
  Eigen::Matrix<double,N,1> banded_values_;
  Eigen::Matrix<double,N,1> values_;
  Eigen::Matrix<double,N,1> dead_band_;
  Eigen::Matrix<double,N,1> saturation_;

  std::vector<std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass>> lpf_; 


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

  typedef  std::shared_ptr<FilteredValue> Ptr;
  typedef  std::shared_ptr<FilteredValue const> ConstPtr;

  FilteredValue();
  virtual ~FilteredValue() = default;
  FilteredValue(const FilteredValue&);
  FilteredValue& operator=(const FilteredValue&);
  FilteredValue(FilteredValue&&) = default;
  FilteredValue& operator=(FilteredValue&&) = default;

  void activateFilter ( const Eigen::Matrix<double,N,1>& dead_band
                      , const Eigen::Matrix<double,N,1>& saturation
                      , const double natural_frequency
                      , const double sampling_time
                      , const Eigen::Matrix<double,N,1>& init_value );

  void deactivateFilter(  );

  const Eigen::Matrix<double,N,1>& value() const;
  Eigen::Matrix<double,N,1>& value();

  FilteredValue<N>& update(const Eigen::Matrix<double,N,1>& new_values);

  const double& value(const size_t iAx) const;
  double&       value(const size_t iAx);
  
  const double* data(const size_t iAx) const;
  double* data(const size_t iAx);
  
  const Eigen::Matrix<double,N,1>& raw() const;
  Eigen::Matrix<double,N,1>& raw();
};


/**
 * 
 * @class FilteredValue<-1> 
 * @brief Specialization to VectorXd 
 * 
 */
template<>
class FilteredValue<-1>
{
protected:
  bool filter_active_;
  double natural_frequency_;
  double sampling_time_;

  typedef Eigen::VectorXd Vector;
  enum { NeedsToAlign = (sizeof(Vector)%16)==0 };

  Eigen::VectorXd raw_values_;
  Eigen::VectorXd banded_values_;
  Eigen::VectorXd values_;
  Eigen::VectorXd dead_band_;
  Eigen::VectorXd saturation_;

  std::vector<std::shared_ptr<eigen_control_toolbox::FirstOrderLowPass>> lpf_; 


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

  typedef  std::shared_ptr<FilteredValue> Ptr;
  typedef  std::shared_ptr<FilteredValue const> ConstPtr;

  FilteredValue(): filter_active_(false), natural_frequency_(0.0), sampling_time_(0.0)
  {
  }
  virtual ~FilteredValue() = default;
  FilteredValue(const FilteredValue&);
  FilteredValue& operator=(const FilteredValue&);
  FilteredValue(FilteredValue&&) = default;
  FilteredValue& operator=(FilteredValue&&) = default;

  void activateFilter ( const Eigen::VectorXd& dead_band
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


  void deactivateFilter(  );
  
  const Eigen::VectorXd& value() const;
  Eigen::VectorXd& value();

  double value(const size_t iAx) const;
  double& value(const size_t iAx);

  Eigen::VectorXd& update(const Eigen::VectorXd& new_values);
  
  const double* data(const size_t iAx) const;
  double* data(const size_t iAx);
  
  const Eigen::VectorXd& raw() const;
  Eigen::VectorXd& raw();

  virtual void resize(size_t nAx)
  {
    filter_active_ = false;
    values_.resize(nAx);
    raw_values_.resize(nAx);
    banded_values_.resize(nAx);
  }
};
using FilteredValueX = FilteredValue<-1>; 
typedef std::shared_ptr<FilteredValueX> FilteredValueXPtr;
typedef std::shared_ptr<FilteredValueX const> FilteredValueXConstPtr;

/**
 * Pre-allocated classes with dimension already defined
 */
#define DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(nAx)\
using  FilteredValue ## nAx = FilteredValue<nAx>;\
typedef std::shared_ptr<FilteredValue<nAx>      > FilteredValue ## nAx ## Ptr;\
typedef std::shared_ptr<FilteredValue<nAx> const> FilteredValue ## nAx ## ConstPtr;\

DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(1); // FilteredValue1, FilteredValue1Ptr, FilteredValue1ConstPtr
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(2)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(3)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(4)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(5)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(6)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(7)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(9)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(10)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(11)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(12)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(13)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(14)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(15)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(16)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(17)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(19)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(20)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(21)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(22)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(23)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(24)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(25)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(26)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(27)
DEFINE_FILTEREDVALUE_STATIC_DIMENSION_PTR(29)


} // namespace rosdyn

#include <rosdyn_utilities/internal/filtered_values_impl.h>

#endif
