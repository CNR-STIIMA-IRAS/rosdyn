/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <ros/ros.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <rosdyn_utilities/filtered_values.h>
#include <rosdyn_utilities/chain_interface.h>
#include <rosdyn_utilities/chain_state.h>
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, FilteredScalar)
{

  EXPECT_NO_FATAL_FAILURE(rosdyn::FilteredScalar fv1) << "Ctor";
  
  constexpr size_t N = 1;
  rosdyn::FilteredScalar fv1;
  double  d = 9;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(2); y << 1,2; 

  EXPECT_NO_FATAL_FAILURE( fv1.value() = d );
  EXPECT_NO_FATAL_FAILURE( d = fv1.value() );
  EXPECT_TRUE( eigen_utils::norm(fv1.value()) == std::fabs(d) );

  // These will break at compile time 
  // EXPECT_FATAL_FAILURE( fv1.value() = x );
  // EXPECT_ANY_THROW( fv1.value() = y )<< "The allocation is static, and the dimensions mismatches";
  
  // === ACTIVATED FILTER
  EXPECT_NO_FATAL_FAILURE( fv1.activateFilter ( d, d, 0.1, 1e-3, d) );
  EXPECT_ANY_THROW( fv1.value() = d ) << "The filter is activated, you must call, update";
  EXPECT_NO_FATAL_FAILURE( fv1.update(d) );
  
  // === DEACTIVATED FILTER
  EXPECT_NO_FATAL_FAILURE( fv1.deactivateFilter(  ) );
  EXPECT_NO_FATAL_FAILURE( fv1.value() = d );
  EXPECT_ANY_THROW( fv1.update(d) );
  
  // ACCESSOR
  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv1.data() );
  
  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv1.value() == vv ); 
}

// Declare a test
TEST(TestSuite, FilteredVectorXd)
{
  EXPECT_NO_FATAL_FAILURE(rosdyn::FilteredVectorXd fv) << "Ctor";
  
  rosdyn::FilteredVectorXd fv;
  Eigen::Matrix<double, 1,1> d;    d <<  9;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(2); y << 1,2; 

  // DIMENSION ARE NOT CHECKED, BECAUSE OF DYNAMIC ALLOCATION
  EXPECT_NO_FATAL_FAILURE( fv.value() = d ) << "Dynamic resize";
  EXPECT_TRUE( fv.value().norm() == d.norm() );
  EXPECT_TRUE( fv.value().rows() == d.rows() );

  EXPECT_NO_FATAL_FAILURE( fv.value() = x ) << "Dynamic resize";
  EXPECT_TRUE( (fv.value()- x).norm() == 0 );
  EXPECT_TRUE( fv.value().rows() == x.rows() );
  
  EXPECT_NO_FATAL_FAILURE( fv.value() = y );
  EXPECT_TRUE( fv.value().rows() == y.rows() );
  
  // ======================
  EXPECT_NO_FATAL_FAILURE( fv.activateFilter ( d, d, 0.1, 1e-3, d) );
  
  // if filter activated, you cannot direct assign the value, 
  // you must use "update"
  EXPECT_ANY_THROW( fv.value() = d ); 
  // Specifically, the value() method throw an exception when the filter is activated 
  EXPECT_ANY_THROW( int rows = fv.value().rows()) << "Getting the row() is not a const operator, sigh";

  EXPECT_NO_FATAL_FAILURE( fv.update(d) );
  EXPECT_NO_FATAL_FAILURE( d = fv.getUpdatedValue( ) );
  // ======================

  // ======================
  EXPECT_NO_FATAL_FAILURE( fv.deactivateFilter(  ) );
  EXPECT_NO_FATAL_FAILURE( fv.value() = d );
  EXPECT_ANY_THROW( fv.update(d) );
  EXPECT_ANY_THROW( d = fv.getUpdatedValue( ) );
  EXPECT_TRUE( fv.value().norm() == d.norm() );
  // ======================
  
  // ACCESSORS
  // ======================
  EXPECT_NO_FATAL_FAILURE( d = fv.value() );
  EXPECT_TRUE( fv.value().norm() == d.norm() );
  
  double a = 0.0;
  EXPECT_NO_FATAL_FAILURE( a = fv.value(0) );
  EXPECT_TRUE( fv.value(0) == a );
  
  a = 99.0;
  EXPECT_NO_FATAL_FAILURE( fv.value(0) = a );
  EXPECT_TRUE( fv.value(0) == a );

  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv.data() );
  EXPECT_ANY_THROW( ptr = fv.data(99) );

  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv.value(0) == vv ); 
  // ======================
  
}


// Declare a test
TEST(TestSuite, FilteredVector2d)
{

  EXPECT_NO_FATAL_FAILURE(rosdyn::FilteredVector2d fv2) << "Ctor";
  constexpr size_t N = 2;
  rosdyn::FilteredValue<N> fv2; // equivalent to rosdyn::FilteredVector1d
  Eigen::Matrix<double, N,1> d; d <<  9,10;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(3); y << 1,2,3; 

  Eigen::Matrix<double, N,1> dd; dd <<  9,10;
  Eigen::Matrix<double,-1,1> yy(3); yy << 1,2,3; 

  // DIMENSION ARE CHECKED, BECAUSE OF STATIC ALLOCATION
  EXPECT_NO_FATAL_FAILURE( fv2.value() = d ) << "The Eigen underlay equality fail if dimensions are different";
  EXPECT_TRUE( fv2.value().norm() == d.norm() );

  EXPECT_NO_FATAL_FAILURE( fv2.value() = x );
  EXPECT_TRUE( (fv2.value()- x).norm() == 0 );
  
  EXPECT_NO_FATAL_FAILURE( fv2.value() = y ) << "The Eigen underlay equality fail if dimensions are different";
  EXPECT_TRUE( eigen_utils::rows(fv2.value()) == N );
  EXPECT_TRUE( eigen_utils::rows(fv2.value()) != y.rows() );
  EXPECT_TRUE( fv2.value().rows() == N );
  EXPECT_TRUE( fv2.value().rows() != y.rows() );
  
  EXPECT_NO_FATAL_FAILURE( fv2.activateFilter ( d, d, 0.1, 1e-3, d) );
  EXPECT_ANY_THROW( fv2.value() = d );
  
  EXPECT_NO_FATAL_FAILURE( fv2.deactivateFilter(  ) );
  
  EXPECT_NO_FATAL_FAILURE( fv2.value() = d );
  EXPECT_TRUE( fv2.value().norm() == d.norm() );

  EXPECT_NO_FATAL_FAILURE( d = fv2.value() );
  EXPECT_TRUE( fv2.value().norm() == d.norm() );
  
  double a = 0.0;
  EXPECT_NO_FATAL_FAILURE( a = fv2.value(0) );
  EXPECT_TRUE( fv2.value(0) == a );
  
  a = 99.0;
  EXPECT_NO_FATAL_FAILURE( fv2.value(0) = a );
  EXPECT_TRUE( fv2.value(0) == a );

  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv2.data() );
  EXPECT_ANY_THROW( ptr = fv2.data(6) );

  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv2.value(0) == vv ); 
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
