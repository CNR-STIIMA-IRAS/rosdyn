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
#include <rosdyn_utilities/filtered_values.h>
#include <rosdyn_utilities/chain_interface.h>
#include <rosdyn_utilities/chain_state.h>
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, FilteredValue)
{

  EXPECT_NO_FATAL_FAILURE(rosdyn::FilteredValue<1> fv) << "Ctor";
  constexpr size_t N = 1;
  rosdyn::FilteredValue<N> fv1;
  Eigen::Matrix<double, N,1>  d;    d <<  9;
  Eigen::Matrix<double,-1,1> x(1); x << -1;
  Eigen::Matrix<double,-1,1> y(2); y << 1,2; 

  EXPECT_NO_FATAL_FAILURE( fv1.value() = d );
  EXPECT_TRUE( fv1.value().norm() == d.norm() );

  EXPECT_NO_FATAL_FAILURE( fv1.value() = x );
  EXPECT_TRUE( (fv1.value()- x).norm() == 0 );
  
  EXPECT_NO_FATAL_FAILURE( fv1.value() = y );
  EXPECT_TRUE( fv1.value().rows() == N );
  EXPECT_TRUE( fv1.value().rows() != y.rows() );
  
  EXPECT_NO_FATAL_FAILURE( fv1.activateFilter ( d, d, 0.1, 1e-3, d) );
  EXPECT_ANY_THROW( fv1.value() = d );
  
  EXPECT_NO_FATAL_FAILURE( fv1.deactivateFilter(  ) );
  
  EXPECT_NO_FATAL_FAILURE( fv1.value() = d );
  EXPECT_TRUE( fv1.value().norm() == d.norm() );

  EXPECT_NO_FATAL_FAILURE( d = fv1.value() );
  EXPECT_TRUE( fv1.value().norm() == d.norm() );
  
  double a = 0.0;
  EXPECT_NO_FATAL_FAILURE( a = fv1.value(0) );
  EXPECT_TRUE( fv1.value(0) == a );
  
  a = 99.0;
  EXPECT_NO_FATAL_FAILURE( fv1.value(0) = a );
  EXPECT_TRUE( fv1.value(0) == a );

  EXPECT_NO_FATAL_FAILURE( fv1.update(d) );
  EXPECT_TRUE( fv1.value().norm() == d.norm() );

  double* ptr = nullptr;
  EXPECT_NO_FATAL_FAILURE( ptr = fv1.data() );
  EXPECT_ANY_THROW( ptr = fv1.data(1) );

  constexpr double vv = 1234.0;
  *ptr = vv;

  EXPECT_TRUE( fv1.value(0) == vv ); 
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
