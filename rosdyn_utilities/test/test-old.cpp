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
#include <eigen_state_space_systems/filters/filtered_values.h>
#include <rosdyn_utilities/chain_interface.h>
#include <rosdyn_utilities/chain_state.h>
#include <gtest/gtest.h>

using ChainStateX = rosdyn::ChainState<-1, 20>;
using ChainStateXX = rosdyn::ChainState<-1>;
using ChainState1 = rosdyn::ChainState<1>;
using ChainState3 = rosdyn::ChainState<3>;
using ChainState6 = rosdyn::ChainState<6>;


std::shared_ptr<ChainStateX>  chainX;
std::shared_ptr<ChainStateXX> chainXX;
std::shared_ptr<ChainState1>  chain1;
std::shared_ptr<ChainState3>  chain3;
std::shared_ptr<ChainState6>  chain6;


// Declare a test
TEST(TestSuite, fullConstructor)
{
  EXPECT_NO_FATAL_FAILURE(chainX .reset(new ChainStateX ()) );
  EXPECT_NO_FATAL_FAILURE(chainXX.reset(new ChainStateXX()) );
  EXPECT_NO_FATAL_FAILURE(chain1 .reset(new ChainState1 ()) );
  EXPECT_NO_FATAL_FAILURE(chain3 .reset(new ChainState3 ()) );
  EXPECT_NO_FATAL_FAILURE(chain6 .reset(new ChainState6 ()) );
}


// Declare a test
TEST(TestSuite, desctructor)
{
  EXPECT_NO_FATAL_FAILURE(chainX  .reset() );
  EXPECT_NO_FATAL_FAILURE(chainXX .reset() );
  EXPECT_NO_FATAL_FAILURE(chain1  .reset() );
  EXPECT_NO_FATAL_FAILURE(chain3  .reset() );
  EXPECT_NO_FATAL_FAILURE(chain6  .reset() );
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
