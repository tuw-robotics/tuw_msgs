// Copyright 2023 Markus Bader
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Markus Bader nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "gtest/gtest.h"
#include <tuw_msgs/node.hpp>
#include <tuw_msgs/edge.hpp>

TEST(Serialize, node_to_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q1);
  std::string str;
  {
    tuw_msgs::Node o(3, pose0);
    str = tuw_msgs::remove_spaces(o.to_str(tuw_msgs::Format::LOOSE));
    ASSERT_STREQ(str.c_str(), "node:3:[[0.000000,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]");
  }
  {
    tuw_msgs::Node o(3, pose0);
    str = tuw_msgs::remove_spaces(o.to_str(tuw_msgs::Format::COMPACT));
    ASSERT_STREQ(str.c_str(), "node:3:[[],[]]");
  }
}

TEST(Serialize, node_from_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q1);
  {
    tuw_msgs::Node o("node:3:[[0.000000,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]");
    ASSERT_EQ(o, tuw_msgs::Node(3, pose0));
  }

  {
    tuw_msgs::Node o("node:   3: [[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]");
    ASSERT_EQ(o, tuw_msgs::Node(3, pose0));
  }
}

TEST(Serialize, edge_to_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q0);
  tuw_msgs::Pose pose3(p1, q1);
  std::vector<tuw_msgs::Pose> path = {pose0, pose1, pose2, pose3};

  {
    tuw_msgs::Edge o(3);
    std::string str = o.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "edge: 3: invalid: undirected:   0.0000: [   0,    0]:[]"));
  }
  {
    tuw_msgs::Edge o(5, true, true, 0.2, 19, 22);
    std::string str = o.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "edge: 5: valid: directed: 0.2000: [19, 22]:[]"));
  }
  {
    tuw_msgs::Edge o(5, true, true, 0.2, 19, 22);
    o.set_path(path);
    std::string str = o.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "edge:    5:   valid:   directed:   0.2000: [  19,   22]: [[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[0.000000, 0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.300000, 2.200000, 1.100000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[3.300000, 2.200000, 1.100000], [0.413010, 0.197120, -0.753504, 0.472016]]]"));
  }
  {
    tuw_msgs::Edge o(5, true, true, 0.2, 19, 22);
    o.set_path(path);
    std::string str = o.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "edge:    5:   valid:   directed:   0.2000: [  19,   22]: [[[], []]; [[], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.300000, 2.200000, 1.100000], []]; [[3.300000, 2.200000, 1.100000], [0.413010, 0.197120, -0.753504, 0.472016]]]"));
  }
}

TEST(Serialize, edge_from_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q0);
  tuw_msgs::Pose pose3(p1, q1);
  std::vector<tuw_msgs::Pose> path = {pose0, pose1, pose2, pose3};

  {
    std::string src("edge: 5: valid: directed: 0.2000: [19, 22]: [[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.3, 2.2, 1.1], []]; [[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]]");
    tuw_msgs::Edge edge(5, true, true, 0.2, 19, 22);
    edge.set_path(path);
    tuw_msgs::Edge edge_src(src);
    bool test = edge_src.similar(edge);
    ASSERT_TRUE(test);
  }
}


TEST(Serialize, edge_to_and_from_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q0);
  tuw_msgs::Pose pose3(p1, q1);
  std::vector<tuw_msgs::Pose> path = {pose0, pose1, pose2, pose3};

  {
    tuw_msgs::Edge edge(5, true, true, 0.2, 19, 22, path);
    std::string src = edge.to_str(tuw_msgs::Format::COMPACT);
    tuw_msgs::Edge edge_src(src);
    bool test = edge_src.similar(edge);
    ASSERT_TRUE(test);
  }
  {
    tuw_msgs::Edge edge(5, true, true, 0.2, 19, 22, path);
    std::string src = edge.to_str(tuw_msgs::Format::LOOSE);
    tuw_msgs::Edge edge_src(src);
    bool test = edge_src.similar(edge);
    ASSERT_TRUE(test);
  }
}