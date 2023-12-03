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
#include <tuw_msgs/pose.hpp>

TEST(Serialize, equal)
{

  tuw_msgs::Point pa(3, 2, 1);
  tuw_msgs::Point pb(3, 2, 1);
  tuw_msgs::Point pc(3, 1, 1);
  ASSERT_EQ(pa, pb);
  ASSERT_NE(pa, pc);

  tuw_msgs::Quaternion qa(4, 3, 2, 1);
  tuw_msgs::Quaternion qb(4, 3, 2, 1);
  tuw_msgs::Quaternion qc(0, 3, 1, 1);
  ASSERT_EQ(qa, qb);
  ASSERT_NE(qa, qc);

  tuw_msgs::Pose pose_a(pa, qa);
  tuw_msgs::Pose pose_b(pb, qb);
  tuw_msgs::Pose pose_c(pb, qc);
  tuw_msgs::Pose pose_d(pc, qb);
  tuw_msgs::Pose pose_f(pc, qc);
  ASSERT_EQ(pose_a, pose_b);
  ASSERT_NE(pose_a, pose_c);
  ASSERT_NE(pose_a, pose_d);
  ASSERT_NE(pose_a, pose_f);
}

TEST(Serialize, point_to_string)
{
  std::string str;
  {
    tuw_msgs::Point src(3.3, 221.2, 5.3);
    str = src.to_str();
    ASSERT_STREQ(str.c_str(), "[3.300000, 221.200000, 5.300000]");
  }
  {
    tuw_msgs::Point src(3.3, 0.0, 0.0);
    str = src.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_STREQ(str.c_str(), "[3.300000, 0.000000, 0.000000]");
  }
  {
    tuw_msgs::Point src(3.3, 0.0, 0.0);
    str = src.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_STREQ(str.c_str(), "[3.300000, 0.000000, 0.000000]");
  }
  {
    tuw_msgs::Point src(3.3, 1.0, 0.0);
    str = src.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_STREQ(str.c_str(), "[3.300000, 1.000000]");
  }
  {
    tuw_msgs::Point src(0.0, 0.0, 0.0);
    str = src.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_STREQ(str.c_str(), "[0.000000, 0.000000, 0.000000]");
  }
  {
    tuw_msgs::Point src(0.0, 0.0, 0.0);
    str = src.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_STREQ(str.c_str(), "[]");
  }
}

TEST(Serialize, point_from_string)
{
  {
    tuw_msgs::Point des("[3.300000, 1.000000, 0.000000]");
    ASSERT_EQ(des, tuw_msgs::Point(3.3, 1.0, 0.0));
  }
  {
    tuw_msgs::Point des("[3.300000, 221.200000, 5.300000]");
    ASSERT_EQ(des, tuw_msgs::Point(3.3, 221.2, 5.3));
  }
  {
    tuw_msgs::Point des("[3.300000, 221.200000]");
    ASSERT_EQ(des, tuw_msgs::Point(3.3, 221.2, 0));
  }
  {
    tuw_msgs::Point des("[]");
    ASSERT_EQ(des, tuw_msgs::Point(0, 0, 0));
  }
  {
    tuw_msgs::Point des(" [3.300000, 1.000000, 0.000000]");
    ASSERT_EQ(des, tuw_msgs::Point(3.3, 1.0, 0.0));
  }
  {
    tuw_msgs::Point des(" [3.300000, 221.200000, 5.300000]");
    ASSERT_EQ(des, tuw_msgs::Point(3.3, 221.2, 5.3));
  }
  {
    tuw_msgs::Point des(" [3.300000, 221.200000]");
    ASSERT_EQ(des, tuw_msgs::Point(3.3, 221.2, 0));
  }
  {
    tuw_msgs::Point des(" []");
    ASSERT_EQ(des, tuw_msgs::Point(0, 0, 0));
  }
}


TEST(Serialize, quaternion_to_string)
{
  std::string str;
  {
    str = tuw_msgs::Quaternion(0.413010, 0.197120, -0.753504, 0.472016).to_str();
    ASSERT_STREQ(str.c_str(), "[0.413010, 0.197120, -0.753504, 0.472016]");
  }
  {
    str = tuw_msgs::Quaternion(0.0, 0.0, 0.0, 1.0).to_str();
    ASSERT_STREQ(str.c_str(), "[0.000000, 0.000000, 0.000000, 1.000000]");
  }
  {
    ;
    str = tuw_msgs::Quaternion(0.413010, 0.197120, -0.753504, 0.472016).to_str(tuw_msgs::Format::LOOSE);
    ASSERT_STREQ(str.c_str(), "[0.413010, 0.197120, -0.753504, 0.472016]");
  }
  {
    str = tuw_msgs::Quaternion(0.0, 0.0, 0.0, 1.0).to_str(tuw_msgs::Format::COMPACT);
    ASSERT_STREQ(str.c_str(), "[]");
  }
}


TEST(Serialize, Quaternion_from_string)
{
  {
    tuw_msgs::Quaternion des("[0.413010, 0.197120, -0.753504, 0.472016]");
    ASSERT_EQ(des, tuw_msgs::Quaternion(0.413010, 0.197120, -0.753504, 0.472016));
  }
  {
    tuw_msgs::Quaternion des("[0.00000, 0.000000, 0.000000, 1.000000]");
    ASSERT_EQ(des, tuw_msgs::Quaternion(0.0, 0.0, 0.0, 1.0));
  }
  {
    tuw_msgs::Quaternion des("[]");
    ASSERT_EQ(des, tuw_msgs::Quaternion(0.0, 0.0, 0.0, 1.0));
  }
  {
    tuw_msgs::Quaternion des("[3.3, 2.2, 1.2]");
    ASSERT_TRUE(tuw_msgs::Quaternion(0.413010, 0.197120, -0.753504, 0.472016).similar(des));
  }
}


TEST(Serialize, pose_to_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 1.0, 0.0);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  {
    tuw_msgs::Pose pose(p0, q0);
    std::string str = pose.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_STREQ(str.c_str(), "[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]");
  }
  {
    tuw_msgs::Pose pose(p0, q0);
    std::string str = pose.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_STREQ(str.c_str(), "[[], []]");
  }
  {
    tuw_msgs::Pose pose(p1, q0);
    std::string str = pose.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_STREQ(str.c_str(), "[[3.300000, 1.000000], []]");
  }
  {
    tuw_msgs::Pose pose(p1, q1);
    std::string str = pose.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_STREQ(str.c_str(), "[[3.300000, 1.000000], [0.413010, 0.197120, -0.753504, 0.472016]]");
  }
}
/*
TEST(Serialize, decode_encode_pose)
{

  std::string str(" [[3.3, 221.2, 1], [0.413010, 0.197120, -0.753504, 0.472016]]");
  tuw_msgs::Pose src;
  size_t pos = 0;
  pos = tuw_msgs::decode(src, str, pos);
  ASSERT_FLOAT_EQ(src.position.x, 3.3);

  std::string str_pose;
  tuw_msgs::encode(src, str_pose);
  ASSERT_STREQ(str_pose.c_str(), "[[3.300000, 221.200000, 1.000000], [0.413010, 0.197120, -0.753504, 0.472016]]");
  tuw_msgs::Pose des;
  pos = tuw_msgs::decode(des, str_pose);
  ASSERT_FLOAT_EQ(des.position.x, 3.3);
}

TEST(Serialize, decode_encode_pose_with_roll_pitch_yaw)
{
  std::string str(" [[3.3, 221.2, 1], [3.3, 2.2, 1.2]]");
  tuw_msgs::Pose src;
  size_t pos = 0;
  pos = tuw_msgs::decode(src, str, pos);
  ASSERT_FLOAT_EQ(src.position.x, 3.3);

  std::string str_pose;
  tuw_msgs::encode(src, str_pose);
  ASSERT_STREQ(str_pose.c_str(), "[[3.300000, 221.200000, 1.000000], [0.413010, 0.197120, -0.753504, 0.472016]]");
  tuw_msgs::Pose des;
  pos = tuw_msgs::decode(des, str_pose);
  ASSERT_FLOAT_EQ(des.position.x, 3.3);
}
*/