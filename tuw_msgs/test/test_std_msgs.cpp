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


TEST(Serialize, Pose_from_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q1);
  {
    tuw_msgs::Pose des("[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]");
    ASSERT_TRUE(pose0.similar(des));
  }
  {
    tuw_msgs::Pose des("[[0.000000, 0.000000, 0.000000], []]");
    ASSERT_TRUE(pose0.similar(des));
  }
  {
    tuw_msgs::Pose des("[[0.000000, 0.000000], []]");
    ASSERT_TRUE(pose0.similar(des));
  }
  {
    tuw_msgs::Pose des("[[0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]");
    ASSERT_TRUE(pose1.similar(des));
  }
  {
    tuw_msgs::Pose des(" [[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]");
    ASSERT_TRUE(pose2.similar(des));
  }
  {
    tuw_msgs::Pose des(" [[], [0.413010, 0.197120, -0.753504, 0.472016]]");
    ASSERT_TRUE(pose1.similar(des));
  }
  {
    tuw_msgs::Pose des("[[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]");
    ASSERT_TRUE(pose2.similar(des));
  }
}

TEST(Serialize, Pose_vector_from_string)
{
  tuw_msgs::Point p0(0, 0, 0);
  tuw_msgs::Quaternion q0(0, 0, 0, 1);
  tuw_msgs::Point p1(3.3, 2.2, 1.1);
  tuw_msgs::Quaternion q1(0.413010, 0.197120, -0.753504, 0.472016);
  tuw_msgs::Pose pose0(p0, q0);
  tuw_msgs::Pose pose1(p0, q1);
  tuw_msgs::Pose pose2(p1, q1);
  {
    std::string src("[[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]]");
    std::vector<tuw_msgs::Pose> vec = {pose0, pose1, pose2};
    std::vector<tuw_msgs::Pose> vec_src;
    tuw_msgs::Pose::from_str(src, vec_src);
    ASSERT_TRUE(vec == vec_src);
  }
  {
    std::string src("[[[0.000000, 0.100000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]]");
    std::vector<tuw_msgs::Pose> vec = {pose0, pose1, pose2};
    std::vector<tuw_msgs::Pose> vec_src;
    tuw_msgs::Pose::from_str(src, vec_src);
    ASSERT_FALSE(vec == vec_src);
  }
  {
    std::string src("[[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.010000]]; [[0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]]");
    std::vector<tuw_msgs::Pose> vec = {pose0, pose1, pose2};
    std::vector<tuw_msgs::Pose> vec_src;
    tuw_msgs::Pose::from_str(src, vec_src);
    ASSERT_FALSE(vec == vec_src);
  }
}