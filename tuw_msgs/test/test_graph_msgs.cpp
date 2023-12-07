#include "gtest/gtest.h"
#include <tuw_msgs/node.hpp>
#include <tuw_msgs/edge.hpp>
#include <tuw_msgs/graph.hpp>

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
    ASSERT_STREQ(str.c_str(), "3:[[0.000000,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]");
  }
  {
    tuw_msgs::Node o(3, pose0);
    str = tuw_msgs::remove_spaces(o.to_str(tuw_msgs::Format::COMPACT));
    ASSERT_STREQ(str.c_str(), "3:[[],[]]");
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
    tuw_msgs::Node o("3:[[0.000000,0.000000,0.000000],[0.000000,0.000000,0.000000,1.000000]]");
    ASSERT_EQ(o, tuw_msgs::Node(3, pose0));
  }

  {
    tuw_msgs::Node o(" 3: [[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]");
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
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "3: invalid: undirected:   0.0000: [   0,    0]:[]"));
  }
  {
    tuw_msgs::Edge o(5, true, true, 0.2, 19, 22);
    std::string str = o.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "5: valid: directed: 0.2000: [19, 22]:[]"));
  }
  {
    tuw_msgs::Edge o(5, false, false, 0.2, 19, 22);
    o.set_path(path);
    std::string str = o.to_str(tuw_msgs::Format::LOOSE);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, "  5:   invalid:   undirected:   0.2000: [  19,   22]: [[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[0.000000, 0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.300000, 2.200000, 1.100000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[3.300000, 2.200000, 1.100000], [0.413010, 0.197120, -0.753504, 0.472016]]]"));
  }
  {
    tuw_msgs::Edge o(5, true, true, 0.2, 19, 22);
    o.set_path(path);
    std::string str = o.to_str(tuw_msgs::Format::COMPACT);
    ASSERT_TRUE(tuw_msgs::is_similar_with_cout(str, " 5:   valid:   directed:   0.2000: [  19,   22]: [[[], []]; [[], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.300000, 2.200000, 1.100000], []]; [[3.300000, 2.200000, 1.100000], [0.413010, 0.197120, -0.753504, 0.472016]]]"));
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
    std::string src("5: valid: directed: 0.2000: [19, 22]: [[[0.000000, 0.000000, 0.000000], [0.000000, 0.000000, 0.000000, 1.000000]]; [[0.000000, 0.000000], [0.413010, 0.197120, -0.753504, 0.472016]]; [[3.3, 2.2, 1.1], []]; [[3.3, 2.2, 1.1], [0.413010, 0.197120, -0.753504, 0.472016]]]");
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

TEST(Serialize, graph_to_string)
{
  tuw_msgs::Pose pose0(0, 0, 0);
  tuw_msgs::Pose pose1(1, 0, 0);
  tuw_msgs::Pose pose2(2, 0, 0);
  tuw_msgs::Pose pose3(2, 1, 0);
  tuw_msgs::Pose pose4(3, 1, 0);
  std::vector<tuw_msgs::Pose> path = {pose1, pose2, pose3};

  {
    tuw_msgs::Edge edge1(1, true, false, 0.2, 0, 1, path);
    tuw_msgs::Node node0(0, pose0);
    tuw_msgs::Node node1(1, pose4);
    tuw_msgs::Graph src("r0_map", tuw_msgs::Pose(-1,-2, 0));
    src.edges.push_back(edge1);
    src.nodes.push_back(node0);
    src.nodes.push_back(node1);
    std::string src_str = src.to_str(tuw_msgs::Format::COMPACT);
    // std::cout << src_str << std::endl;
    tuw_msgs::Graph des;
    des.from_str(src_str);
    ASSERT_TRUE(des.similar(src));
  }
  {
    tuw_msgs::Edge edge1(1, true, true, 0.2, 0, 1, path);
    tuw_msgs::Node node0(0, pose0);
    tuw_msgs::Node node1(1, pose4);
    tuw_msgs::Graph src("r0_map", tuw_msgs::Pose(-1,-2, 0));
    src.edges.push_back(edge1);
    src.nodes.push_back(node0);
    src.nodes.push_back(node1);
    std::string src_str = src.to_str(tuw_msgs::Format::COMPACT);

    tuw_msgs::Graph des;
    des.from_str(src_str);
    des.header.frame_id = "map";
    ASSERT_FALSE(des.similar(src));
  }
  {
    tuw_msgs::Edge edge1(1, true, true, 0.2, 0, 1, path);
    tuw_msgs::Node node0(0, pose0);
    tuw_msgs::Node node1(1, pose4);
    tuw_msgs::Graph src("r0_map", tuw_msgs::Pose(-1,-2, 0));
    src.edges.push_back(edge1);
    src.nodes.push_back(node0);
    src.nodes.push_back(node1);
    src.write("graph.txt");
    tuw_msgs::Graph des;
    des.read("graph.txt");
    ASSERT_TRUE(des.similar(src));
  }
  {
    tuw_msgs::Edge edge1(1, true, true, 0.2, 0, 1, path);
    tuw_msgs::Node node0(0, pose0);
    tuw_msgs::Node node1(1, pose4);
    tuw_msgs::Graph src("r0_map", tuw_msgs::Pose(-1,-2, 0));
    src.edges.push_back(edge1);
    src.nodes.push_back(node0);
    src.nodes.push_back(node1);
    src.write("graph.txt", tuw_msgs::Format::COMPACT);
    tuw_msgs::Graph des;
    des.read("graph.txt");
    ASSERT_TRUE(des.similar(src));
  }
}