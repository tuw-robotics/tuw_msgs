#include <jsoncpp/json/json.h>

#include <fstream>
#include <iostream>
#include <tuw_msgs/graph.hpp>

int main()
{
  tuw_msgs::Graph graph;
  graph.readJson("graph-captured.json");
  graph.writeJson("graph-captured2.json");

  return 0;
}
