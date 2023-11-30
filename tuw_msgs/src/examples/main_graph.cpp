#include <iostream>
#include <tuw_msgs/utils.hpp>
#include <tuw_msgs/graph.hpp>

int main(int argc, char ** argv)
{
  std::string filename = "res/graph.txt";
  std::cout << "usage: " << argv[0] << " <filename>" << std::endl;
  std::cout << "default filname = " << filename << std::endl;
  if (argc == 2) {filename = argv[1];}

  tuw_msgs::Graph graph_serialization;
  tuw_graph_msgs::msg::Graph msg;
  graph_serialization.read(filename, msg);
}
