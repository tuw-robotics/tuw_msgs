#include <iostream>
#include <fstream>
#include <tuw_msgs/graph.hpp>
#include <jsoncpp/json/json.h>

int main() {
    tuw_msgs::Graph graph;
    graph.readJson("graph-captured.json");
    graph.writeJson("graph-captured2.json");

    return 0;
}