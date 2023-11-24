#ifndef TUW_MSGS__GRAPH_HPP_
#define TUW_MSGS__GRAPH_HPP_

#include <tuw_msgs/serialize.hpp>
#include <tuw_graph_msgs/msg/graph.hpp>

namespace tuw_msgs
{
    class Graph : public Serialize
    {
    public:
        Graph();
        ~Graph();

        /**
         * read graph file
         * @param graph reference to destination
         * @return return -1 on success otherwise the line number with the issue
         */
        int read(std::string const &filename, tuw_graph_msgs::msg::Graph &graph);
    private:
        /**
         * decode node from a string
         * @param line string with information
         * @param node reference to destination
         * @return return -1 on success otherwise the coloum index number with the issue
         */
        int decode_node(const std::string &line, tuw_graph_msgs::msg::Node &node);

        /**
         * decode edge from a string
         * @param line string with information
         * @param edge reference to destination
         * @return return -1 on success otherwise the coloum index number with the issue
         */
        int decode_edge(const std::string &line, tuw_graph_msgs::msg::Edge &edge);

    };

}

#endif  // TUW_MSGS__GRAPH_HPP_