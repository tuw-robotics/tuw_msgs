#include <tuw_msgs/node.hpp>

using namespace tuw_msgs;

Node::Node() {}

Node::Node(Idx id)
{
    this->id = id;
}

Node::Node(Idx id, const Pose &pose)
{
    set(id, pose);
}

Node::Node(const std::string &str)
{
    this->from_str(str);
}

Node &Node::set(Idx id, const Pose &pose)
{
    this->id = id;
    this->pose = pose;
    return *this;
}

bool Node::operator==(const Node &rhs) const
{
    return (this->id == rhs.id) && (get_pose() == rhs.get_pose());
}

double Node::similar(const Node &rhs, double threshold_position, double threshold_orientation) const
{

    return (this->id == rhs.id) && get_pose().similar(rhs.get_pose(), threshold_position, threshold_orientation);
}
std::string Node::to_str(tuw_msgs::Format format) const
{
    std::string str;
    return this->to_str(str, format);
}
std::string &Node::to_str(std::string &des, tuw_msgs::Format format, bool append) const
{
    char txt[128];
    sprintf(txt, "node: %4ld: %s", id, this->get_pose().to_str(format).c_str());
    if (!append)
        des.clear();
    des.append(txt);
    return des;
}
size_t Node::from_str(const std::string &str)
{
    size_t offset = str.find("node:");
    if (offset == std::string::npos)
        throw std::runtime_error("Failed decode node: " + str);
    int n = sscanf(str.c_str(), "node:%4ld:%*s", &this->id);
    if (n != 1)
        throw std::runtime_error("Failed decode Node: " + str);
    offset = str.rfind(":");
    if (offset == std::string::npos)
        throw std::runtime_error("Failed decode node: " + str);

    offset = this->get_pose().from_str(str.substr(offset)) + offset;
    return offset;
}