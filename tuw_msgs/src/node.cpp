#include <tuw_msgs/node.hpp>

using namespace tuw_msgs;

Node::Node() {}

Node::Node(Idx id){
    this->id = id;
}

Node::Node(Idx id, const Pose& pose){
    set(id, pose);
}

Node& Node::set(Idx id, const Pose& pose)
{
   this->id = id;
   this->pose = pose;
   return *this;
}


bool Node::operator==(const Node &rhs) const
{
    return (this->id == rhs.id) && (get_pose() == rhs.get_pose());
}

double Node::similar(const Node &rhs, double threshold_position, double threshold_orientation) const {

    return (this->id == rhs.id) && get_pose().similar(rhs.get_pose(), threshold_position, threshold_orientation);
}
std::string Node::to_str(tuw_msgs::Format format) const
{
    std::string str;
    return this->to_str(str, format);
}
std::string &Node::to_str(std::string &des, tuw_msgs::Format format, bool append) const
{
    char txt[0xFF];
    sprintf(txt, "node: %4ld: %s", id, this->get_pose().to_str(format).c_str());
    if (!append)
        des.clear();
    des.append(txt);
    return des;
}
Node &Node::from_str(const std::string &src) {

}