#include <stdexcept>
#include <string>
#include <tuw_msgs/point.hpp>

using namespace tuw_msgs;

Point::Point(double x, double y, double z) {this->x = x, this->y = y, this->z = z;}
Point::Point(const std::string & src) {this->from_str(src);}
Point & Point::set(double x, double y, double z)
{
  this->x = x, this->y = y, this->z = z;
  return *this;
}

bool Point::operator==(const Point & rhs) const
{
  return (x == rhs.x) && (y == rhs.y) && (z == rhs.z);
}
bool Point::is_zero() const {return (this->x == 0.) && (this->y == 0.) && (this->z == 0.);}

std::string Point::to_str(tuw_msgs::Format format) const
{
  std::string str;
  return to_str(str, format);
}

std::string & Point::to_str(std::string & des, tuw_msgs::Format format, bool append) const
{
  char txt[0x40];
  if ((format == COMPACT) && this->is_zero()) {
    sprintf(txt, "[]");
  } else if ((format == COMPACT) && this->z == 0.) {
    sprintf(txt, "[%f, %f]", this->x, this->y);
  } else {
    sprintf(txt, "[%f, %f, %f]", this->x, this->y, this->z);
  }
  if (append) {
    des.append(txt);
  } else {
    des.assign(txt);
  }
  return des;
}

size_t Point::from_str(const std::string & src)
{
  size_t offset = src.find("[");
  if (offset == std::string::npos) {throw std::runtime_error("Failed decode Point: " + src);}
  int n = sscanf(src.c_str() + offset, "[%lf,%lf,%lf]%*s", &this->x, &this->y, &this->z);
  if (n != 3) {
    int n = sscanf(src.c_str() + offset, "[%lf,%lf]%*s", &this->x, &this->y);
    if (n != 2) {
      size_t pos = src.find("[]", offset);
      if (pos == std::string::npos) {
        throw std::runtime_error("Failed decode Point incorrect number of values: " + src);
      }
      this->x = 0, this->y = 0, this->z = 0;
    }
    this->z = 0;
  }
  offset = src.find("]");
  if (offset == std::string::npos) {throw std::runtime_error("Failed decode Point: " + src);}
  return offset;
}

geometry_msgs::msg::Point & Point::msg() {return static_cast<geometry_msgs::msg::Point &>(*this);}
const geometry_msgs::msg::Point & Point::msg() const
{
  return static_cast<const geometry_msgs::msg::Point &>(*this);
}
double Point::similar(const Point & rhs, double epsilon) const
{
  return is_similar(this->msg(), rhs.msg(), epsilon);
}


#include <jsoncpp/json/json.h>
  int Point::json_get(Json::Value &value){
    value["x"] = this->x;
    value["y"] = this->y;
    value["z"] = this->z;
    return 0;
  }
  int Point::json_add(const char* key, Json::Value &des){
    Json::Value value;
    json_get(value);
    des[key] = value;
    return 0;
  }
  Json::Value Point::toJson() const {
        Json::Value jsonValue;
        jsonValue["x"] = this->x;
        jsonValue["y"] = this->y;
        jsonValue["z"] = this->z;
        return jsonValue;
    }