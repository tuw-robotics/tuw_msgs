#ifndef TUW_STD_MSGS_MSGS__PARAMETER__ARRAY_HPP_
#define TUW_STD_MSGS_MSGS__PARAMETER__ARRAY_HPP_

#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <tuw_std_msgs/msg/parameter_array.hpp>
#include <tuw_std_msgs/parameter.hpp>
#include <vector>

namespace tuw_std_msgs
{
struct ParameterArray : public tuw_std_msgs::msg::ParameterArray
{
  ParameterArray() {
    this->id = -1;
  }
  ParameterArray(int id) {
    this->id = id;
  }
  
  template<typename T>
  ParameterArray(int id, const std::vector<std::string> &names, const std::vector<T> &values) {
    this->id = id;
    this->data.resize(names.size());
    for(size_t i = 0; i < names.size(); i++){
      Parameter p(names[i], values[i]);
      this->data.push_back(std::move(p));
    }
  }
  
  /**
   * searches in the array for a parameter name
   * @param name name of the parameter
   * @return pointer to the parameter or null if it does not exist
   */
  template<typename T>
  tuw_std_msgs::Parameter *add(const std::string &name, const T &data){
    for(size_t i = 0; i < this->data.size(); i++){
      Parameter *p = (Parameter*) &this->data[i] ;
      if(p->name == name){
        return p;
      }
    }
    return NULL;
  }

  /**
   * searches in the array for a parameter name
   * @param name name of the parameter
   * @return pointer to the parameter or null if it does not exist
   */
  const Parameter *get(const std::string &name) const{
    for(size_t i = 0; i < this->data.size(); i++){
      const Parameter *p = (Parameter*) &this->data[i] ;
      if(p->name == name){
        return p;
      }
    }
    return NULL;
  }

  /**
   * used to read a parameter
   * @param name name of the parameter
   * @param des varaible to filled with the parameter if exists
   * @return true if the parameter exists
   */
  template<typename T>
  bool get(const std::string &name, T &des) const{
    const Parameter *p = get(name);
    if(p != NULL){
      p->get(des);
      return true;
    }
    return false;
  }
};
}  // namespace tuw_std_msgs
#endif  // TUW_STD_MSGS_MSGS__PARAMETER__ARRAY_HPP_
