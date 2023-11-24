
#include <iostream>
#include <fstream>
#include <string>
#include <string_view>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <tuw_msgs/serialize.hpp>

using namespace tuw_msgs;

Serialize::Serialize()
{
    this->delim();
}

void Serialize::delim(char delim_entries, char delim_rows, char delim_cols){
    this->delim_entries = delim_entries;
    this->delim_rows = delim_rows;
    this->delim_cols = delim_cols;
}

Serialize::~Serialize()
{
}
void Serialize::split(std::vector<std::string> &strings, const std::string &str, char delim)
{
    std::istringstream ss(str);
    std::string line;
    while (std::getline(ss, line, delim))
    {
        strings.push_back(line); 
    }
}

bool Serialize::check(const std::string &search, std::string &target, bool trim)
{
    std::string::size_type pos = target.find(search);
    if ((pos != std::string::npos) && (pos == 0))
    {
        if(trim)
            target.erase(pos, search.length());
        return true;
    }
    return false;
}

bool Serialize::check_and_remove_symbol_open(std::string &line, const char *symbol)
{
    /// remove the opening bracket from the first entry if it exits
    std::string::size_type pos_open_bracket = line.find(symbol);
    if (pos_open_bracket != std::string::npos)
        line.erase(0, pos_open_bracket + 1);
    else
        return false;
    return true;
}

bool Serialize::check_and_remove_symbol_close(std::string &line, const char *symbol)
{
    /// remove the closing bracket from the last entry if it exits
    std::string::size_type pos_close_bracket = line.rfind(symbol);
    if (pos_close_bracket != std::string::npos)
        line.erase(pos_close_bracket, line.length() - pos_close_bracket);
    else
        return false;
    return true;
}

int Serialize::decode_value(const std::string &line, double &nr)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    try
    {
        nr = std::stod(line);
    }
    catch (const std::invalid_argument &ia)
    {
        return DECODE_ERROR_INVALID;
    }
    return DECODE_SUCCESSFUL;
}
int Serialize::decode_value(const std::string &line, float &nr)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    try
    {
        nr = std::stof(line);
    }
    catch (const std::invalid_argument &ia)
    {
        return DECODE_ERROR_INVALID;
    }
    return DECODE_SUCCESSFUL;
}

int Serialize::decode_value(const std::string &line, uint32_t &nr)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    try
    {
        nr = std::stoi(line);
    }
    catch (const std::invalid_argument &ia)
    {
        return DECODE_ERROR_INVALID;
    }
    return DECODE_SUCCESSFUL;
}

int Serialize::decode_value(const std::string &line, int64_t &nr)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    try
    {
        nr = std::stoi(line);
    }
    catch (const std::invalid_argument &ia)
    {
        return DECODE_ERROR_INVALID;
    }
    return DECODE_SUCCESSFUL;
}
int Serialize::decode_value(const std::string &line, int32_t &nr)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    try
    {
        nr = std::stoi(line);
    }
    catch (const std::invalid_argument &ia)
    {
        return DECODE_ERROR_INVALID;
    }
    return DECODE_SUCCESSFUL;
}

int Serialize::decode_value(std::string line, std::__cxx11::basic_string<char> &frame_id)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    line.erase(line.begin(), std::find_if(line.begin(), line.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
    frame_id = line;
    return DECODE_SUCCESSFUL;
}

int Serialize::decode_value(const std::string &line, bool &des)
{
    if (line.empty())
        return DECODE_ERROR_SYNTAX;
    if ((line.find("true") != std::string::npos))
        des = true;
    else if ((line.find("1") != std::string::npos))
        des = true;
    else if ((line.find("false") != std::string::npos))
        des = false;
    else if ((line.find("0") != std::string::npos))
        des = false;
    else
        return DECODE_ERROR_SYNTAX;
    return DECODE_SUCCESSFUL;
}

int Serialize::decode_value(const std::string &line, geometry_msgs::msg::Point &point)
{
    std::vector<std::string> entries;
    split(entries, line, delim_cols);

    /// remove the opening and closing brackets
    if (entries.size() == 0)
        return DECODE_EMPTY;
    bool open_found = check_and_remove_symbol_open(entries.front(), "[");
    bool close_found = check_and_remove_symbol_close(entries.back(), "]");
    if (!open_found && !close_found)
        return DECODE_EMPTY;
    if (!open_found || !close_found)
        return DECODE_ERROR_SYNTAX;

    /// check the number of entries
    if ((entries.size() != 2) && (entries.size() != 3))
        return DECODE_ERROR_SYNTAX;

    try
    {
        point.x = std::stod(entries[0]);
        point.y = std::stod(entries[1]);
        if (entries.size() >= 3)
            point.z = std::stod(entries[2]);
        else
            point.z = 0.;
    }
    catch (const std::invalid_argument &ia)
    {
        return 0;
    }
    return DECODE_SUCCESSFUL;
}

int Serialize::decode_value(const std::string &line, geometry_msgs::msg::Pose &pose)
{
    std::vector<std::string> entries;
    split(entries, line, delim_cols);

    /// remove the opening and closing brackets
    bool open_found = check_and_remove_symbol_open(entries.front(), "[");
    bool close_found = check_and_remove_symbol_close(entries.back(), "]");
    if (!open_found && !close_found)
        return DECODE_EMPTY;
    if (!open_found || !close_found)
        return DECODE_ERROR_SYNTAX;

    /// check the number of entries
    if ((entries.size() != 3) && (entries.size() != 6))
        return DECODE_ERROR_SYNTAX;

    try
    {
        pose.position.x = std::stod(entries[0]);
        pose.position.y = std::stod(entries[1]);
        pose.position.z = std::stod(entries[2]);
    }
    catch (const std::invalid_argument &ia)
    {
        return DECODE_ERROR_INVALID;
    }
    if (entries.size() == 6)
    {
        pose.orientation.x = 0.;
        pose.orientation.y = 0.;
        pose.orientation.z = 0.;
        pose.orientation.w = 1.;
        /// ToDo computation quaternion
    }
    return DECODE_SUCCESSFUL;
}
