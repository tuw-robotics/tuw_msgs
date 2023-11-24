#ifndef TUW_MSGS__SERIALIZE_HPP_
#define TUW_MSGS__SERIALIZE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace tuw_msgs
{
    /**
     * Class to read a costomized message format for selected tuw_msgs
    */
    class Serialize
    {
    protected:
        static const int DECODE_ERROR_INVALID = 1; 
        static const int DECODE_ERROR_SYNTAX = 0; 
        static const int DECODE_SUCCESSFUL = -1; 
        static const int ENCODE_SUCCESSFUL = -2; 
        static const int DECODE_EMPTY = -3; 
        char delim_entries;
        char delim_rows;
        char delim_cols;

    public:
        Serialize();
        ~Serialize();



        void delim(char delim_entries = ':', char delim_rows = ';', char delim_cols = ',');
        /**
         * decode origin from a string
         * @param line string with information
         * @param origin reference to destination
         * @return return -1 on success otherwise the coloum index number with the issue
         */
        int decode_origin(const std::string &line, geometry_msgs::msg::Point &origin);

        /**
         * decode string from a string
         * @param line string with information
         * @param frame reference to destination
         * @return return -1 on success otherwise the coloum index number with the issue
         */
        int decode_value(std::string line, std::__cxx11::basic_string<char> &frame);

        /**
         * decode point from a string [x, y] or [x, y, z]
         * @param line string with information
         * @param point reference to destination
         * @return return -1 on success otherwise the coloum index number with the issue
         */
        int decode_value(const std::string &line, geometry_msgs::msg::Point &point);

        /**
         * decode pose from a string [x, y, z, yaw, pitch, roll]
         * @param line string with information
         * @param point reference to destination
         * @return return -1 on success otherwise the coloum index number with the issue
         */
        int decode_value(const std::string &line, geometry_msgs::msg::Pose &pose);

        /**
         * decode a number from a string
         * @param line string with information
         * @param nr reference to destination
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        int decode_value(const std::string &line, uint32_t &nr);


        /**
         * decode a number from a string
         * @param line string with information
         * @param nr reference to destination
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        int decode_value(const std::string &line, int64_t &nr);

        /**
         * decode a number from a string
         * @param line string with information
         * @param nr reference to destination
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        int decode_value(const std::string &line, int32_t &nr);

        /**
         * decode a number from a string
         * @param line string with information
         * @param nr reference to destination
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        int decode_value(const std::string &line, double &nr);

        /**
         * decode a number from a string
         * @param line string with information
         * @param nr reference to destination
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        int decode_value(const std::string &line, float &nr);

        /**
         * decode a number from a string
         * @param line string with information
         * @param nr reference to destination
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        int decode_value(const std::string &line, bool &des);

        /**
         * decode a list of values from a string
         * @param line string with information
         * @param list pointer to a vector of numbers
         * @return return -1 on 0 on empty string and 1 otherwise on an exeption
         */
        template <class T>
        int decode_list(const std::string &line, std::vector<T> &list){
            std::vector<std::string> entries;
            split(entries, line, delim_rows);

            bool  open_found = check_and_remove_symbol_open(entries.front(), "[");
            bool close_found = check_and_remove_symbol_close(entries.back(), "]");
            if(!open_found && !close_found) return DECODE_EMPTY;
            if(!open_found || !close_found) return DECODE_ERROR_SYNTAX;

            for(size_t i = 0; i < entries.size(); i++){
                T value;
                int result = decode_value(entries[i], value);
                if(result == DECODE_SUCCESSFUL){
                    list.push_back(std::move(value));
                } else if(result != DECODE_EMPTY){
                    return DECODE_ERROR_SYNTAX;
                }
            }
            return DECODE_SUCCESSFUL;
        }

        /**
         * check if the target string starts with the serach string and trims if set
         * @param search serach string 
         * @param target target string
         * @param trim trim string on true
         * @return return true if the target string contained the search string
         */
        static bool check(const std::string &search, std::string &target, bool trim = false);

        /**
         * splits a string in substrings by a given charater
         * @param strings strings 
         * @param str string to split
         * @param delim charater used to split
         */
        static void split(std::vector<std::string> &strings, const std::string &str, char delim);

        /**
         * check if a start symbol exists and removes it
         * @param line serach string 
         * @param symbol  
         * @return return true if the target string started with the given symbol
         */
        static bool check_and_remove_symbol_open(std::string &line, const char *symbol = "[");
        /**
         * check if an end symbol exists and removes it
         * @param line serach string 
         * @param symbol  
         * @return return true if the target string ends with the given symbol
         */
        static bool check_and_remove_symbol_close(std::string &line, const char *symbol = "]");
    };
}

#endif  // TUW_MSGS__SERIALIZE_HPP_