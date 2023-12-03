#ifndef TUW_MSGS__UTILS_HPP_
#define TUW_MSGS__UTILS_HPP_

namespace tuw_msgs
{
    enum Format{
        COMPACT,
        LOOSE
    };
    
    inline size_t nr_of_leading_spaces(const std::string& inputString) {
        size_t nr_of_spaces = 0;
        for (char ch : inputString) {
            if (ch == ' ') {
                nr_of_spaces++;
            } else {
                break;
            }
        }
        return nr_of_spaces;
    }
};


#endif // TUW_MSGS__UTILS_HPP_
