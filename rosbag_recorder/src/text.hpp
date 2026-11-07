#ifndef TEXT_HPP_
#define TEXT_HPP_

#include <string>
#include <vector>

namespace rosbag_recorder
{

// The first few names, then how many more there are.
std::string listed(const std::vector<std::string> & names);
std::string fixed(double value, int precision);

}  // namespace rosbag_recorder

#endif  // TEXT_HPP_
