#include "text.hpp"

#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace rosbag_recorder
{
namespace
{

constexpr std::size_t names_listed = 4;

}  // namespace

std::string listed(const std::vector<std::string> & names)
{
  std::string out;
  for (std::size_t i = 0; i < names.size() && i < names_listed; ++i) {
    out += (i ? ", " : "") + names[i];
  }
  if (names.size() > names_listed) {
    out += " and " + std::to_string(names.size() - names_listed) + " more";
  }
  return out;
}

std::string fixed(double value, int precision)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

}  // namespace rosbag_recorder
