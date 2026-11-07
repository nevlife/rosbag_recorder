#ifndef DISK_WATCH_HPP_
#define DISK_WATCH_HPP_

#include <cstdint>
#include <deque>
#include <utility>

namespace rosbag_recorder
{

struct DiskReading
{
  double write_mbps{0.0};
  // NaN while nothing is written.
  double remaining_min{0.0};
  bool stop{false};
};

// The write rate of one session over its last reports, and when low disk space stops it.
class DiskWatch
{
public:
  void reset();
  DiskReading update(double mono, std::uint64_t bag_bytes, double free_gb, double stop_gb);

private:
  std::deque<std::pair<double, std::uint64_t>> history_;
  int low_reports_{0};
};

}  // namespace rosbag_recorder

#endif  // DISK_WATCH_HPP_
