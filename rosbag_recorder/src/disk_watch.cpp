#include "disk_watch.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>

namespace rosbag_recorder
{
namespace
{

constexpr std::size_t reports_kept = 12;
constexpr int low_reports_to_stop = 3;
constexpr double min_write_mbps = 0.01;

}  // namespace

void DiskWatch::reset()
{
  history_.clear();
  low_reports_ = 0;
}

DiskReading DiskWatch::update(double mono, std::uint64_t bag_bytes, double free_gb, double stop_gb)
{
  history_.emplace_back(mono, bag_bytes);
  if (history_.size() > reports_kept) {
    history_.pop_front();
  }
  const auto [t0, b0] = history_.front();
  DiskReading r;
  if (mono > t0) {
    r.write_mbps =
      (static_cast<double>(bag_bytes) - static_cast<double>(b0)) / (mono - t0) / 1048576.0;
  }
  r.remaining_min = r.write_mbps > min_write_mbps
                      ? std::max(0.0, (free_gb - stop_gb) * 1024.0 / r.write_mbps / 60.0)
                      : std::numeric_limits<double>::quiet_NaN();
  low_reports_ = free_gb < stop_gb ? low_reports_ + 1 : 0;
  r.stop = low_reports_ >= low_reports_to_stop;
  return r;
}

}  // namespace rosbag_recorder
