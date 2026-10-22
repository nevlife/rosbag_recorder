// Copyright 2026 pgw
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <string>
#include <utility>
#include <vector>

#include "compat.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rmw/rmw.h"
#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag_recorder/config.hpp"
#include "rosbag_recorder/session_info.hpp"
#include "rosbag_recorder/stream.hpp"
#include "rosbag_recorder_checks/check.hpp"
#include "rosbag_recorder_msgs/msg/check_result.hpp"
#include "rosbag_recorder_msgs/msg/health.hpp"
#include "rosbag_recorder_msgs/msg/marker.hpp"
#include "rosbag_recorder_msgs/msg/session.hpp"
#include "rosbag_recorder_msgs/srv/catalog_diff.hpp"
#include "rosbag_recorder_msgs/srv/mark.hpp"
#include "rosbag_recorder_msgs/srv/select.hpp"
#include "rosbag_recorder_msgs/srv/start.hpp"
#include "rosbag_recorder_msgs/srv/status.hpp"
#include "rosbag_recorder_msgs/srv/stop.hpp"
#include "rosbag_recorder_source/source.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

namespace rosbag_recorder
{
namespace
{

namespace fs = std::filesystem;
namespace msg = rosbag_recorder_msgs::msg;
namespace srv = rosbag_recorder_msgs::srv;

const char * const kSessionTopic = "/rosbag_recorder/session";
const char * const kHealthTopic = "/rosbag_recorder/health";
const char * const kMarkerTopic = "/rosbag_recorder/markers";
constexpr int kDiskStopTicks = 3;
constexpr double kMarkerGap = 10.0;
constexpr std::size_t kListed = 4;

double mono_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double wall_now()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string listed(const std::vector<std::string> & names)
{
  std::string out;
  for (std::size_t i = 0; i < names.size() && i < kListed; ++i) {
    out += (i ? ", " : "") + names[i];
  }
  return names.size() > kListed ? out + " and " + std::to_string(names.size() - kListed) + " more" :
         out;
}

std::string fixed(double v, int precision)
{
  return rosbag_recorder_checks::fixed(v, precision);
}

msg::CheckResult result(std::uint8_t level, const std::string & text)
{
  msg::CheckResult r;
  r.level = level;
  r.text = text;
  return r;
}

std::string local_time(const builtin_interfaces::msg::Time & t)
{
  const std::time_t seconds = t.sec;
  std::tm local{};
  localtime_r(&seconds, &local);
  char text[32];
  std::strftime(text, sizeof(text), "%Y-%m-%d %H:%M:%S", &local);
  return text;
}

rclcpp::QoS session_qos() {return rclcpp::QoS(2).reliable().transient_local();}
rclcpp::QoS health_qos() {return rclcpp::QoS(10).reliable();}
rclcpp::QoS marker_qos() {return rclcpp::QoS(100).reliable();}

}  // namespace

class RecorderNode : public rclcpp::Node
{
  using CheckResult = msg::CheckResult;
  using Health = msg::Health;
  using Marker = msg::Marker;
  using Session = msg::Session;

public:
  explicit RecorderNode(const rclcpp::NodeOptions & options)
  : Node("rosbag_recorder", options),
    check_loader_("rosbag_recorder_checks", "rosbag_recorder_checks::Check"),
    source_loader_("rosbag_recorder_source", "rosbag_recorder_source::Source")
  {
    config_dir_ = expand_user(declare_parameter("config_dir", std::string()));
    output_dir_ = expand_user(declare_parameter("output_dir", std::string("~/rosbag_recorder")));
    workspace_ = expand_user(declare_parameter("workspace", std::string()));
    param_globs_ = declare_parameter("param_globs", std::vector<std::string>{});
    param_copy_max_kb_ = declare_parameter("param_copy_max_kb", 5120);
    hz_tolerance_ = std::clamp(declare_parameter("hz_tolerance", 0.8), 0.1, 1.0);
    value_checks_ = declare_parameter("value_checks", true);
    const std::string profile = declare_parameter("profile", std::string());
    const bool autostart = declare_parameter("autostart", false);
    label_ = declare_parameter("label", std::string());

    if (value_checks_) {
      for (const auto & name : check_loader_.getDeclaredClasses()) {
        check_prototypes_[name] = check_loader_.createSharedInstance(name);
      }
    }
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    data_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    session_pub_ = create_publisher<Session>(kSessionTopic, session_qos());
    health_pub_ = create_publisher<Health>(kHealthTopic, health_qos());
    marker_pub_ = create_publisher<Marker>(kMarkerTopic, marker_qos());
    services();
    graph_event_ = get_graph_event();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {tick();}, control_group_);

    if (!profile.empty()) {
      try {
        select(profile);
        if (autostart) {
          request_start(false);
        }
      } catch (const std::exception & e) {
        message_ = e.what();
        RCLCPP_ERROR(get_logger(), "%s", e.what());
      }
    }
  }

  ~RecorderNode() override
  {
    if (session_open()) {
      finish("shutdown");
    }
    unselect();
  }

private:
  class SourceSink : public rosbag_recorder_source::Sink
  {
public:
    SourceSink(RecorderNode & node, std::string name, std::vector<std::shared_ptr<Stream>> streams)
    : node_(node), name_(std::move(name)), streams_(std::move(streams)) {}

    void write(
      std::size_t channel, std::shared_ptr<rclcpp::SerializedMessage> message,
      std::int64_t time_ns) override
    {
      node_.on_data(*streams_.at(channel), std::move(message), time_ns);
    }

    void mark(const std::string & kind, const std::string & text, const std::string & topic)
    override
    {
      node_.mark(kind, text, name_, topic);
    }

    std::vector<std::shared_ptr<Stream>> & streams() {return streams_;}

private:
    RecorderNode & node_;
    std::string name_;
    std::vector<std::shared_ptr<Stream>> streams_;
  };

  struct LoadedSource
  {
    std::string name;
    std::shared_ptr<rosbag_recorder_source::Source> plugin;
    std::unique_ptr<SourceSink> sink;
  };

  void services()
  {
    const auto qos = service_qos();
    select_srv_ = create_service<srv::Select>(
      "~/select", [this](const std::shared_ptr<srv::Select::Request> req,
      std::shared_ptr<srv::Select::Response> res) {on_select(*req, *res);},
      qos, control_group_);
    start_srv_ = create_service<srv::Start>(
      "~/start", [this](const std::shared_ptr<srv::Start::Request> req,
      std::shared_ptr<srv::Start::Response> res) {on_start(*req, *res);},
      qos, control_group_);
    stop_srv_ = create_service<srv::Stop>(
      "~/stop", [this](const std::shared_ptr<srv::Stop::Request>,
      std::shared_ptr<srv::Stop::Response> res) {on_stop(*res);},
      qos, control_group_);
    pause_srv_ = trigger("~/pause", [this](std::string & m) {return pause(true, m);});
    resume_srv_ = trigger("~/resume", [this](std::string & m) {return pause(false, m);});
    split_srv_ = trigger("~/split", [this](std::string & m) {return split(m);});
    mark_srv_ = create_service<srv::Mark>(
      "~/mark", [this](const std::shared_ptr<srv::Mark::Request> req,
      std::shared_ptr<srv::Mark::Response> res) {
        res->success = session_open();
        res->message = res->success ? "marked" : "not recording";
        if (res->success) {
          mark(req->kind.empty() ? "note" : req->kind, req->text, "user", "");
        }
      }, qos, control_group_);
    status_srv_ = create_service<srv::Status>(
      "~/status", [this](const std::shared_ptr<srv::Status::Request>,
      std::shared_ptr<srv::Status::Response> res) {on_status(*res);},
      qos, control_group_);
    diff_srv_ = create_service<srv::CatalogDiff>(
      "~/catalog_diff", [this](const std::shared_ptr<srv::CatalogDiff::Request>,
      std::shared_ptr<srv::CatalogDiff::Response> res) {on_catalog_diff(*res);},
      qos, control_group_);
  }

  template<class F>
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger(const std::string & name, F f)
  {
    return create_service<std_srvs::srv::Trigger>(
      name, [f](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = f(res->message);
      }, service_qos(), control_group_);
  }

  bool session_open()
  {
    const std::shared_lock<std::shared_mutex> lock(writer_mutex_);
    return writer_ != nullptr;
  }

  bool idle_or_preview() const
  {
    return state_ == Health::IDLE || state_ == Health::PREVIEW;
  }

  // ---- selection ---------------------------------------------------------------------------

  void select(const std::string & name)
  {
    unselect();
    Catalog catalog = Catalog::load((fs::path(config_dir_) / "catalog.yaml").string());
    ResolvedProfile profile =
      ProfileStore((fs::path(config_dir_) / "profiles").string()).resolve(name, catalog);
    catalog_ = std::move(catalog);
    profile_ = std::move(profile);
    std::vector<std::string> topics = profile_->topics;
    topics.insert(topics.end(), profile_->unknown_topics.begin(), profile_->unknown_topics.end());
    for (const auto & t : topics) {
      StreamSpec s{t, "", "", std::nullopt, false, false, false, true};
      if (const TopicSpec * spec = catalog_->topic(t)) {
        s.type = spec->type.value_or("");
        s.hz = spec->hz;
        s.required = spec->required;
        s.latched = spec->latched;
        s.heavy = spec->heavy;
      }
      streams_[t] = std::make_shared<Stream>(s);
    }
    for (const auto & m : catalog_->auto_markers) {
      if (!streams_.count(m.topic)) {
        streams_[m.topic] = std::make_shared<Stream>(
          StreamSpec{m.topic, "", "", std::nullopt, false, false, false, false});
      }
    }
    try {
      for (const auto & s : profile_->sources) {
        load_source(catalog_->sources.at(s));
      }
    } catch (const std::exception & e) {
      unselect();
      throw ConfigError(std::string("source: ") + e.what());
    }
    state_ = Health::PREVIEW;
    message_ = "selected " + name;
    subscribe_pending(true);
    preflight_ = preflight();
    RCLCPP_INFO(
      get_logger(), "profile %s: %zu topics, %zu sources", name.c_str(), topics.size(),
      profile_->sources.size());
  }

  void load_source(const SourceSpec & spec)
  {
    auto plugin = source_loader_.createSharedInstance(spec.plugin);
    plugin->initialize(*this, spec.name, spec.params_yaml);
    std::vector<std::shared_ptr<Stream>> streams;
    for (const auto & ch : plugin->channels()) {
      if (streams_.count(ch.name)) {
        throw ConfigError("channel " + ch.name + " of source " + spec.name + " is also a topic");
      }
      StreamSpec s{ch.name, ch.type, spec.name, std::nullopt, ch.required, ch.latched, false, true};
      if (ch.hz > 0) {
        s.hz = ch.hz;
      }
      if (const auto it = spec.channels.find(ch.name); it != spec.channels.end()) {
        s.hz = it->second.hz ? it->second.hz : s.hz;
        s.required = s.required || it->second.required;
        s.latched = s.latched || it->second.latched;
      }
      auto stream = std::make_shared<Stream>(s);
      stream->serialization_format = ch.serialization_format;
      stream->definition = ch.definition;
      stream->definition_encoding = ch.definition_encoding;
      stream->offers = {s.latched ? rclcpp::QoS(1).transient_local() : rclcpp::QoS(10)};
      attach(*stream);
      streams_[ch.name] = stream;
      streams.push_back(stream);
    }
    LoadedSource loaded{spec.name, plugin, std::make_unique<SourceSink>(*this, spec.name, streams)};
    plugin->start(*loaded.sink);
    sources_.push_back(std::move(loaded));
  }

  void unselect()
  {
    for (auto & s : sources_) {
      s.plugin->stop();
    }
    sources_.clear();
    for (auto & kv : streams_) {
      kv.second->subscription.reset();
    }
    streams_.clear();
    profile_.reset();
    catalog_.reset();
    state_ = Health::IDLE;
  }

  // Graph queries cost O(graph) each; they run when the graph changed, not every tick.
  void subscribe_pending(bool graph_changed)
  {
    for (auto & loaded : sources_) {
      auto & streams = loaded.sink->streams();
      for (std::size_t i = 0; i < streams.size(); ++i) {
        streams[i]->publishers = loaded.plugin->publishers(i);
      }
    }
    if (!graph_changed) {
      return;
    }
    bool pending = false;
    for (auto & [name, s] : streams_) {
      if (s->spec.source.empty()) {
        s->publishers = static_cast<std::uint32_t>(count_publishers(name));
        pending = pending || (!s->subscription && s->error.empty());
      }
    }
    if (!pending) {
      return;
    }
    const auto graph = get_topic_names_and_types();
    for (auto & [name, s] : streams_) {
      const auto it = graph.find(name);
      if (!s->spec.source.empty() || s->subscription || !s->error.empty() || s->publishers == 0 ||
        it == graph.end() || it->second.empty())
      {
        continue;
      }
      subscribe(s, it->second.front());
    }
  }

  void subscribe(const std::shared_ptr<Stream> & s, const std::string & type)
  {
    const std::string & name = s->spec.name;
    s->spec.type = type;
    const auto publishers = get_publishers_info_by_topic(name);
    rclcpp::QoS qos = Rosbag2QoS::adapt_request_to_offers(name, publishers);
    bool all_latched = !publishers.empty();
    s->offers.clear();
    for (const auto & p : publishers) {
      all_latched = all_latched &&
        p.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
      s->offers.push_back(p.qos_profile());
    }
    s->spec.latched = s->spec.latched || all_latched;
    if (s->spec.latched) {
      qos.keep_last(100);
    } else if (s->spec.heavy && profile_->options.heavy_qos == "best_effort") {
      qos.best_effort();
    }
    attach(*s);
    if (const auto m = std::find_if(
        catalog_->auto_markers.begin(), catalog_->auto_markers.end(),
        [&](const AutoMarker & a) {return a.topic == name;}); m != catalog_->auto_markers.end())
    {
      try {
        s->set_auto_marker(*m);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "auto marker on %s off: %s", name.c_str(), e.what());
      }
    }
    if (s->spec.record) {
      create_bag_topic(*s);
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = data_group_;
    // SCV saw Humble crash on QoS event handlers of subscriptions added while spinning.
    options.use_default_callbacks = false;
    std::weak_ptr<Stream> weak = s;
    try {
      s->subscription = create_generic_subscription(
        name, type, qos, [this, weak](std::shared_ptr<rclcpp::SerializedMessage> m) {
          if (auto stream = weak.lock()) {
            on_data(*stream, std::move(m), now().nanoseconds());
          }
        }, options);
    } catch (const std::exception & e) {
      s->error = "type unavailable: " + type;
      RCLCPP_WARN(get_logger(), "%s (%s): %s", name.c_str(), type.c_str(), e.what());
    }
  }

  void attach(Stream & s)
  {
    const rosbag_recorder_checks::TopicInfo topic{s.spec.name, s.spec.type, s.spec.latched};
    std::vector<std::shared_ptr<rosbag_recorder_checks::Check>> checks;
    for (const auto & [name, prototype] : check_prototypes_) {
      try {
        if (prototype->accepts(topic)) {
          auto check = check_loader_.createSharedInstance(name);
          check->configure(topic, catalog_->value_checks);
          checks.push_back(check);
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "check %s on %s off: %s", name.c_str(), s.spec.name.c_str(),
          e.what());
      }
    }
    s.set_checks(std::move(checks));
    if (!profile_->options.required_frames.empty() && s.spec.type == "tf2_msgs/msg/TFMessage") {
      s.collect_frames();
    }
  }

  // ---- data path (any thread) --------------------------------------------------------------

  void on_data(Stream & s, std::shared_ptr<rclcpp::SerializedMessage> message, std::int64_t time)
  {
    if (const auto change = s.receive(*message, mono_now(), wall_now())) {
      mark(s.auto_kind, *change, "auto", s.spec.name);
    }
    if (s.spec.latched) {
      s.keep_latched(message);
    }
    if (!s.spec.record || paused_) {
      return;
    }
    const std::shared_lock<std::shared_mutex> lock(writer_mutex_);
    if (writer_) {
      writer_->write(std::move(message), s.spec.name, s.spec.type, rclcpp::Time(time));
      s.count_written();
    }
  }

  void mark(
    const std::string & kind, const std::string & text, const std::string & source,
    const std::string & topic)
  {
    Marker m;
    m.stamp = now();
    m.kind = kind;
    m.text = text;
    m.source = source;
    m.topic = topic;
    {
      const std::shared_lock<std::shared_mutex> lock(writer_mutex_);
      m.session_id = session_id_;
      if (writer_) {
        writer_->write(m, kMarkerTopic, m.stamp);
        const std::lock_guard<std::mutex> markers_lock(markers_mutex_);
        markers_.push_back(m);
      }
    }
    marker_pub_->publish(m);
  }

  template<class MessageT>
  void record(const MessageT & message, const std::string & topic)
  {
    const std::shared_lock<std::shared_mutex> lock(writer_mutex_);
    if (writer_) {
      writer_->write(message, topic, now());
    }
  }

  // ---- control (control callback group) ----------------------------------------------------

  void tick()
  {
    const double begin = mono_now();
    const bool graph_changed = graph_event_->check_and_clear() || graph_dirty_;
    graph_dirty_ = false;
    if (graph_changed) {
      clock_published_ = count_publishers("/clock") > 0;
    }
    if (clock_published_) {
      sim_until_ = wall_now() + 5.0;
    }
    sim_time_ = wall_now() < sim_until_;
    if (profile_) {
      subscribe_pending(graph_changed);
    }
    report();
    if (state_ == Health::WAITING) {
      wait();
    } else if (state_ == Health::PREVIEW) {
      preflight_ = preflight();
    }
    tick_ms_ = (mono_now() - begin) * 1000.0;
  }

  void report()
  {
    const double mono = mono_now();
    const double wall = wall_now();
    Health h;
    h.stamp = now();
    h.state = state_;
    h.output_dir = output_dir_;
    h.tick_ms = tick_ms_;
    const auto free = disk_free_gb(output_dir_);
    h.disk_free_gb = free.value_or(std::numeric_limits<double>::quiet_NaN());
    h.remaining_min = std::numeric_limits<double>::quiet_NaN();
    if (profile_) {
      h.profile = profile_->name;
      h.disk_warn_gb = profile_->options.disk_warn_gb;
      h.disk_stop_gb = profile_->options.disk_stop_gb;
      if (free && *free < h.disk_warn_gb) {
        h.warnings.push_back(
          "disk free " + fixed(*free, 0) + " GB (warn " + fixed(h.disk_warn_gb, 0) + " GB)");
      }
    }
    const bool open = session_open();
    for (auto & [name, s] : streams_) {
      TopicHealth t = s->report(mono, wall, hz_tolerance_, sim_time_);
      if (open && s->spec.record) {
        value_marker(*s, t, wall);
      }
      if (s->spec.record) {
        h.topics.push_back(std::move(t));
      }
    }
    if (open) {
      h.session_id = session_id_;
      h.elapsed_s = std::round((mono - start_mono_) * 10) / 10;
      h.bag_bytes = directory_bytes(bag_path_);
      bytes_history_.emplace_back(mono, h.bag_bytes);
      if (bytes_history_.size() > 12) {
        bytes_history_.pop_front();
      }
      const auto [t0, b0] = bytes_history_.front();
      h.write_mbps = mono > t0 ?
        (static_cast<double>(h.bag_bytes) - static_cast<double>(b0)) / (mono - t0) / 1048576.0 :
        0.0;
      if (free && h.write_mbps > 0.01) {
        h.remaining_min = std::max(0.0, (*free - h.disk_stop_gb) * 1024.0 / h.write_mbps / 60.0);
      }
      record(h, kHealthTopic);
    }
    last_health_ = h;
    health_pub_->publish(h);
    if (open) {
      disk_low_ticks_ = free && *free < h.disk_stop_gb ? disk_low_ticks_ + 1 : 0;
      if (disk_low_ticks_ >= kDiskStopTicks) {
        mark("disk_stop", "disk free " + fixed(*free, 1) + " GB, recording stopped", "auto", "");
        finish("disk_low");
      }
    }
  }

  void value_marker(Stream & s, const TopicHealth & t, double wall)
  {
    const bool abnormal = t.status == TopicHealth::ABNORMAL;
    if (abnormal && !s.marker_open && wall - s.last_marker >= kMarkerGap) {
      std::string issues;
      for (const auto & i : t.issues) {
        issues += (issues.empty() ? "" : ", ") + i;
      }
      mark("value_issue", "value issue: " + t.name + ": " + issues, "auto", t.name);
      s.marker_open = true;
      s.last_marker = wall;
    } else if (!abnormal && s.marker_open) {
      mark("value_ok", "values normal again: " + t.name, "auto", t.name);
      s.marker_open = false;
    }
  }

  std::vector<std::string> required_missing() const
  {
    std::vector<std::string> missing;
    for (const auto & t : last_health_.topics) {
      if (t.required && t.status != TopicHealth::OK) {
        missing.push_back(t.name);
      }
    }
    return missing;
  }

  std::vector<CheckResult> preflight()
  {
    std::vector<CheckResult> out;
    const auto & o = profile_->options;
    if (!writable(output_dir_)) {
      out.push_back(result(CheckResult::ERROR, "cannot write to " + output_dir_));
    }
    const auto free = disk_free_gb(output_dir_);
    if (!free) {
      out.push_back(result(CheckResult::WARN, "disk free space unknown"));
    } else if (*free < o.disk_stop_gb) {
      out.push_back(
        result(
          CheckResult::ERROR, "disk free " + fixed(*free, 0) +
          " GB, below the stop limit " + fixed(o.disk_stop_gb, 0) + " GB"));
    } else if (*free < o.disk_warn_gb) {
      out.push_back(
        result(
          CheckResult::WARN, "disk free " + fixed(*free, 0) +
          " GB, below the warning limit " + fixed(o.disk_warn_gb, 0) + " GB"));
    } else {
      out.push_back(
        result(
          CheckResult::OK, "disk free " + fixed(*free, 0) + " GB (" +
          output_dir_ + ")"));
    }
    if (!profile_->unknown_topics.empty()) {
      out.push_back(
        result(
          CheckResult::WARN, std::to_string(profile_->unknown_topics.size()) +
          " topics not in the catalog, no reference rate: " + listed(profile_->unknown_topics)));
    }
    std::size_t required = 0;
    std::vector<std::string> no_type, silent, abnormal;
    for (const auto & t : last_health_.topics) {
      required += t.required;
      const bool no_data = t.status == TopicHealth::NO_PUBLISHER ||
        t.status == TopicHealth::WAITING || t.status == TopicHealth::STALE;
      if (t.status == TopicHealth::TYPE_UNAVAILABLE) {
        no_type.push_back(t.name);
      } else if (no_data) {
        silent.push_back(t.name);
      } else if (t.status == TopicHealth::ABNORMAL) {
        abnormal.push_back(t.name + " (" + (t.issues.empty() ? "" : t.issues.front()) + ")");
      }
    }
    const auto missing = required_missing();
    if (required > 0) {
      const std::string count =
        std::to_string(required - missing.size()) + "/" + std::to_string(required);
      out.push_back(
        missing.empty() ?
        result(CheckResult::OK, "required topics " + count + " ok") :
        result(
          CheckResult::WARN, "required topics " + count + " ok, not ready: " +
          listed(missing)));
    }
    if (!no_type.empty()) {
      out.push_back(
        result(
          CheckResult::WARN, "message type not available (source the "
          "workspace that defines it): " + listed(no_type)));
    }
    if (!silent.empty()) {
      out.push_back(
        result(
          CheckResult::INFO, std::to_string(silent.size()) + "/" +
          std::to_string(last_health_.topics.size()) + " topics without data: " + listed(silent)));
    }
    if (!abnormal.empty()) {
      out.push_back(result(CheckResult::WARN, "value issues: " + listed(abnormal)));
    }
    if (!o.required_frames.empty()) {
      std::set<std::string> frames;
      for (const auto & kv : streams_) {
        const auto f = kv.second->frames();
        frames.insert(f.begin(), f.end());
      }
      std::vector<std::string> absent;
      for (const auto & f : o.required_frames) {
        if (!frames.count(f)) {
          absent.push_back(f);
        }
      }
      out.push_back(
        absent.empty() ?
        result(CheckResult::OK, "TF frames present: " + listed(o.required_frames)) :
        result(CheckResult::WARN, "TF frames missing: " + listed(absent)));
    }
    return out;
  }

  std::string request_start(bool force)
  {
    const auto & o = profile_->options;
    if (force || o.start_mode == "immediate") {
      begin();
      return "recording " + session_id_;
    }
    const bool ready = o.start_mode == "wait_ready";
    wait_deadline_ = mono_now() + static_cast<double>(ready ? o.wait_ready_sec : o.start_delay_sec);
    state_ = Health::WAITING;
    message_ = ready ? "waiting up to " + std::to_string(o.wait_ready_sec) +
      " s for required topics" : "starting in " + std::to_string(o.start_delay_sec) + " s";
    return message_;
  }

  void wait()
  {
    const auto & o = profile_->options;
    const bool due = mono_now() >= wait_deadline_;
    const auto missing = required_missing();
    std::string note;
    if (o.start_mode == "delay" ? !due : (!missing.empty() && !due)) {
      return;
    }
    if (o.start_mode == "wait_ready" && !missing.empty()) {
      if (o.on_not_ready == "abort") {
        state_ = Health::PREVIEW;
        message_ = "required topics not ready after " + std::to_string(o.wait_ready_sec) +
          " s: " + listed(missing);
        RCLCPP_WARN(get_logger(), "%s", message_.c_str());
        return;
      }
      note = "started without required topics: " + listed(missing);
    }
    try {
      begin();
      if (!note.empty()) {
        mark("not_ready", note, "auto", "");
      }
    } catch (const std::exception & e) {
      state_ = Health::PREVIEW;
      message_ = std::string("start failed: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", message_.c_str());
    }
  }

  void begin()
  {
    const auto & o = profile_->options;
    const std::string id = session_id(profile_->name, label_, std::time(nullptr));
    fs::create_directories(output_dir_);
    std::string path = (fs::path(output_dir_) / id).string();
    for (int n = 2; fs::exists(path); ++n) {
      path = (fs::path(output_dir_) / (id + "_" + std::to_string(n))).string();
    }
    rosbag2_storage::StorageOptions storage;
    storage.uri = path;
    storage.storage_id = o.storage;
    storage.max_bagfile_duration = static_cast<std::uint64_t>(o.split_duration_sec);
    storage.max_bagfile_size = static_cast<std::uint64_t>(o.split_size_mb) * 1024 * 1024;
    storage.max_cache_size = static_cast<std::uint64_t>(o.max_cache_mb) * 1024 * 1024;
    storage.storage_preset_profile = o.preset();
    storage.storage_config_uri = o.storage_config_uri;
    set_custom_data(
      storage, {{"rosbag_recorder.session_id", fs::path(path).filename().string()},
        {"rosbag_recorder.profile", profile_->name}});
    std::shared_ptr<rosbag2_cpp::Writer> writer;
    if (o.storage == "sqlite3" && o.compression == "zstd") {
      rosbag2_compression::CompressionOptions compression;
      compression.compression_format = "zstd";
      compression.compression_mode = rosbag2_compression::CompressionMode::FILE;
      writer = std::make_shared<rosbag2_cpp::Writer>(
        std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression));
    } else {
      writer = std::make_shared<rosbag2_cpp::Writer>();
    }
    const std::string format = rmw_get_serialization_format();
    writer->open(storage, {format, format});
    for (const auto & [topic, type, qos] : {
        std::make_tuple(kSessionTopic, "rosbag_recorder_msgs/msg/Session", session_qos()),
        std::make_tuple(kHealthTopic, "rosbag_recorder_msgs/msg/Health", health_qos()),
        std::make_tuple(kMarkerTopic, "rosbag_recorder_msgs/msg/Marker", marker_qos())})
    {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic;
      meta.type = type;
      meta.serialization_format = format;
      set_offered_qos(meta, {qos});
      writer->create_topic(meta);
    }
    {
      const std::unique_lock<std::shared_mutex> lock(writer_mutex_);
      writer_ = writer;
      session_id_ = fs::path(path).filename().string();
      bag_path_ = path;
    }
    start_time_ = now();
    for (auto & kv : streams_) {
      Stream & s = *kv.second;
      s.reset_counts();
      s.marker_open = false;
      if (s.spec.record && !s.spec.type.empty() && (s.subscription || !s.spec.source.empty())) {
        create_bag_topic(s);
        for (auto & m : s.latched()) {
          writer->write(m, s.spec.name, s.spec.type, start_time_);
          s.count_written();
        }
      }
    }
    paused_ = false;
    start_mono_ = mono_now();
    disk_low_ticks_ = 0;
    bytes_history_.clear();
    {
      const std::lock_guard<std::mutex> lock(markers_mutex_);
      markers_.clear();
    }
    session_preflight_ = preflight();
    start_session_ = session_message(Session::START);
    session_pub_->publish(start_session_);
    record(start_session_, kSessionTopic);
    state_ = Health::RECORDING;
    message_ = "recording " + session_id_;
    RCLCPP_INFO(get_logger(), "recording %s", bag_path_.c_str());
  }

  void create_bag_topic(const Stream & s)
  {
    const std::shared_lock<std::shared_mutex> lock(writer_mutex_);
    if (!writer_) {
      return;
    }
    rosbag2_storage::TopicMetadata meta;
    meta.name = s.spec.name;
    meta.type = s.spec.type;
    meta.serialization_format = s.serialization_format;
    set_offered_qos(meta, s.offers);
    create_topic(*writer_, meta, s.definition, s.definition_encoding);
  }

  void finish(const std::string & reason)
  {
    std::shared_ptr<rosbag2_cpp::Writer> writer;
    Session end = session_message(Session::END);
    end.end_time = now();
    end.end_reason = reason;
    end.bag_bytes = directory_bytes(bag_path_);
    {
      const std::lock_guard<std::mutex> lock(markers_mutex_);
      end.markers = static_cast<std::uint32_t>(markers_.size());
    }
    if (get_node_base_interface()->get_context()->is_valid()) {
      session_pub_->publish(end);
    }
    record(end, kSessionTopic);
    {
      const std::unique_lock<std::shared_mutex> lock(writer_mutex_);
      writer = std::move(writer_);
    }
    writer->close();
    paused_ = false;
    state_ = profile_ ? Health::PREVIEW : Health::IDLE;
    message_ = "stopped " + session_id_ + " (" + reason + ")";
    write_summary(end);
    RCLCPP_INFO(get_logger(), "%s", message_.c_str());
  }

  Session session_message(std::uint8_t event)
  {
    Session s;
    s.event = event;
    s.session_id = session_id_;
    s.label = label_;
    s.notes = notes_;
    s.profile = profile_->name;
    s.bag_path = bag_path_;
    s.start_time = start_time_;
    if (event == Session::END) {
      return s;
    }
    s.profile_chain = profile_->chain;
    s.profile_yaml = profile_->to_yaml();
    s.config_dir = config_dir_;
    s.host = host_name();
    s.user = user_name();
    s.ros_distro = env("ROS_DISTRO");
    s.rmw_implementation = rmw_get_implementation_identifier();
    s.ros_domain_id = env("ROS_DOMAIN_ID");
    YAML::Node params;
    for (const auto & p : get_parameters(list_parameters({}, 0).names)) {
      params[p.get_name()] = p.value_to_string();
    }
    s.parameters_yaml = YAML::Dump(params);
    s.git_yaml = git_snapshot(workspace_);
    s.params = snapshot_files(
      workspace_, param_globs_, static_cast<std::uint64_t>(param_copy_max_kb_) * 1024);
    s.graph_nodes = get_node_names();
    for (const auto & [topic, types] : get_topic_names_and_types()) {
      for (const auto & type : types) {
        s.graph_topics.push_back(topic + " " + type);
      }
    }
    s.preflight = session_preflight_;
    return s;
  }

  void write_summary(const Session & end)
  {
    static const char * const kLevels[] = {"ok", "info", "warn", "error"};
    static const char * const kStatus[] = {"ok", "low", "stale", "waiting", "no_publisher",
      "latched_missing", "type_unavailable", "abnormal"};
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "session_id" << YAML::Value << end.session_id;
    out << YAML::Key << "profile" << YAML::Value << end.profile;
    out << YAML::Key << "label" << YAML::Value << end.label;
    out << YAML::Key << "notes" << YAML::Value << end.notes;
    out << YAML::Key << "host" << YAML::Value << start_session_.host;
    out << YAML::Key << "start" << YAML::Value << local_time(end.start_time);
    out << YAML::Key << "end" << YAML::Value << local_time(end.end_time);
    out << YAML::Key << "duration_s" << YAML::Value << fixed(mono_now() - start_mono_, 1);
    out << YAML::Key << "end_reason" << YAML::Value << end.end_reason;
    out << YAML::Key << "bag_bytes" << YAML::Value << directory_bytes(bag_path_);
    out << YAML::Key << "preflight" << YAML::Value << YAML::BeginSeq;
    for (const auto & c : session_preflight_) {
      out << std::string(kLevels[c.level]) + ": " + c.text;
    }
    out << YAML::EndSeq << YAML::Key << "topics" << YAML::Value << YAML::BeginMap;
    for (const auto & t : last_health_.topics) {
      out << YAML::Key << t.name << YAML::Value << YAML::Flow << YAML::BeginMap;
      out << YAML::Key << "type" << YAML::Value << t.type;
      out << YAML::Key << "written" << YAML::Value << streams_.at(t.name)->written();
      out << YAML::Key << "last_status" << YAML::Value << kStatus[t.status];
      out << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
    {
      const std::lock_guard<std::mutex> lock(markers_mutex_);
      for (const auto & m : markers_) {
        const double t = (rclcpp::Time(m.stamp) - rclcpp::Time(end.start_time)).seconds();
        out << fixed(t, 1) + " s [" + m.kind + "] " + m.text;
      }
    }
    out << YAML::EndSeq << YAML::EndMap;
    std::ofstream(fs::path(bag_path_) / "summary.yaml") << out.c_str() << "\n";
  }

  // ---- services ----------------------------------------------------------------------------

  void on_select(const srv::Select::Request & req, srv::Select::Response & res)
  {
    if (!idle_or_preview()) {
      res.message = "recording; stop first";
      return;
    }
    try {
      select(req.profile);
      res.success = true;
      res.message = message_;
      res.preflight = preflight_;
    } catch (const std::exception & e) {
      res.message = e.what();
    }
  }

  void on_start(const srv::Start::Request & req, srv::Start::Response & res)
  {
    if (!idle_or_preview()) {
      res.message = "already recording or waiting";
      return;
    }
    try {
      if (!req.profile.empty() && (!profile_ || req.profile != profile_->name)) {
        select(req.profile);
      }
      if (!profile_) {
        res.message = "no profile selected";
        return;
      }
      if (profile_->abstract) {
        res.message = profile_->name + " is abstract";
        return;
      }
      label_ = req.label;
      notes_ = req.notes;
      res.preflight = preflight();
      for (const auto & c : res.preflight) {
        if (c.level == CheckResult::ERROR) {
          res.message = "preflight: " + c.text;
          return;
        }
      }
      res.message = request_start(req.force);
      res.success = true;
      res.session_id = session_id_;
    } catch (const std::exception & e) {
      state_ = profile_ ? Health::PREVIEW : Health::IDLE;
      res.message = e.what();
    }
  }

  void on_stop(srv::Stop::Response & res)
  {
    if (state_ == Health::WAITING) {
      state_ = Health::PREVIEW;
      res.success = true;
      res.message = message_ = "start cancelled";
      return;
    }
    if (!session_open()) {
      res.message = "not recording";
      return;
    }
    finish("user");
    res.success = true;
    res.message = message_;
    res.session_id = session_id_;
    res.bag_path = bag_path_;
  }

  bool pause(bool on, std::string & message)
  {
    if (!session_open() || paused_ == on) {
      message = on ? "not recording" : "not paused";
      return false;
    }
    paused_ = on;
    state_ = on ? Health::PAUSED : Health::RECORDING;
    mark(on ? "pause" : "resume", on ? "recording paused" : "recording resumed", "user", "");
    message = on ? "paused" : "resumed";
    return true;
  }

  bool split(std::string & message)
  {
    {
      const std::shared_lock<std::shared_mutex> lock(writer_mutex_);
      if (!writer_) {
        message = "not recording";
        return false;
      }
      if (!rosbag_recorder::split(*writer_)) {
        message = "this rosbag2 cannot split on request";
        return false;
      }
    }
    mark("split", "bag file split", "user", "");
    message = "split";
    return true;
  }

  void on_status(srv::Status::Response & res)
  {
    res.state = state_;
    res.session_id = session_open() ? session_id_ : "";
    res.profile = profile_ ? profile_->name : "";
    res.bag_path = bag_path_;
    res.config_dir = config_dir_;
    res.message = message_;
    res.preflight = preflight_;
    const ProfileStore store((fs::path(config_dir_) / "profiles").string());
    std::map<std::string, std::pair<fs::file_time_type, bool>> listed;
    for (const auto & name : store.names()) {
      const auto mtime = fs::last_write_time(fs::path(config_dir_) / "profiles" / (name + ".yaml"));
      const auto cached = profile_list_.find(name);
      bool listable = false;
      if (cached != profile_list_.end() && cached->second.first == mtime) {
        listable = cached->second.second;
      } else {
        try {
          listable = !store.load(name).abstract;
        } catch (const ConfigError & e) {
          RCLCPP_WARN(get_logger(), "%s", e.what());
        }
      }
      listed[name] = {mtime, listable};
      if (listable) {
        res.profiles.push_back(name);
      }
    }
    profile_list_ = std::move(listed);
  }

  void on_catalog_diff(srv::CatalogDiff::Response & res)
  {
    Catalog catalog;
    try {
      catalog = Catalog::load((fs::path(config_dir_) / "catalog.yaml").string());
    } catch (const ConfigError & e) {
      res.message = e.what();
      return;
    }
    const auto graph = get_topic_names_and_types();
    for (const auto & [topic, types] : graph) {
      if (topic.rfind("/rosbag_recorder/", 0) == 0 || topic == "/rosout" ||
        topic == "/parameter_events")
      {
        continue;
      }
      const TopicSpec * spec = catalog.topic(topic);
      msg::CatalogDiffItem item;
      item.topic = topic;
      item.graph_type = types.front();
      item.publishers = static_cast<std::uint32_t>(count_publishers(topic));
      if (!spec) {
        item.kind = msg::CatalogDiffItem::NEW;
        res.items.push_back(item);
      } else if (spec->type && *spec->type != item.graph_type) {
        item.kind = msg::CatalogDiffItem::TYPE_CHANGED;
        item.catalog_type = *spec->type;
        res.items.push_back(item);
      }
    }
    for (const auto & [topic, spec] : catalog.topics) {
      if (!graph.count(topic)) {
        msg::CatalogDiffItem item;
        item.kind = msg::CatalogDiffItem::MISSING;
        item.topic = topic;
        item.catalog_type = spec.type.value_or("");
        res.items.push_back(item);
      }
    }
    res.success = true;
    res.message = std::to_string(res.items.size()) + " differences";
  }

  std::string config_dir_, output_dir_, workspace_;
  std::vector<std::string> param_globs_;
  std::int64_t param_copy_max_kb_{0};
  double hz_tolerance_{0.8};
  bool value_checks_{true};

  pluginlib::ClassLoader<rosbag_recorder_checks::Check> check_loader_;
  pluginlib::ClassLoader<rosbag_recorder_source::Source> source_loader_;
  std::map<std::string, std::shared_ptr<rosbag_recorder_checks::Check>> check_prototypes_;

  std::optional<Catalog> catalog_;
  std::optional<ResolvedProfile> profile_;
  std::map<std::string, std::shared_ptr<Stream>> streams_;
  std::vector<LoadedSource> sources_;

  std::atomic<std::uint8_t> state_{Health::IDLE};
  std::string message_;
  std::vector<CheckResult> preflight_;
  std::string label_, notes_;
  double wait_deadline_{0.0};
  bool sim_time_{false};
  double sim_until_{0.0};
  Health last_health_;
  double tick_ms_{0.0};
  rclcpp::Event::SharedPtr graph_event_;
  bool graph_dirty_{true};
  bool clock_published_{false};
  std::map<std::string, std::pair<fs::file_time_type, bool>> profile_list_;

  std::shared_mutex writer_mutex_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  std::atomic<bool> paused_{false};
  std::string session_id_, bag_path_;
  rclcpp::Time start_time_;
  double start_mono_{0.0};
  std::vector<CheckResult> session_preflight_;
  Session start_session_;
  std::mutex markers_mutex_;
  std::vector<Marker> markers_;
  std::deque<std::pair<double, std::uint64_t>> bytes_history_;
  int disk_low_ticks_{0};

  rclcpp::CallbackGroup::SharedPtr control_group_, data_group_;
  rclcpp::Publisher<Session>::SharedPtr session_pub_;
  rclcpp::Publisher<Health>::SharedPtr health_pub_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::Service<srv::Select>::SharedPtr select_srv_;
  rclcpp::Service<srv::Start>::SharedPtr start_srv_;
  rclcpp::Service<srv::Stop>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_, resume_srv_, split_srv_;
  rclcpp::Service<srv::Mark>::SharedPtr mark_srv_;
  rclcpp::Service<srv::Status>::SharedPtr status_srv_;
  rclcpp::Service<srv::CatalogDiff>::SharedPtr diff_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rosbag_recorder

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag_recorder::RecorderNode)
