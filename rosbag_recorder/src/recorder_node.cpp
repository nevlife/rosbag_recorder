#include "catalog_diff.hpp"
#include "compat.hpp"
#include "config.hpp"
#include "disk_watch.hpp"
#include "flux/schemas.hpp"
#include "flux/topics.hpp"
#include "flux_input.hpp"
#include "preflight.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rmw/rmw.h"
#include "rosbag2_compression/compression_options.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "selection.hpp"
#include "session_info.hpp"
#include "start_gate.hpp"
#include "stream.hpp"
#include "summary.hpp"
#include "text.hpp"
#include "yaml-cpp/yaml.h"

#include "rosbag_recorder_msgs/msg/check_result.hpp"
#include "rosbag_recorder_msgs/msg/health.hpp"
#include "rosbag_recorder_msgs/msg/marker.hpp"
#include "rosbag_recorder_msgs/msg/session.hpp"
#include "rosbag_recorder_msgs/srv/catalog_diff.hpp"
#include "rosbag_recorder_msgs/srv/list_topics.hpp"
#include "rosbag_recorder_msgs/srv/mark.hpp"
#include "rosbag_recorder_msgs/srv/select.hpp"
#include "rosbag_recorder_msgs/srv/set_topics.hpp"
#include "rosbag_recorder_msgs/srv/start.hpp"
#include "rosbag_recorder_msgs/srv/status.hpp"
#include "rosbag_recorder_msgs/srv/stop.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rosbag_recorder
{
namespace
{

namespace fs = std::filesystem;
namespace msg = rosbag_recorder_msgs::msg;
namespace srv = rosbag_recorder_msgs::srv;

double mono_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

rclcpp::QoS session_qos()
{
  return rclcpp::QoS(2).reliable().transient_local();
}
rclcpp::QoS health_qos()
{
  return rclcpp::QoS(10).reliable();
}
rclcpp::QoS marker_qos()
{
  return rclcpp::QoS(100).reliable();
}

}  // namespace

class RecorderNode : public rclcpp::Node
{
  using Health = msg::Health;
  using Marker = msg::Marker;
  using Session = msg::Session;

public:
  explicit RecorderNode(const rclcpp::NodeOptions & options) : Node("rosbag_recorder", options)
  {
    config_dir_ = expand_user(declare_parameter<std::string>("config_dir"));
    output_dir_ = expand_user(declare_parameter<std::string>("output_dir"));
    workspace_ = expand_user(declare_parameter<std::string>("workspace"));
    // A ROS parameter file cannot hold an empty list, so this one keeps an empty default.
    param_globs_ = declare_parameter("param_globs", std::vector<std::string>{});
    param_copy_max_kb_ = declare_parameter<std::int64_t>("param_copy_max_kb");
    hz_tolerance_ = declare_parameter<double>("hz_tolerance");
    if (hz_tolerance_ < 0.1 || hz_tolerance_ > 1.0) {
      throw std::invalid_argument("hz_tolerance must be within [0.1, 1.0]");
    }
    const auto profile = declare_parameter<std::string>("profile");
    const bool autostart = declare_parameter<bool>("autostart");
    label_ = declare_parameter<std::string>("label");

    session_pub_ = create_publisher<Session>("~/output/session", session_qos());
    health_pub_ = create_publisher<Health>("~/output/health", health_qos());
    marker_pub_ = create_publisher<Marker>("~/output/markers", marker_qos());
    services();
    graph_event_ = get_graph_event();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { tick(); });

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
    if (writer_) {
      finish("shutdown");
    }
    unselect();
  }

private:
  void services()
  {
    const auto qos = service_qos();
    topics_srv_ = create_service<srv::ListTopics>(
      "~/topics",
      [this](
        const std::shared_ptr<srv::ListTopics::Request>,
        std::shared_ptr<srv::ListTopics::Response> res) { on_topics(*res); },
      qos);
    set_topics_srv_ = create_service<srv::SetTopics>(
      "~/set_topics",
      [this](
        const std::shared_ptr<srv::SetTopics::Request> req,
        std::shared_ptr<srv::SetTopics::Response> res) { on_set_topics(*req, *res); },
      qos);
    select_srv_ = create_service<srv::Select>(
      "~/select",
      [this](
        const std::shared_ptr<srv::Select::Request> req,
        std::shared_ptr<srv::Select::Response> res) { on_select(*req, *res); },
      qos);
    start_srv_ = create_service<srv::Start>(
      "~/start",
      [this](
        const std::shared_ptr<srv::Start::Request> req, std::shared_ptr<srv::Start::Response> res) {
        on_start(*req, *res);
      },
      qos);
    stop_srv_ = create_service<srv::Stop>(
      "~/stop",
      [this](const std::shared_ptr<srv::Stop::Request>, std::shared_ptr<srv::Stop::Response> res) {
        on_stop(*res);
      },
      qos);
    pause_srv_ = trigger("~/pause", [this](std::string & m) { return pause(true, m); });
    resume_srv_ = trigger("~/resume", [this](std::string & m) { return pause(false, m); });
    split_srv_ = trigger("~/split", [this](std::string & m) { return split(m); });
    mark_srv_ = create_service<srv::Mark>(
      "~/mark",
      [this](
        const std::shared_ptr<srv::Mark::Request> req, std::shared_ptr<srv::Mark::Response> res) {
        on_mark(*req, *res);
      },
      qos);
    status_srv_ = create_service<srv::Status>(
      "~/status",
      [this](
        const std::shared_ptr<srv::Status::Request>, std::shared_ptr<srv::Status::Response> res) {
        on_status(*res);
      },
      qos);
    diff_srv_ = create_service<srv::CatalogDiff>(
      "~/catalog_diff",
      [this](
        const std::shared_ptr<srv::CatalogDiff::Request>,
        std::shared_ptr<srv::CatalogDiff::Response> res) { on_catalog_diff(*res); },
      qos);
  }

  template <class F>
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger(const std::string & name, F f)
  {
    return create_service<std_srvs::srv::Trigger>(
      name,
      [f](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) { res->success = f(res->message); },
      service_qos());
  }

  bool idle_or_preview() const { return state_ == Health::IDLE || state_ == Health::PREVIEW; }

  bool hidden(const std::string & topic) const
  {
    return topic == session_pub_->get_topic_name() || topic == health_pub_->get_topic_name() ||
           topic == marker_pub_->get_topic_name() || topic == "/rosout" ||
           topic == "/parameter_events";
  }

  Catalog load_catalog() const
  {
    return Catalog::load((fs::path(config_dir_) / "catalog.yaml").string());
  }

  // ---- selection ---------------------------------------------------------------------------

  void refresh_topics(bool graph_changed)
  {
    if (graph_changed) {
      ros_topics_ = get_topic_names_and_types();
    }
    flux_topics_.clear();
    for (auto & topic : flux::enumerate_topics()) {
      if (!topic.key_exact || topic.key.empty() || topic.key.front() != '/') {
        continue;
      }
      if (flux::read_channel_stats(topic.signpost).live) {
        flux_topics_.emplace(topic.key, std::move(topic));
      }
    }
  }

  std::string flux_type(std::uint64_t fingerprint) const
  {
    return schemas_.contains(fingerprint) ? schemas_.at(fingerprint).ros_type() : "raw";
  }

  void add_stream(
    const std::string & name, const std::string & transport, std::uint64_t fingerprint = 0)
  {
    const std::string bag = bag_name(name, transport);
    StreamSpec spec{bag, "", "", std::nullopt, false, false, false};
    if (const auto * entry = catalog_->topic(name)) {
      spec.type = entry->type.value_or("");
      spec.hz = entry->hz;
      spec.required = entry->required;
      spec.latched = entry->latched;
      spec.heavy = entry->heavy;
    }
    if (transport == "flux") {
      spec.type = "flux_msgs/msg/FluxFrame";
      spec.source = "flux";
      spec.latched = false;
    }
    auto stream = std::make_shared<Stream>(spec);
    streams_[bag] = stream;
    if (transport != "flux") {
      return;
    }
    stream->publishers = flux_topics_.count(name) ? 1 : 0;
    stream->offers = {rclcpp::QoS(10)};
    auto & input = flux_subscriptions_[{name, fingerprint}];
    if (!input) {
      input = std::make_unique<FluxInput>(*this, name, fingerprint);
    }
    input->set_callback(
      [this, stream](std::shared_ptr<rclcpp::SerializedMessage> bytes, std::int64_t stamp) {
        on_data(*stream, std::move(bytes), stamp);
      });
    flux_inputs_[bag] = input.get();
  }

  void on_topics(srv::ListTopics::Response & res)
  {
    try {
      refresh_topics(true);
      const auto catalog = load_catalog();
      for (const auto & [name, types] : ros_topics_) {
        if (types.empty() || hidden(name)) {
          continue;
        }
        const auto publishers = static_cast<std::uint32_t>(count_publishers(name));
        if (publishers == 0 && !streams_.count(name)) {
          continue;
        }
        msg::AvailableTopic topic;
        topic.name = name;
        topic.transport = "ros";
        topic.type = types.front();
        topic.publishers = publishers;
        topic.selected = streams_.count(name) && streams_.at(name)->spec.source.empty();
        res.topics.push_back(topic);
      }
      for (const auto & [name, entry] : flux_topics_) {
        res.topics.push_back(flux_topic(name, entry.fingerprint, 1));
      }
      for (const auto & [bag, input] : flux_inputs_) {
        const auto name = bag.substr(0, bag.size() - 5);
        if (!flux_topics_.count(name)) {
          res.topics.push_back(flux_topic(name, input->fingerprint(), 0));
        }
      }
      for (auto & topic : res.topics) {
        if (const auto * spec = catalog.topic(topic.name)) {
          topic.group = spec->group;
        }
      }
      res.success = true;
    } catch (const std::exception & e) {
      res.message = e.what();
    }
  }

  msg::AvailableTopic flux_topic(
    const std::string & name, std::uint64_t fingerprint, std::uint32_t publishers) const
  {
    msg::AvailableTopic topic;
    topic.name = name;
    topic.transport = "flux";
    topic.type = flux_type(fingerprint);
    topic.fingerprint = fingerprint;
    topic.publishers = publishers;
    topic.selected = flux_inputs_.count(bag_name(name, "flux")) > 0;
    return topic;
  }

  void on_set_topics(const srv::SetTopics::Request & req, srv::SetTopics::Response & res)
  {
    if (!idle_or_preview()) {
      res.message = "recording; stop before changing topics";
      return;
    }
    try {
      refresh_topics(true);
      auto catalog = load_catalog();
      std::set<std::string> ros, flux;
      for (const auto & kv : ros_topics_) {
        ros.insert(kv.first);
      }
      std::map<std::string, std::uint64_t> fingerprints;
      for (const auto & [name, topic] : flux_topics_) {
        fingerprints[name] = topic.fingerprint;
      }
      for (const auto & [bag, input] : flux_inputs_) {
        fingerprints.emplace(bag.substr(0, bag.size() - 5), input->fingerprint());
      }
      for (const auto & kv : fingerprints) {
        flux.insert(kv.first);
      }
      check_selection(req.topics, ros, flux);
      unselect();
      catalog_ = std::move(catalog);
      profile_ = ResolvedProfile{};
      profile_->name = "default";
      profile_->options.start_mode = "immediate";
      for (const auto & topic : req.topics) {
        add_stream(topic.name, topic.transport, fingerprints[topic.name]);
        profile_->topics.push_back(bag_name(topic.name, topic.transport));
      }
      enter_preview("selected " + std::to_string(streams_.size()) + " topics");
      res.success = true;
      res.message = message_;
      res.preflight = preflight_;
    } catch (const std::exception & e) {
      res.message = e.what();
    }
  }

  void select(const std::string & name)
  {
    unselect();
    Catalog catalog = load_catalog();
    ResolvedProfile profile =
      ProfileStore((fs::path(config_dir_) / "profiles").string()).resolve(name, catalog);
    catalog_ = std::move(catalog);
    profile_ = std::move(profile);
    std::vector<std::string> topics = profile_->topics;
    topics.insert(topics.end(), profile_->unknown_topics.begin(), profile_->unknown_topics.end());
    for (const auto & t : topics) {
      add_stream(t, "ros");
    }
    enter_preview("selected " + name);
    RCLCPP_INFO(get_logger(), "profile %s: %zu topics", name.c_str(), topics.size());
  }

  void enter_preview(const std::string & message)
  {
    state_ = Health::PREVIEW;
    message_ = message;
    subscribe_pending(true);
    for (const auto & [name, stream] : streams_) {
      last_health_.topics.push_back(stream->report(mono_now(), hz_tolerance_));
    }
    preflight_ = preflight_now();
  }

  void unselect()
  {
    for (auto & [bag, input] : flux_inputs_) {
      input->set_callback({});
    }
    flux_inputs_.clear();
    streams_.clear();
    profile_.reset();
    catalog_.reset();
    last_health_ = Health{};
    preflight_.clear();
    state_ = Health::IDLE;
  }

  // Graph queries cost O(graph) each; they run when the graph changed, not every tick.
  void subscribe_pending(bool graph_changed)
  {
    for (auto & [bag, input] : flux_inputs_) {
      streams_.at(bag)->publishers = flux_topics_.count(bag.substr(0, bag.size() - 5)) ? 1 : 0;
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
    const auto graph =
      pending ? get_topic_names_and_types() : std::map<std::string, std::vector<std::string>>{};
    for (auto & [name, s] : streams_) {
      if (!s->spec.source.empty() || !s->error.empty() || s->publishers == 0) {
        continue;
      }
      if (s->subscription) {
        subscribe(s, s->spec.type);
        continue;
      }
      const auto it = graph.find(name);
      if (it != graph.end() && !it->second.empty()) {
        subscribe(s, it->second.front());
      }
    }
  }

  void subscribe(const std::shared_ptr<Stream> & s, const std::string & type)
  {
    const std::string & name = s->spec.name;
    const bool initial = !s->subscription;
    if (initial) {
      s->spec.type = type;
    }
    const auto publishers = get_publishers_info_by_topic(name);
    rclcpp::QoS qos = Rosbag2QoS::adapt_request_to_offers(name, publishers);
    bool all_latched = !publishers.empty();
    s->offers.clear();
    for (const auto & p : publishers) {
      all_latched =
        all_latched && p.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
      s->offers.push_back(p.qos_profile());
    }
    if (initial) {
      s->spec.latched = s->spec.latched || all_latched;
    }
    if (s->spec.latched) {
      qos.keep_last(100);
    } else if (s->spec.heavy && profile_->options.heavy_qos == "best_effort") {
      qos.best_effort();
    }
    if (s->subscription && s->subscription_qos && *s->subscription_qos == qos) {
      return;
    }
    if (
      initial && !profile_->options.required_frames.empty() &&
      s->spec.type == "tf2_msgs/msg/TFMessage") {
      s->collect_frames();
    }
    if (writer_) {
      create_bag_topic(*s, *writer_);
    }
    create_subscription(s, type, qos);
  }

  void create_subscription(
    const std::shared_ptr<Stream> & s, const std::string & type, const rclcpp::QoS & qos)
  {
    rclcpp::SubscriptionOptions options;
    // SCV saw Humble crash on QoS event handlers of subscriptions added while spinning.
    options.use_default_callbacks = false;
    std::weak_ptr<Stream> weak = s;
    s->subscription.reset();
    try {
      s->subscription = create_generic_subscription(
        s->spec.name, type, qos,
        [this, weak](std::shared_ptr<rclcpp::SerializedMessage> m) {
          if (auto stream = weak.lock()) {
            on_data(*stream, std::move(m), now().nanoseconds());
          }
        },
        options);
      s->subscription_qos = qos;
    } catch (const std::exception & e) {
      s->error = "type unavailable: " + type;
      RCLCPP_WARN(get_logger(), "%s (%s): %s", s->spec.name.c_str(), type.c_str(), e.what());
    }
  }

  // ---- data path ---------------------------------------------------------------------------

  void on_data(Stream & s, std::shared_ptr<rclcpp::SerializedMessage> message, std::int64_t time)
  {
    s.receive(*message, mono_now());
    if (s.spec.latched) {
      s.keep_latched(message);
    }
    if (writer_ && !paused_) {
      writer_->write(std::move(message), s.spec.name, s.spec.type, rclcpp::Time(time));
      s.count_written();
    }
  }

  void mark(const std::string & kind, const std::string & text, const std::string & source)
  {
    Marker m;
    m.stamp = now();
    m.kind = kind;
    m.text = text;
    m.source = source;
    m.session_id = session_id_;
    if (writer_) {
      writer_->write(m, marker_pub_->get_topic_name(), m.stamp);
      markers_.push_back(m);
    }
    marker_pub_->publish(m);
  }

  template <class MessageT>
  void record(const MessageT & message, const std::string & topic)
  {
    if (writer_) {
      writer_->write(message, topic, now());
    }
  }

  // ---- control -----------------------------------------------------------------------------

  void tick()
  {
    const double begin = mono_now();
    const bool graph_changed = graph_event_->check_and_clear() || graph_dirty_;
    graph_dirty_ = false;
    graph_reads_ += graph_changed;
    refresh_topics(graph_changed);
    if (profile_) {
      subscribe_pending(graph_changed);
    }
    report();
    if (state_ == Health::WAITING) {
      wait();
    } else if (state_ == Health::PREVIEW) {
      preflight_ = preflight_now();
    }
    tick_ms_ = (mono_now() - begin) * 1000.0;
  }

  void report()
  {
    const double mono = mono_now();
    Health h;
    h.stamp = now();
    h.state = state_;
    h.output_dir = output_dir_;
    h.tick_ms = tick_ms_;
    h.graph_reads = graph_reads_;
    h.disk_free_gb = disk_free_gb(output_dir_);
    h.remaining_min = std::numeric_limits<double>::quiet_NaN();
    if (profile_) {
      h.profile = profile_->name;
      h.disk_warn_gb = profile_->options.disk_warn_gb;
      h.disk_stop_gb = profile_->options.disk_stop_gb;
      if (h.disk_free_gb < h.disk_warn_gb) {
        h.warnings.push_back(
          "disk free " + fixed(h.disk_free_gb, 0) + " GB (warn " + fixed(h.disk_warn_gb, 0) +
          " GB)");
      }
    }
    for (const auto & [name, s] : streams_) {
      h.topics.push_back(s->report(mono, hz_tolerance_));
    }
    bool disk_stop = false;
    if (writer_) {
      h.session_id = session_id_;
      h.elapsed_s = std::round((mono - start_mono_) * 10) / 10;
      h.bag_bytes = directory_bytes(bag_path_);
      const auto disk = disk_watch_.update(mono, h.bag_bytes, h.disk_free_gb, h.disk_stop_gb);
      h.write_mbps = disk.write_mbps;
      h.remaining_min = disk.remaining_min;
      disk_stop = disk.stop;
      record(h, health_pub_->get_topic_name());
    }
    last_health_ = h;
    health_pub_->publish(h);
    if (disk_stop) {
      mark("disk_stop", "disk free " + fixed(h.disk_free_gb, 1) + " GB, recording stopped", "auto");
      finish("disk_low");
    }
  }

  std::vector<CheckResult> preflight_now() const
  {
    const auto & o = profile_->options;
    PreflightInput in;
    in.output_dir = output_dir_;
    in.writable = writable(output_dir_);
    in.disk_free_gb = disk_free_gb(output_dir_);
    in.disk_warn_gb = o.disk_warn_gb;
    in.disk_stop_gb = o.disk_stop_gb;
    in.unknown_topics = profile_->unknown_topics;
    in.topics = last_health_.topics;
    in.required_frames = o.required_frames;
    for (const auto & kv : streams_) {
      in.frames.insert(kv.second->frames().begin(), kv.second->frames().end());
    }
    return preflight(in);
  }

  std::string request_start(bool force)
  {
    preflight_ = preflight_now();
    if (start_gate_.request(*profile_, streams_.size(), preflight_, force, mono_now())) {
      begin();
      return "recording " + session_id_;
    }
    state_ = Health::WAITING;
    message_ = start_gate_.message();
    return message_;
  }

  void wait()
  {
    const auto step =
      start_gate_.poll(profile_->options, mono_now(), required_missing(last_health_.topics));
    if (step == StartGate::Step::Wait) {
      return;
    }
    if (step == StartGate::Step::Abort) {
      state_ = Health::PREVIEW;
      message_ = start_gate_.message();
      RCLCPP_WARN(get_logger(), "%s", message_.c_str());
      return;
    }
    const std::string note = start_gate_.message();
    try {
      begin();
      if (!note.empty()) {
        mark("not_ready", note, "auto");
      }
    } catch (const std::exception & e) {
      state_ = Health::PREVIEW;
      message_ = std::string("start failed: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", message_.c_str());
    }
  }

  void begin()
  {
    const std::string id = session_id(profile_->name, label_, std::time(nullptr));
    fs::create_directories(output_dir_);
    std::string path = (fs::path(output_dir_) / id).string();
    for (int n = 2; fs::exists(path); ++n) {
      path = (fs::path(output_dir_) / (id + "_" + std::to_string(n))).string();
    }
    session_id_ = fs::path(path).filename().string();
    bag_path_ = path;
    start_time_ = now();
    session_preflight_ = preflight_now();
    // Built before the bag opens: the git and file snapshots can throw.
    start_session_ = session_message(Session::START);
    writer_ = open_writer(path);
    for (auto & kv : streams_) {
      Stream & s = *kv.second;
      s.reset_counts();
      if (!s.spec.type.empty() && (s.subscription || !s.spec.source.empty())) {
        create_bag_topic(s, *writer_);
        for (const auto & m : s.latched()) {
          writer_->write(m, s.spec.name, s.spec.type, start_time_);
          s.count_written();
        }
      }
    }
    paused_ = false;
    start_mono_ = mono_now();
    disk_watch_.reset();
    markers_.clear();
    session_pub_->publish(start_session_);
    record(start_session_, session_pub_->get_topic_name());
    state_ = Health::RECORDING;
    message_ = "recording " + session_id_;
    RCLCPP_INFO(get_logger(), "recording %s", bag_path_.c_str());
  }

  std::shared_ptr<rosbag2_cpp::Writer> open_writer(const std::string & path)
  {
    const auto & o = profile_->options;
    rosbag2_storage::StorageOptions storage;
    storage.uri = path;
    storage.storage_id = o.storage;
    storage.max_bagfile_duration = static_cast<std::uint64_t>(o.split_duration_sec);
    storage.max_bagfile_size = static_cast<std::uint64_t>(o.split_size_mb) * 1024 * 1024;
    storage.max_cache_size = static_cast<std::uint64_t>(o.max_cache_mb) * 1024 * 1024;
    storage.storage_preset_profile = o.preset();
    storage.storage_config_uri = o.storage_config_uri;
    set_custom_data(
      storage,
      {{"rosbag_recorder.session_id", session_id_}, {"rosbag_recorder.profile", profile_->name}});
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
    for (const auto & [topic, type, qos] :
         {std::make_tuple(
            session_pub_->get_topic_name(), "rosbag_recorder_msgs/msg/Session", session_qos()),
          std::make_tuple(
            health_pub_->get_topic_name(), "rosbag_recorder_msgs/msg/Health", health_qos()),
          std::make_tuple(
            marker_pub_->get_topic_name(), "rosbag_recorder_msgs/msg/Marker", marker_qos())}) {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic;
      meta.type = type;
      meta.serialization_format = format;
      set_offered_qos(meta, {qos});
      writer->create_topic(meta);
    }
    return writer;
  }

  void create_bag_topic(const Stream & s, rosbag2_cpp::Writer & writer)
  {
    rosbag2_storage::TopicMetadata meta;
    meta.name = s.spec.name;
    meta.type = s.spec.type;
    meta.serialization_format = rmw_get_serialization_format();
    set_offered_qos(meta, s.offers);
    writer.create_topic(meta);
  }

  void finish(const std::string & reason)
  {
    Session end = session_message(Session::END);
    end.end_time = now();
    end.end_reason = reason;
    end.bag_bytes = directory_bytes(bag_path_);
    end.markers = static_cast<std::uint32_t>(markers_.size());
    if (get_node_base_interface()->get_context()->is_valid()) {
      session_pub_->publish(end);
    }
    record(end, session_pub_->get_topic_name());
    writer_->close();
    writer_.reset();
    paused_ = false;
    state_ = profile_ ? Health::PREVIEW : Health::IDLE;
    message_ = "stopped " + session_id_ + " (" + reason + ")";
    Summary summary{
      end,
      start_session_.host,
      mono_now() - start_mono_,
      directory_bytes(bag_path_),
      session_preflight_,
      {},
      markers_};
    for (const auto & [name, stream] : streams_) {
      summary.topics.push_back(stream->report(mono_now(), hz_tolerance_));
    }
    std::ofstream(fs::path(bag_path_) / "summary.yaml") << summary_yaml(summary);
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
      label_ = req.label;
      notes_ = req.notes;
      res.message = request_start(req.force);
      res.preflight = preflight_;
      res.success = true;
      res.session_id = session_id_;
    } catch (const std::exception & e) {
      state_ = profile_ ? Health::PREVIEW : Health::IDLE;
      res.preflight = preflight_;
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
    if (!writer_) {
      res.message = "not recording";
      return;
    }
    finish("user");
    res.success = true;
    res.message = message_;
    res.session_id = session_id_;
    res.bag_path = bag_path_;
  }

  void on_mark(const srv::Mark::Request & req, srv::Mark::Response & res)
  {
    res.success = writer_ != nullptr;
    res.message = res.success ? "marked" : "not recording";
    if (res.success) {
      mark(req.kind.empty() ? "note" : req.kind, req.text, "user");
    }
  }

  bool pause(bool on, std::string & message)
  {
    if (!writer_ || paused_ == on) {
      message = on ? "not recording" : "not paused";
      return false;
    }
    paused_ = on;
    state_ = on ? Health::PAUSED : Health::RECORDING;
    mark(on ? "pause" : "resume", on ? "recording paused" : "recording resumed", "user");
    message = on ? "paused" : "resumed";
    return true;
  }

  bool split(std::string & message)
  {
    if (!writer_) {
      message = "not recording";
      return false;
    }
    if (!rosbag_recorder::split(*writer_)) {
      message = "this rosbag2 cannot split on request";
      return false;
    }
    mark("split", "bag file split", "user");
    message = "split";
    return true;
  }

  void on_status(srv::Status::Response & res)
  {
    res.state = state_;
    res.session_id = writer_ ? session_id_ : "";
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
      catalog = load_catalog();
    } catch (const ConfigError & e) {
      res.message = e.what();
      return;
    }
    std::map<std::string, std::vector<std::string>> graph;
    for (auto & [topic, types] : get_topic_names_and_types()) {
      if (!hidden(topic)) {
        graph[topic] = std::move(types);
      }
    }
    res.items = catalog_diff(catalog, graph);
    for (auto & item : res.items) {
      if (item.kind != msg::CatalogDiffItem::MISSING) {
        item.publishers = static_cast<std::uint32_t>(count_publishers(item.topic));
      }
    }
    res.success = true;
    res.message = std::to_string(res.items.size()) + " differences";
  }

  std::string config_dir_, output_dir_, workspace_;
  std::vector<std::string> param_globs_;
  std::int64_t param_copy_max_kb_{0};
  double hz_tolerance_{0.0};

  std::optional<Catalog> catalog_;
  std::optional<ResolvedProfile> profile_;
  std::map<std::string, std::shared_ptr<Stream>> streams_;
  flux::Schemas schemas_;
  std::map<std::string, flux::Topic> flux_topics_;
  std::map<std::string, std::vector<std::string>> ros_topics_;
  // Every Flux subscription made so far, by topic and fingerprint; see FluxInput.
  std::map<std::pair<std::string, std::uint64_t>, std::unique_ptr<FluxInput>> flux_subscriptions_;
  // The selected ones, by bag topic.
  std::map<std::string, FluxInput *> flux_inputs_;

  std::uint8_t state_{Health::IDLE};
  std::string message_;
  std::vector<CheckResult> preflight_;
  std::string label_, notes_;
  StartGate start_gate_;
  Health last_health_;
  double tick_ms_{0.0};
  rclcpp::Event::SharedPtr graph_event_;
  bool graph_dirty_{true};
  std::uint64_t graph_reads_{0};
  std::map<std::string, std::pair<fs::file_time_type, bool>> profile_list_;

  std::shared_ptr<rosbag2_cpp::Writer> writer_;
  bool paused_{false};
  std::string session_id_, bag_path_;
  rclcpp::Time start_time_;
  double start_mono_{0.0};
  std::vector<CheckResult> session_preflight_;
  Session start_session_;
  std::vector<Marker> markers_;
  DiskWatch disk_watch_;

  rclcpp::Publisher<Session>::SharedPtr session_pub_;
  rclcpp::Publisher<Health>::SharedPtr health_pub_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::Service<srv::ListTopics>::SharedPtr topics_srv_;
  rclcpp::Service<srv::SetTopics>::SharedPtr set_topics_srv_;
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
