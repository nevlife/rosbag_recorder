#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "rosbag_recorder/config.hpp"

using rosbag_recorder::Catalog;
using rosbag_recorder::ConfigError;
using rosbag_recorder::Profile;
using rosbag_recorder::ProfileStore;

namespace
{

const char kScv[] = TEST_DATA_DIR "/scv";

Catalog catalog(const std::string & text)
{
  return Catalog::parse(YAML::Load(text), "catalog");
}

const char * const kSmall =
  "groups:\n"
  "  a:\n"
  "    topics:\n"
  "      /a1: {type: std_msgs/msg/String, hz: 10, required: true}\n"
  "      /a2: {}\n"
  "  b:\n"
  "    topics:\n"
  "      /b1:\n"
  "sources:\n"
  "  counter:\n"
  "    plugin: test/Counter\n"
  "    params: {rate_hz: 5}\n"
  "    channels:\n"
  "      /counter/value: {hz: 5, required: true}\n";

void expect_error(const std::string & text, const std::string & part)
{
  try {
    catalog(text);
    ADD_FAILURE() << "no error for: " << text;
  } catch (const ConfigError & e) {
    EXPECT_NE(std::string(e.what()).find(part), std::string::npos) << e.what();
  }
}

}  // namespace

TEST(ScvConfig, CatalogFields)
{
  const Catalog c = Catalog::load(std::string(kScv) + "/catalog.yaml");
  EXPECT_EQ(c.groups.size(), 11U);
  EXPECT_EQ(c.groups.front().name, "lidar");
  EXPECT_EQ(c.auto_markers.size(), 3U);
  EXPECT_EQ(c.auto_markers[0].topic, "/vehicle/mux_status");
  EXPECT_EQ(c.auto_markers[0].kind, "auto");
  EXPECT_DOUBLE_EQ(c.value_checks.at("gnss_max_std_m"), 10.0);
  const auto * packets = c.topic("/velodyne_packets");
  ASSERT_TRUE(packets);
  EXPECT_EQ(*packets->type, "velodyne_msgs/msg/VelodyneScan");
  EXPECT_TRUE(packets->required);
  EXPECT_DOUBLE_EQ(*packets->hz, 10.0);
  const auto * raw = c.topic("/camera/camera/color/image_raw");
  EXPECT_TRUE(raw->heavy);
  EXPECT_EQ(*raw->monitor_via, "/camera/camera/color/camera_info");
  EXPECT_TRUE(c.topic("/tf_static")->latched);
  EXPECT_FALSE(c.topic("/ublox_gps_node/fix")->hz);
  EXPECT_TRUE(c.sources.empty());
}

TEST(ScvConfig, EveryProfileResolvesLikeScv)
{
  const Catalog c = Catalog::load(std::string(kScv) + "/catalog.yaml");
  const ProfileStore store(std::string(kScv) + "/profiles");
  const YAML::Node golden = YAML::LoadFile(std::string(kScv) + "/resolved.yaml");
  ASSERT_EQ(store.names().size(), golden.size());
  for (const auto & name : store.names()) {
    SCOPED_TRACE(name);
    const auto r = store.resolve(name, c);
    const YAML::Node g = golden[name];
    EXPECT_EQ(r.chain, g["chain"].as<std::vector<std::string>>());
    EXPECT_EQ(r.abstract, g["abstract"].as<bool>());
    EXPECT_EQ(r.topics, g["topics"].as<std::vector<std::string>>());
    EXPECT_EQ(r.unknown_topics, g["unknown_topics"].as<std::vector<std::string>>());
    const YAML::Node o = g["options"];
    EXPECT_EQ(r.options.storage, o["storage"].as<std::string>());
    EXPECT_EQ(r.options.compression, o["compression"].as<std::string>());
    EXPECT_EQ(r.options.split_duration_sec, o["split_duration_sec"].as<int>());
    EXPECT_EQ(r.options.split_size_mb, o["split_size_mb"].as<int>());
    EXPECT_EQ(r.options.heavy_qos, o["heavy_qos"].as<std::string>());
    EXPECT_EQ(r.options.start_mode, o["start_mode"].as<std::string>());
    EXPECT_EQ(r.options.wait_ready_sec, o["wait_ready_sec"].as<int>());
    EXPECT_EQ(r.options.start_delay_sec, o["start_delay_sec"].as<int>());
    EXPECT_DOUBLE_EQ(r.options.disk_warn_gb, o["disk_warn_gb"].as<double>());
    EXPECT_DOUBLE_EQ(r.options.disk_stop_gb, o["disk_stop_gb"].as<double>());
    EXPECT_EQ(r.options.max_cache_mb, o["max_cache_mb"].as<int>());
    EXPECT_TRUE(r.sources.empty());
  }
}

TEST(ScvConfig, PresetFollowsCompression)
{
  const Catalog c = Catalog::load(std::string(kScv) + "/catalog.yaml");
  const ProfileStore store(std::string(kScv) + "/profiles");
  EXPECT_EQ(store.resolve("drive_standard", c).options.preset(), "fastwrite");
  EXPECT_EQ(store.resolve("long_transit", c).options.preset(), "zstd_fast");
}

// The original files, Korean descriptions included, when the SCV tree is at hand.
TEST(ScvConfig, OriginalFiles)
{
  const char * dir = std::getenv("ROSBAG_RECORDER_SCV_CONFIG");
  if (!dir) {
    GTEST_SKIP() << "ROSBAG_RECORDER_SCV_CONFIG not set";
  }
  const Catalog c = Catalog::load(std::string(dir) + "/catalog.yaml");
  const ProfileStore store(std::string(dir) + "/profiles");
  const YAML::Node golden = YAML::LoadFile(std::string(kScv) + "/resolved.yaml");
  for (const auto & name : store.names()) {
    EXPECT_EQ(
      store.resolve(name, c).topics, golden[name]["topics"].as<std::vector<std::string>>());
  }
}

TEST(Catalog, Errors)
{
  expect_error("bogus: 1\n", "unknown keys [bogus]");
  expect_error("groups: {Bad: {}}\n", "group names");
  expect_error("groups: {a: {topics: {no_slash: {}}}}\n", "bad topic name");
  expect_error("groups: {a: {topics: {/x: {hz: 0}}}}\n", "positive number");
  expect_error("groups: {a: {topics: {/x: {hz: '10'}}}}\n", "positive number");
  expect_error("groups: {a: {topics: {/x: {required: 1}}}}\n", "true or false");
  expect_error("groups: {a: {topics: {/x: {type: String}}}}\n", "pkg/msg/Type");
  expect_error("groups: {a: {topics: {/x: {colour: red}}}}\n", "unknown keys [colour]");
  expect_error("groups: {a: {topics: {/x: {}}}, b: {topics: {/x: {}}}}\n", "also in group a");
  expect_error("auto_markers: [{label: x}]\n", "bad topic name");
  expect_error("value_checks: {stamp_delay_max_s: -1}\n", "positive number");
  expect_error("sources: {s: {params: {}}}\n", "plugin: required");
}

TEST(Catalog, Sources)
{
  const Catalog c = catalog(kSmall);
  const auto & s = c.sources.at("counter");
  EXPECT_EQ(s.plugin, "test/Counter");
  EXPECT_EQ(YAML::Load(s.params_yaml)["rate_hz"].as<int>(), 5);
  EXPECT_TRUE(s.channels.at("/counter/value").required);
  EXPECT_EQ(
    c.sort_topics({"/zz", "/b1", "/a2", "/a1"}),
    (std::vector<std::string>{"/a1", "/a2", "/b1", "/zz"}));
}

TEST(Profile, Errors)
{
  auto parse = [](const std::string & text) {return Profile::parse(YAML::Load(text), "p");};
  EXPECT_THROW(parse("name: Bad\n"), ConfigError);
  EXPECT_THROW(parse("name: p\nwhat: 1\n"), ConfigError);
  EXPECT_THROW(parse("name: p\ntopics: [nope]\n"), ConfigError);
  EXPECT_THROW(parse("name: p\noptions: {speed: 1}\n"), ConfigError);
  EXPECT_THROW(parse("name: p\ngroups: a\n"), ConfigError);
  EXPECT_EQ(*parse("name: p\ngroups: []\n").groups, std::vector<std::string>{});
}

class Store : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = std::string(::testing::TempDir()) + "/profiles_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::filesystem::create_directories(dir_);
  }

  void write(const std::string & name, const std::string & text)
  {
    std::ofstream(dir_ + "/" + name + ".yaml") << text;
  }

  std::string resolve_error(const std::string & name)
  {
    try {
      ProfileStore(dir_).resolve(name, catalog(kSmall));
    } catch (const ConfigError & e) {
      return e.what();
    }
    return "";
  }

  std::string dir_;
};

TEST_F(Store, InheritanceAndSources)
{
  write(
    "base", "name: base\nabstract: true\ngroups: [a]\nsources: [counter]\n"
    "options: {compression: zstd, required_frames: [base_link]}\n");
  write(
    "child", "name: child\nextends: base\nadd_groups: [b]\nexclude: [/a2]\n"
    "topics: [/extra]\noptions: {split_duration_sec: 60}\n");
  write("nosrc", "name: nosrc\nextends: child\nexclude_sources: [counter]\n");
  const ProfileStore store(dir_);
  const auto r = store.resolve("child", catalog(kSmall));
  EXPECT_EQ(r.chain, (std::vector<std::string>{"child", "base"}));
  EXPECT_EQ(r.topics, (std::vector<std::string>{"/a1", "/b1"}));
  EXPECT_EQ(r.unknown_topics, std::vector<std::string>{"/extra"});
  EXPECT_EQ(r.sources, std::vector<std::string>{"counter"});
  EXPECT_EQ(r.options.compression, "zstd");
  EXPECT_EQ(r.options.split_duration_sec, 60);
  EXPECT_EQ(r.options.required_frames, std::vector<std::string>{"base_link"});
  EXPECT_FALSE(r.abstract);
  EXPECT_TRUE(store.resolve("nosrc", catalog(kSmall)).sources.empty());
  EXPECT_NE(r.to_yaml().find("split_duration_sec: 60"), std::string::npos);
}

TEST_F(Store, Errors)
{
  write("loop_a", "name: loop_a\nextends: loop_b\n");
  write("loop_b", "name: loop_b\nextends: loop_a\n");
  write("nogroup", "name: nogroup\ngroups: [zzz]\n");
  write("nosource", "name: nosource\nsources: [zzz]\n");
  write("disk", "name: disk\noptions: {disk_stop_gb: 50}\n");
  write("choice", "name: choice\noptions: {storage: tar}\n");
  write("count", "name: count\noptions: {split_size_mb: 1.5}\n");
  write("wait", "name: wait\noptions: {wait_ready_sec: 0}\n");
  write("renamed", "name: other\n");
  EXPECT_NE(
    resolve_error("loop_a").find("inheritance cycle: loop_a -> loop_b -> loop_a"),
    std::string::npos);
  EXPECT_NE(resolve_error("nogroup").find("no group 'zzz'"), std::string::npos);
  EXPECT_NE(resolve_error("nosource").find("no source 'zzz'"), std::string::npos);
  EXPECT_NE(resolve_error("disk").find("disk_stop_gb is larger"), std::string::npos);
  EXPECT_NE(resolve_error("choice").find("choices: mcap, sqlite3"), std::string::npos);
  EXPECT_NE(resolve_error("count").find("integer >= 0"), std::string::npos);
  EXPECT_NE(resolve_error("wait").find("wait_ready_sec"), std::string::npos);
  EXPECT_NE(resolve_error("renamed").find("differ"), std::string::npos);
  EXPECT_NE(resolve_error("missing").find("no profile missing"), std::string::npos);
}
