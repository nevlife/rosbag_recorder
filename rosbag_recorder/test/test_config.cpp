#include "config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using rosbag_recorder::Catalog;
using rosbag_recorder::ConfigError;
using rosbag_recorder::Profile;
using rosbag_recorder::ProfileStore;

namespace
{

Catalog catalog(const std::string & text)
{
  return Catalog::parse(YAML::Load(text), "catalog");
}

const char * const small_catalog =
  "groups:\n"
  "  a:\n"
  "    topics:\n"
  "      /a1: {type: std_msgs/msg/String, hz: 10, required: true}\n"
  "      /a2: {}\n"
  "  b:\n"
  "    topics:\n"
  "      /b1:\n";

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
  expect_error("sources: {}\n", "unknown keys [sources]");
  expect_error("heavy_threshold_kb: 500\n", "unknown keys [heavy_threshold_kb]");
  expect_error(
    "groups: {a: {topics: {/x: {monitor_via: /y}}}}\n", "unknown keys [monitor_via]");
  expect_error("auto_markers: []\n", "unknown keys [auto_markers]");
  expect_error("value_checks: {}\n", "unknown keys [value_checks]");
  expect_error("groups: {a: {topics: {/x: {size_kb: 1}}}}\n", "unknown keys [size_kb]");
}

TEST(Catalog, SortTopics)
{
  const Catalog c = catalog(small_catalog);
  EXPECT_EQ(
    c.sort_topics({"/zz", "/b1", "/a2", "/a1"}),
    (std::vector<std::string>{"/a1", "/a2", "/b1", "/zz"}));
}

TEST(Profile, Errors)
{
  auto parse = [](const std::string & text) { return Profile::parse(YAML::Load(text), "p"); };
  EXPECT_THROW(parse("name: Bad\n"), ConfigError);
  EXPECT_THROW(parse("name: p\nwhat: 1\n"), ConfigError);
  EXPECT_THROW(parse("name: p\ntopics: [nope]\n"), ConfigError);
  EXPECT_THROW(parse("name: p\noptions: {speed: 1}\n"), ConfigError);
  EXPECT_THROW(parse("name: p\ngroups: a\n"), ConfigError);
  EXPECT_THROW(parse("name: p\nsources: [s]\n"), ConfigError);
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
      ProfileStore(dir_).resolve(name, catalog(small_catalog));
    } catch (const ConfigError & e) {
      return e.what();
    }
    return "";
  }

  std::string dir_;
};

TEST_F(Store, Inheritance)
{
  write(
    "base",
    "name: base\nabstract: true\ngroups: [a]\n"
    "options: {compression: zstd, required_frames: [base_link]}\n");
  write(
    "child",
    "name: child\nextends: base\nadd_groups: [b]\nexclude: [/a2]\n"
    "topics: [/extra]\noptions: {split_duration_sec: 60}\n");
  const ProfileStore store(dir_);
  const auto r = store.resolve("child", catalog(small_catalog));
  EXPECT_EQ(r.chain, (std::vector<std::string>{"child", "base"}));
  EXPECT_EQ(r.topics, (std::vector<std::string>{"/a1", "/b1"}));
  EXPECT_EQ(r.unknown_topics, std::vector<std::string>{"/extra"});
  EXPECT_EQ(r.options.compression, "zstd");
  EXPECT_EQ(r.options.split_duration_sec, 60);
  EXPECT_EQ(r.options.required_frames, std::vector<std::string>{"base_link"});
  EXPECT_FALSE(r.abstract);
  EXPECT_NE(r.to_yaml().find("split_duration_sec: 60"), std::string::npos);
  EXPECT_EQ(r.options.preset(), "zstd_fast");
  EXPECT_EQ(store.resolve("base", catalog(small_catalog)).options.preset(), "zstd_fast");
  write("plain", "name: plain\n");
  EXPECT_EQ(
    ProfileStore(dir_).resolve("plain", catalog(small_catalog)).options.preset(), "fastwrite");
}

TEST_F(Store, Errors)
{
  write("loop_a", "name: loop_a\nextends: loop_b\n");
  write("loop_b", "name: loop_b\nextends: loop_a\n");
  write("nogroup", "name: nogroup\ngroups: [zzz]\n");
  write("disk", "name: disk\noptions: {disk_stop_gb: 50}\n");
  write("choice", "name: choice\noptions: {storage: tar}\n");
  write("count", "name: count\noptions: {split_size_mb: 1.5}\n");
  write("wait", "name: wait\noptions: {wait_ready_sec: 0}\n");
  write("renamed", "name: other\n");
  EXPECT_NE(
    resolve_error("loop_a").find("inheritance cycle: loop_a -> loop_b -> loop_a"),
    std::string::npos);
  EXPECT_NE(resolve_error("nogroup").find("no group 'zzz'"), std::string::npos);
  EXPECT_NE(resolve_error("disk").find("disk_stop_gb is larger"), std::string::npos);
  EXPECT_NE(resolve_error("choice").find("choices: mcap, sqlite3"), std::string::npos);
  EXPECT_NE(resolve_error("count").find("integer >= 0"), std::string::npos);
  EXPECT_NE(resolve_error("wait").find("wait_ready_sec"), std::string::npos);
  EXPECT_NE(resolve_error("renamed").find("differ"), std::string::npos);
  EXPECT_NE(resolve_error("missing").find("no profile missing"), std::string::npos);
}
