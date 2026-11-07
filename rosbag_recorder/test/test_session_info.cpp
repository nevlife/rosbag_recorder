#include "session_info.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace
{

class Workspace : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ =
      fs::path(::testing::TempDir()) /
      ("workspace_" + std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "src" / "a" / "config");
    fs::create_directories(dir_ / "build" / "config");
    write("src/a/config/x.yaml", "x: 1\n");
    write("src/a/config/y.yaml", "y: 22\n");
    write("build/config/z.yaml", "z: 3\n");
  }

  void write(const std::string & path, const std::string & text)
  {
    std::ofstream(dir_ / path) << text;
  }

  fs::path dir_;
};

}  // namespace

TEST(SessionInfo, GlobAndIds)
{
  using rosbag_recorder::glob_match;
  EXPECT_TRUE(glob_match("src/**/config/*.yaml", "src/a/b/config/x.yaml"));
  EXPECT_TRUE(glob_match("src/**/config/*.yaml", "src/config/x.yaml"));
  EXPECT_FALSE(glob_match("src/**/config/*.yaml", "src/a/config/sub/x.yaml"));
  EXPECT_TRUE(glob_match("src/bring_up/launch/*.py", "src/bring_up/launch/a.py"));
  EXPECT_FALSE(glob_match("src/*.py", "src/a/b.py"));
  EXPECT_TRUE(glob_match("params/scv?.yaml", "params/scv1.yaml"));
  const std::time_t t = 0;
  const std::string id = rosbag_recorder::session_id("drive", " test run!", t);
  EXPECT_EQ(id.substr(15), "_drive_test_run");
  EXPECT_EQ(rosbag_recorder::session_id("drive", "", t).substr(15), "_drive");
  EXPECT_EQ(rosbag_recorder::session_id("drive", "___", t).substr(15), "_drive");
  EXPECT_EQ(
    rosbag_recorder::sha256("abc"),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(rosbag_recorder::expand_user("/x"), "/x");
  EXPECT_EQ(rosbag_recorder::expand_user("~/x"), std::string(std::getenv("HOME")) + "/x");
  EXPECT_FALSE(rosbag_recorder::user_name().empty());
  EXPECT_FALSE(rosbag_recorder::host_name().empty());
}

TEST_F(Workspace, DiskAndDirectory)
{
  EXPECT_EQ(rosbag_recorder::directory_bytes((dir_ / "src" / "a" / "config").string()), 11U);
  EXPECT_THROW(rosbag_recorder::directory_bytes((dir_ / "missing").string()), fs::filesystem_error);
  EXPECT_GT(rosbag_recorder::disk_free_gb((dir_ / "not" / "yet").string()), 0.0);
  EXPECT_TRUE(rosbag_recorder::writable((dir_ / "not" / "yet").string()));
}

TEST_F(Workspace, SnapshotFiles)
{
  const auto files = rosbag_recorder::snapshot_files(dir_.string(), {"**/config/*.yaml"}, 6);
  ASSERT_EQ(files.size(), 2U);
  EXPECT_EQ(files[0].path, "src/a/config/x.yaml");
  EXPECT_TRUE(files[0].copied);
  EXPECT_EQ(files[0].content, "x: 1\n");
  EXPECT_EQ(files[1].size, 6U);
  EXPECT_FALSE(files[1].copied);
  EXPECT_EQ(files[1].sha256, rosbag_recorder::sha256("y: 22\n"));
  EXPECT_TRUE(rosbag_recorder::snapshot_files(dir_.string(), {}, 6).empty());
  EXPECT_THROW(
    rosbag_recorder::snapshot_files((dir_ / "missing").string(), {"*"}, 6), fs::filesystem_error);
}

TEST_F(Workspace, GitSnapshot)
{
  EXPECT_EQ(rosbag_recorder::git_snapshot(""), "");
  EXPECT_THROW(rosbag_recorder::git_snapshot(dir_.string()), std::runtime_error);
  const std::string git = "git -C '" + dir_.string() +
                          "' -c user.name=t -c user.email=t@t -c commit.gpgsign=false "
                          "-c core.hooksPath=/dev/null ";
  ASSERT_EQ(
    std::system((git + "init -q && " + git + "add src && " + git + "commit -qm init").c_str()), 0);
  write("src/a/config/x.yaml", "x: 2\n");
  const std::string yaml = rosbag_recorder::git_snapshot(dir_.string());
  EXPECT_NE(yaml.find("dirty: true"), std::string::npos) << yaml;
  EXPECT_NE(yaml.find("src/a/config/x.yaml"), std::string::npos) << yaml;
}
