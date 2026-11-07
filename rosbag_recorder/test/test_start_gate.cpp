#include "start_gate.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using rosbag_recorder::ConfigError;
using rosbag_recorder::ResolvedProfile;
using rosbag_recorder::StartGate;
using rosbag_recorder_msgs::msg::CheckResult;
using Step = StartGate::Step;

namespace
{

ResolvedProfile profile(const std::string & mode)
{
  ResolvedProfile p;
  p.name = "p";
  p.options.start_mode = mode;
  p.options.wait_ready_sec = 10;
  p.options.start_delay_sec = 5;
  return p;
}

std::string refusal(const ResolvedProfile & p, std::size_t streams, std::vector<CheckResult> checks)
{
  try {
    StartGate().request(p, streams, checks, true, 0.0);
  } catch (const ConfigError & e) {
    return e.what();
  }
  return "";
}

}  // namespace

TEST(StartGate, Refusals)
{
  auto p = profile("immediate");
  p.name = "default";
  EXPECT_EQ(refusal(p, 0, {}), "select at least one topic");
  EXPECT_EQ(refusal(p, 1, {}), "");
  p.name = "p";
  p.abstract = true;
  EXPECT_EQ(refusal(p, 1, {}), "p is abstract");
  p.abstract = false;
  CheckResult error;
  error.level = CheckResult::ERROR;
  error.text = "cannot write to /x";
  EXPECT_EQ(refusal(p, 1, {error}), "preflight: cannot write to /x");
}

TEST(StartGate, ImmediateAndForce)
{
  StartGate gate;
  EXPECT_TRUE(gate.request(profile("immediate"), 1, {}, false, 0.0));
  EXPECT_TRUE(gate.request(profile("wait_ready"), 1, {}, true, 0.0));
}

TEST(StartGate, Delay)
{
  StartGate gate;
  const auto p = profile("delay");
  EXPECT_FALSE(gate.request(p, 1, {}, false, 100.0));
  EXPECT_EQ(gate.message(), "starting in 5 s");
  EXPECT_EQ(gate.poll(p.options, 104.0, {}), Step::Wait);
  EXPECT_EQ(gate.poll(p.options, 105.0, {"/a"}), Step::Begin);
  EXPECT_EQ(gate.message(), "");
}

TEST(StartGate, WaitReady)
{
  StartGate gate;
  auto p = profile("wait_ready");
  EXPECT_FALSE(gate.request(p, 1, {}, false, 100.0));
  EXPECT_EQ(gate.message(), "waiting up to 10 s for required topics");
  EXPECT_EQ(gate.poll(p.options, 101.0, {"/a"}), Step::Wait);
  EXPECT_EQ(gate.poll(p.options, 101.0, {}), Step::Begin);
  EXPECT_EQ(gate.poll(p.options, 110.0, {"/a"}), Step::Abort);
  EXPECT_EQ(gate.message(), "required topics not ready after 10 s: /a");
  p.options.on_not_ready = "start";
  EXPECT_EQ(gate.poll(p.options, 110.0, {"/a"}), Step::Begin);
  EXPECT_EQ(gate.message(), "started without required topics: /a");
}
