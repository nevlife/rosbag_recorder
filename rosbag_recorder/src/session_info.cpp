#include "session_info.hpp"

#include "yaml-cpp/yaml.h"

#include <openssl/evp.h>
#include <pwd.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace rosbag_recorder
{
namespace
{

namespace fs = std::filesystem;

constexpr std::size_t max_changes = 50;

std::string run(const std::string & command)
{
  std::string out;
  FILE * pipe = popen(command.c_str(), "r");
  std::array<char, 4096> buffer{};
  std::size_t n;
  while ((n = fread(buffer.data(), 1, buffer.size(), pipe)) > 0) {
    out.append(buffer.data(), n);
  }
  if (pclose(pipe) != 0) {
    throw std::runtime_error("failed: " + command);
  }
  return out;
}

fs::path existing_parent(const std::string & path)
{
  fs::path p = fs::absolute(path);
  while (!fs::exists(p)) {
    p = p.parent_path();
  }
  return p;
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \n\t");
  const auto e = s.find_last_not_of(" \n\t");
  return b == std::string::npos ? "" : s.substr(b, e - b + 1);
}

std::string quote(const std::string & s)
{
  return "'" + std::regex_replace(s, std::regex("'"), "'\\''") + "'";
}

std::vector<std::string> lines(const std::string & s)
{
  std::vector<std::string> out;
  std::istringstream in(s);
  for (std::string line; std::getline(in, line);) {
    if (!line.empty()) {
      out.push_back(line);
    }
  }
  return out;
}

YAML::Node repo_info(const std::string & path)
{
  const std::string git = "git -C " + quote(path) + " ";
  YAML::Node n;
  const std::string branch = trim(run(git + "rev-parse --abbrev-ref HEAD"));
  n["branch"] = branch == "HEAD" ? "(detached)" : branch;
  n["commit"] = trim(run(git + "rev-parse HEAD"));
  const auto changes = lines(run(git + "status --porcelain --ignore-submodules=all"));
  n["dirty"] = !changes.empty();
  if (!changes.empty()) {
    n["changes_total"] = changes.size();
    n["changes"] = std::vector<std::string>(
      changes.begin(), changes.begin() + std::min(changes.size(), max_changes));
  }
  return n;
}

}  // namespace

double disk_free_gb(const std::string & path)
{
  struct statvfs st
  {
  };
  if (statvfs(existing_parent(path).c_str(), &st) != 0) {
    throw std::system_error(errno, std::generic_category(), "statvfs " + path);
  }
  return static_cast<double>(st.f_bavail) * st.f_frsize / (1024.0 * 1024.0 * 1024.0);
}

std::uint64_t directory_bytes(const std::string & path)
{
  std::uint64_t total = 0;
  for (const auto & e : fs::directory_iterator(path)) {
    total += e.file_size();
  }
  return total;
}

bool writable(const std::string & directory)
{
  return access(existing_parent(directory).c_str(), W_OK) == 0;
}

std::string expand_user(const std::string & path)
{
  if (path.rfind("~", 0) != 0) {
    return path;
  }
  return env("HOME") + path.substr(1);
}

std::string session_id(const std::string & profile, const std::string & label, std::time_t now)
{
  std::array<char, 32> stamp{};
  std::tm local{};
  localtime_r(&now, &local);
  std::strftime(stamp.data(), stamp.size(), "%Y%m%d_%H%M%S", &local);
  std::string id = std::string(stamp.data()) + "_" + profile;
  std::string slug = std::regex_replace(label, std::regex("[^0-9A-Za-z_-]+"), "_");
  slug = slug.substr(0, 40);
  const auto b = slug.find_first_not_of('_');
  const auto e = slug.find_last_not_of('_');
  return b == std::string::npos ? id : id + "_" + slug.substr(b, e - b + 1);
}

std::string host_name()
{
  std::array<char, 256> name{};
  gethostname(name.data(), name.size() - 1);
  return name.data();
}

std::string user_name()
{
  passwd pw{};
  passwd * found = nullptr;
  std::array<char, 1024> buffer{};
  getpwuid_r(getuid(), &pw, buffer.data(), buffer.size(), &found);
  if (!found) {
    throw std::runtime_error("no passwd entry for uid " + std::to_string(getuid()));
  }
  return found->pw_name;
}

std::string env(const char * name)
{
  const char * v = std::getenv(name);
  return v ? v : "";
}

std::string git_snapshot(const std::string & workspace)
{
  if (workspace.empty()) {
    return "";
  }
  YAML::Node n;
  n["workspace"] = workspace;
  n["superproject"] = repo_info(workspace);
  for (const auto & line :
       lines(run("git -C " + quote(workspace) + " submodule status --recursive"))) {
    std::istringstream in(line.substr(1));
    std::string commit, path;
    in >> commit >> path;
    YAML::Node sub = repo_info((fs::path(workspace) / path).string());
    sub["path"] = path;
    sub["matches_superproject"] = line[0] == ' ';
    n["submodules"].push_back(sub);
  }
  return YAML::Dump(n);
}

bool glob_match(const std::string & pattern, const std::string & path)
{
  std::string re;
  for (std::size_t i = 0; i < pattern.size(); ++i) {
    const char c = pattern[i];
    if (pattern.compare(i, 3, "**/") == 0) {
      re += "(.*/)?";
      i += 2;
    } else if (c == '*') {
      re += "[^/]*";
    } else if (c == '?') {
      re += "[^/]";
    } else if (std::string("\\^$.|+()[]{}").find(c) != std::string::npos) {
      re += std::string("\\") + c;
    } else {
      re += c;
    }
  }
  return std::regex_match(path, std::regex(re));
}

std::vector<rosbag_recorder_msgs::msg::FileSnapshot> snapshot_files(
  const std::string & workspace, const std::vector<std::string> & globs, std::uint64_t max_bytes)
{
  std::vector<rosbag_recorder_msgs::msg::FileSnapshot> out;
  if (workspace.empty() || globs.empty()) {
    return out;
  }
  std::vector<fs::path> files;
  for (auto it = fs::recursive_directory_iterator(workspace);
       it != fs::recursive_directory_iterator(); ++it) {
    const std::string name = it->path().filename().string();
    if (
      it->is_directory() &&
      (name == "build" || name == "install" || name == "log" || name == ".git")) {
      it.disable_recursion_pending();
    } else if (it->is_regular_file()) {
      files.push_back(fs::relative(it->path(), workspace));
    }
  }
  std::sort(files.begin(), files.end());
  std::uint64_t copied = 0;
  for (const auto & rel : files) {
    bool match = false;
    for (const auto & g : globs) {
      match = match || glob_match(g, rel.string());
    }
    if (!match) {
      continue;
    }
    std::ifstream in(fs::path(workspace) / rel, std::ios::binary);
    std::ostringstream content;
    content << in.rdbuf();
    rosbag_recorder_msgs::msg::FileSnapshot f;
    f.path = rel.string();
    f.size = content.str().size();
    f.sha256 = sha256(content.str());
    if (copied + f.size <= max_bytes) {
      f.content = content.str();
      f.copied = true;
      copied += f.size;
    }
    out.push_back(f);
  }
  return out;
}

std::string sha256(const std::string & data)
{
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  EVP_Digest(data.data(), data.size(), digest.data(), &length, EVP_sha256(), nullptr);
  std::ostringstream hex;
  for (unsigned int i = 0; i < length; ++i) {
    hex << "0123456789abcdef"[digest[i] >> 4] << "0123456789abcdef"[digest[i] & 0xf];
  }
  return hex.str();
}

}  // namespace rosbag_recorder
