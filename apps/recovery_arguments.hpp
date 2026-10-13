#ifndef SFR_APPS_RECOVERY_ARGUMENTS_HPP_
#define SFR_APPS_RECOVERY_ARGUMENTS_HPP_

#include <charconv>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace sfr::cli {

class ArgumentError final : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/// Shared argument syntax only; no experiment or geometry decisions live here.
class RecoveryArguments final {
public:
  RecoveryArguments(const int argc, char** argv, const std::set<std::string>& allowed) {
    for (int index = 1; index < argc; ++index) {
      const std::string key(argv[index]);
      if (key == "--help" || key == "--overwrite") {
        if (!flags_.insert(key).second) {
          throw ArgumentError("duplicate option: " + key);
        }
      } else {
        if (!allowed.contains(key) || index + 1 >= argc ||
            std::string_view(argv[index + 1]).starts_with("--")) {
          throw ArgumentError("unknown option or missing value: " + key);
        }
        if (!values_.emplace(key, argv[++index]).second) {
          throw ArgumentError("duplicate option: " + key);
        }
      }
    }
  }
  [[nodiscard]] bool help() const { return flags_.contains("--help"); }
  [[nodiscard]] bool overwrite() const { return flags_.contains("--overwrite"); }
  [[nodiscard]] std::string value(const std::string& key, const std::string& fallback = "") const {
    const auto found = values_.find(key);
    return found == values_.end() ? fallback : found->second;
  }
  [[nodiscard]] std::string required(const std::string& key) const {
    const auto result = value(key);
    if (result.empty()) {
      throw ArgumentError("required option: " + key);
    }
    return result;
  }
  [[nodiscard]] double number(const std::string& key, const double fallback) const {
    const auto found = values_.find(key);
    if (found == values_.end()) {
      return fallback;
    }
    try {
      std::size_t used = 0U;
      const double result = std::stod(found->second, &used);
      if (used != found->second.size() || !std::isfinite(result)) {
        throw ArgumentError("expected finite number: " + key);
      }
      return result;
    } catch (const std::invalid_argument&) {
      throw ArgumentError("invalid number: " + key);
    } catch (const std::out_of_range&) {
      throw ArgumentError("number out of range: " + key);
    }
  }
  [[nodiscard]] static std::uint64_t parseUnsigned(const std::string_view text) {
    std::uint64_t result = 0U;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), result);
    if (text.empty() || error != std::errc{} || end != text.data() + text.size()) {
      throw ArgumentError("expected unsigned integer");
    }
    return result;
  }
  [[nodiscard]] std::uint64_t unsignedValue(const std::string& key,
                                            const std::uint64_t fallback) const {
    const auto found = values_.find(key);
    return found == values_.end() ? fallback : parseUnsigned(found->second);
  }
  [[nodiscard]] std::vector<std::uint64_t> ids(const std::string& key,
                                               const std::string& fallback) const {
    const std::string text = value(key, fallback);
    std::vector<std::uint64_t> result;
    std::size_t start = 0U;
    while (start <= text.size()) {
      const auto comma = text.find(',', start);
      const auto count = comma == std::string::npos ? text.size() - start : comma - start;
      result.push_back(parseUnsigned(std::string_view(text).substr(start, count)));
      if (comma == std::string::npos) {
        break;
      }
      start = comma + 1U;
    }
    return result;
  }

private:
  std::map<std::string, std::string> values_;
  std::set<std::string> flags_;
};

} // namespace sfr::cli

#endif // SFR_APPS_RECOVERY_ARGUMENTS_HPP_
