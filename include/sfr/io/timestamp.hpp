#ifndef SFR_IO_TIMESTAMP_HPP_
#define SFR_IO_TIMESTAMP_HPP_

#include <chrono>
#include <compare>
#include <filesystem>
#include <string>
#include <vector>

namespace sfr::io {

class SensorTimestamp final {
public:
  explicit SensorTimestamp(std::chrono::nanoseconds civil_time);

  [[nodiscard]] std::chrono::nanoseconds civilTime() const noexcept;
  auto operator<=>(const SensorTimestamp&) const = default;

private:
  std::chrono::nanoseconds civil_time_;
};

struct TimestampRecord final {
  SensorTimestamp timestamp;
  std::string original_text;
};

[[nodiscard]] SensorTimestamp parseKittiTimestamp(const std::string& text);
[[nodiscard]] std::vector<TimestampRecord>
loadKittiTimestamps(const std::filesystem::path& timestamp_file);

} // namespace sfr::io

#endif // SFR_IO_TIMESTAMP_HPP_
