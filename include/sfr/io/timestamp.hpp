#ifndef SFR_IO_TIMESTAMP_HPP_
#define SFR_IO_TIMESTAMP_HPP_

#include <chrono>
#include <compare>
#include <filesystem>
#include <string>
#include <vector>

namespace sfr::io {

/// Strong nanosecond value on KITTI's timezone-unspecified civil-time scale.
///
/// The value is comparable within the dataset but must not be labeled UTC or
/// local time without external timezone information.
class SensorTimestamp final {
public:
  /// Stores the caller-provided civil-time duration value.
  explicit SensorTimestamp(std::chrono::nanoseconds civil_time);

  /// Returns the stored nanoseconds on the same timezone-free civil scale.
  [[nodiscard]] std::chrono::nanoseconds civilTime() const noexcept;
  auto operator<=>(const SensorTimestamp&) const = default;

private:
  std::chrono::nanoseconds civil_time_;
};

/// Parsed timestamp paired with its exact original input text.
struct TimestampRecord final {
  SensorTimestamp timestamp;
  std::string original_text;
};

/// Parses exactly `YYYY-MM-DD HH:MM:SS.nnnnnnnnn` at nanosecond precision.
/// @throws IoError For malformed fields or out-of-range civil date/time values.
[[nodiscard]] SensorTimestamp parseKittiTimestamp(const std::string& text);

/// Loads a nonempty, strictly increasing timestamp file in line order.
///
/// Returned records own both parsed values and original text.
/// @throws IoError For file access, malformed/non-monotonic input, or count limit.
[[nodiscard]] std::vector<TimestampRecord>
loadKittiTimestamps(const std::filesystem::path& timestamp_file);

} // namespace sfr::io

#endif // SFR_IO_TIMESTAMP_HPP_
