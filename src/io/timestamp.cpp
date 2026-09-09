#include "sfr/io/timestamp.hpp"

#include <charconv>
#include <fstream>
#include <string_view>

#include "sfr/io/io_error.hpp"

namespace sfr::io {

namespace {

constexpr std::size_t kKittiTimestampLength = 29U;
constexpr std::size_t kMaximumTimestampCount = 1'000'000U;

[[nodiscard]] int parseDigits(std::string_view text, std::size_t offset, std::size_t count,
                              const std::string& original) {
  const std::string_view field = text.substr(offset, count);
  int value = 0;
  const auto [end, error] = std::from_chars(field.data(), field.data() + field.size(), value);
  if (error != std::errc{} || end != field.data() + field.size()) {
    throw IoError(IoErrorCode::kInvalidTimestamp,
                  "invalid numeric field in KITTI timestamp: " + original);
  }
  return value;
}

} // namespace

SensorTimestamp::SensorTimestamp(std::chrono::nanoseconds civil_time) : civil_time_(civil_time) {}

std::chrono::nanoseconds SensorTimestamp::civilTime() const noexcept { return civil_time_; }

SensorTimestamp parseKittiTimestamp(const std::string& text) {
  if (text.size() != kKittiTimestampLength || text[4] != '-' || text[7] != '-' || text[10] != ' ' ||
      text[13] != ':' || text[16] != ':' || text[19] != '.') {
    throw IoError(IoErrorCode::kInvalidTimestamp,
                  "KITTI timestamp must use YYYY-MM-DD HH:MM:SS.nnnnnnnnn: " + text);
  }

  const int year_value = parseDigits(text, 0, 4, text);
  const int month_value = parseDigits(text, 5, 2, text);
  const int day_value = parseDigits(text, 8, 2, text);
  const int hour_value = parseDigits(text, 11, 2, text);
  const int minute_value = parseDigits(text, 14, 2, text);
  const int second_value = parseDigits(text, 17, 2, text);
  const int nanosecond_value = parseDigits(text, 20, 9, text);

  const std::chrono::year_month_day date{std::chrono::year(year_value),
                                         std::chrono::month(static_cast<unsigned>(month_value)),
                                         std::chrono::day(static_cast<unsigned>(day_value))};
  if (!date.ok() || hour_value < 0 || hour_value > 23 || minute_value < 0 || minute_value > 59 ||
      second_value < 0 || second_value > 59 || nanosecond_value < 0) {
    throw IoError(IoErrorCode::kInvalidTimestamp, "out-of-range KITTI timestamp: " + text);
  }

  const auto civil_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::sys_days(date).time_since_epoch()) +
                          std::chrono::hours(hour_value) + std::chrono::minutes(minute_value) +
                          std::chrono::seconds(second_value) +
                          std::chrono::nanoseconds(nanosecond_value);
  return SensorTimestamp(civil_time);
}

std::vector<TimestampRecord> loadKittiTimestamps(const std::filesystem::path& timestamp_file) {
  std::ifstream input(timestamp_file);
  if (!input.is_open()) {
    throw IoError(IoErrorCode::kFileOpen,
                  "unable to open timestamp file: " + timestamp_file.string());
  }

  std::vector<TimestampRecord> records;
  std::string line;
  while (std::getline(input, line)) {
    if (records.size() >= kMaximumTimestampCount) {
      throw IoError(IoErrorCode::kLimitExceeded, "timestamp count exceeds configured limit");
    }
    const SensorTimestamp parsed = parseKittiTimestamp(line);
    if (!records.empty() && parsed <= records.back().timestamp) {
      throw IoError(IoErrorCode::kInvalidTimestamp,
                    "timestamps must be strictly increasing at index " +
                        std::to_string(records.size()));
    }
    records.push_back(TimestampRecord{parsed, line});
  }
  if (records.empty()) {
    throw IoError(IoErrorCode::kInvalidTimestamp, "timestamp file must not be empty");
  }
  return records;
}

} // namespace sfr::io
