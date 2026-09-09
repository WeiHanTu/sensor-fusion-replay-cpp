#include "sfr/io/timestamp.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "sfr/io/io_error.hpp"

namespace sfr::io {
namespace {

TEST(TimestampTest, ParsesNanosecondsOnSharedCivilScale) {
  const SensorTimestamp first = parseKittiTimestamp("2011-09-26 13:02:45.000000001");
  const SensorTimestamp second = parseKittiTimestamp("2011-09-26 13:02:45.100000002");

  EXPECT_EQ(second.civilTime() - first.civilTime(), std::chrono::nanoseconds(100'000'001));
}

TEST(TimestampTest, RejectsInvalidDateAndFractionPrecision) {
  EXPECT_THROW(static_cast<void>(parseKittiTimestamp("2011-02-29 13:02:45.000000001")), IoError);
  EXPECT_THROW(static_cast<void>(parseKittiTimestamp("2011-09-26 13:02:45.000001")), IoError);
  EXPECT_THROW(static_cast<void>(parseKittiTimestamp("2011-09-26 13:02:x5.000000001")), IoError);
  EXPECT_THROW(static_cast<void>(parseKittiTimestamp("2011-09-26 24:02:45.000000001")), IoError);
}

TEST(TimestampTest, LoadsStrictlyIncreasingRecords) {
  const std::filesystem::path path =
      std::filesystem::path(::testing::TempDir()) / "sfr_timestamp_valid.txt";
  std::ofstream(path) << "2011-09-26 13:02:45.000000001\n"
                         "2011-09-26 13:02:45.000000002\n";

  const std::vector<TimestampRecord> records = loadKittiTimestamps(path);

  ASSERT_EQ(records.size(), 2U);
  EXPECT_EQ(records[0].original_text, "2011-09-26 13:02:45.000000001");
  std::error_code error;
  std::filesystem::remove(path, error);
}

TEST(TimestampTest, RejectsEqualTimestamp) {
  const std::filesystem::path path =
      std::filesystem::path(::testing::TempDir()) / "sfr_timestamp_equal.txt";
  std::ofstream(path) << "2011-09-26 13:02:45.000000001\n"
                         "2011-09-26 13:02:45.000000001\n";

  EXPECT_THROW(static_cast<void>(loadKittiTimestamps(path)), IoError);
  std::error_code error;
  std::filesystem::remove(path, error);
}

TEST(TimestampTest, RejectsDecreasingAndEmptyTimestampFiles) {
  const std::filesystem::path decreasing_path =
      std::filesystem::path(::testing::TempDir()) / "sfr_timestamp_decreasing.txt";
  const std::filesystem::path empty_path =
      std::filesystem::path(::testing::TempDir()) / "sfr_timestamp_empty.txt";
  std::ofstream(decreasing_path) << "2011-09-26 13:02:45.000000002\n"
                                    "2011-09-26 13:02:45.000000001\n";
  const std::ofstream empty_file(empty_path);
  ASSERT_TRUE(empty_file.good());

  EXPECT_THROW(static_cast<void>(loadKittiTimestamps(decreasing_path)), IoError);
  EXPECT_THROW(static_cast<void>(loadKittiTimestamps(empty_path)), IoError);

  std::error_code error;
  std::filesystem::remove(decreasing_path, error);
  std::filesystem::remove(empty_path, error);
}

} // namespace
} // namespace sfr::io
