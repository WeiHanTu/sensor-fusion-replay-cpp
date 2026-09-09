#include "sfr/io/kitti_io.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

#include "sfr/io/io_error.hpp"

namespace sfr::io {
namespace {

class KittiIoTest : public ::testing::Test {
protected:
  void SetUp() override {
    root_ = std::filesystem::path(::testing::TempDir()) / "sfr_kitti_io_test";
    std::error_code error;
    std::filesystem::remove_all(root_, error);
    ASSERT_TRUE(std::filesystem::create_directories(root_));
  }

  void TearDown() override {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  std::filesystem::path root_;
};

void writeLidarRecords(const std::filesystem::path& path,
                       const std::vector<std::array<float, 4>>& records) {
  std::ofstream output(path, std::ios::binary);
  for (const auto& record : records) {
    output.write(reinterpret_cast<const char*>(record.data()),
                 static_cast<std::streamsize>(record.size() * sizeof(float)));
  }
}

TEST_F(KittiIoTest, LoadsFiniteVelodyneRecordsAndCountsRejectedPoints) {
  const std::filesystem::path path = root_ / "0000000000.bin";
  writeLidarRecords(path, {{1.0F, 2.0F, 3.0F, 0.5F},
                           {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F},
                           {-1.0F, -2.0F, -3.0F, 1.0F}});

  const LidarLoadResult result = loadVelodyneFrame(path);

  ASSERT_EQ(result.points.size(), 2U);
  EXPECT_EQ(result.non_finite_points, 1U);
  EXPECT_DOUBLE_EQ(result.points[1].x_m, -1.0);
  EXPECT_FLOAT_EQ(result.points[0].reflectance, 0.5F);
}

TEST_F(KittiIoTest, RejectsTruncatedVelodyneRecord) {
  const std::filesystem::path path = root_ / "0000000000.bin";
  std::ofstream(path, std::ios::binary).write("short", 5);

  try {
    static_cast<void>(loadVelodyneFrame(path));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kInvalidFrameFile);
  }
}

TEST_F(KittiIoTest, RejectsEmptyAndAllNonFiniteVelodyneFrames) {
  const std::filesystem::path empty_path = root_ / "empty.bin";
  const std::filesystem::path non_finite_path = root_ / "non_finite.bin";
  const std::ofstream empty_file(empty_path, std::ios::binary);
  ASSERT_TRUE(empty_file.good());
  writeLidarRecords(non_finite_path, {{std::numeric_limits<float>::infinity(), 0.0F, 0.0F, 0.0F}});

  EXPECT_THROW(static_cast<void>(loadVelodyneFrame(empty_path)), IoError);
  EXPECT_THROW(static_cast<void>(loadVelodyneFrame(non_finite_path)), IoError);
}

TEST_F(KittiIoTest, LoadsBgr8ImageAndRejectsDimensionMismatch) {
  const std::filesystem::path path = root_ / "0000000000.png";
  const cv::Mat source(3, 4, CV_8UC3, cv::Scalar(1, 2, 3));
  ASSERT_TRUE(cv::imwrite(path.string(), source));

  const cv::Mat loaded = loadBgrImage(path, 4, 3);

  EXPECT_EQ(loaded.type(), CV_8UC3);
  EXPECT_EQ(loaded.cols, 4);
  EXPECT_EQ(loaded.rows, 3);
  EXPECT_THROW(static_cast<void>(loadBgrImage(path, 5, 3)), IoError);
}

TEST_F(KittiIoTest, RejectsUndecodableImage) {
  const std::filesystem::path path = root_ / "0000000000.png";
  std::ofstream(path) << "not a PNG";

  try {
    static_cast<void>(loadBgrImage(path, 4, 3));
    FAIL() << "expected IoError";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kInvalidFrameFile);
  }
}

TEST_F(KittiIoTest, EnumeratesFramesByNumericIdNotLexicalName) {
  std::ofstream(root_ / "10.png") << "ten";
  std::ofstream(root_ / "2.png") << "two";
  std::ofstream(root_ / "ignored.txt") << "ignored";

  const std::vector<FrameFile> frames = enumerateNumericFrameFiles(root_, ".png");

  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].frame_id, 2U);
  EXPECT_EQ(frames[1].frame_id, 10U);
}

TEST_F(KittiIoTest, RejectsDuplicateAndNonNumericFrameIds) {
  std::ofstream(root_ / "1.png") << "one";
  std::ofstream(root_ / "01.png") << "also one";
  try {
    static_cast<void>(enumerateNumericFrameFiles(root_, ".png"));
    FAIL() << "expected duplicate frame ID error";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kInvalidFrameFile);
  }

  std::error_code error;
  std::filesystem::remove(root_ / "01.png", error);
  std::filesystem::remove(root_ / "1.png", error);
  std::ofstream(root_ / "frame.png") << "not numeric";
  try {
    static_cast<void>(enumerateNumericFrameFiles(root_, ".png"));
    FAIL() << "expected non-numeric frame ID error";
  } catch (const IoError& io_error) {
    EXPECT_EQ(io_error.code(), IoErrorCode::kInvalidFrameFile);
  }
}

TEST_F(KittiIoTest, RejectsTraversalAndUnsyncedDriveNames) {
  EXPECT_THROW(static_cast<void>(loadKittiSequenceLayout(root_, "../drive_sync")), IoError);
  EXPECT_THROW(static_cast<void>(loadKittiSequenceLayout(root_, "2011_09_26_drive_0005")), IoError);
}

TEST_F(KittiIoTest, LoadsSyncedLayoutAndChecksPerStreamCounts) {
  const std::filesystem::path drive = root_ / "2011_09_26_drive_0005_sync";
  const std::filesystem::path image_root = drive / "image_02";
  const std::filesystem::path lidar_root = drive / "velodyne_points";
  ASSERT_TRUE(std::filesystem::create_directories(image_root / "data"));
  ASSERT_TRUE(std::filesystem::create_directories(lidar_root / "data"));
  std::ofstream(image_root / "data/0000000000.png") << "enumeration only";
  std::ofstream(lidar_root / "data/0000000000.bin") << "enumeration only";
  std::ofstream(image_root / "timestamps.txt") << "2011-09-26 13:02:45.000000001\n";
  std::ofstream(lidar_root / "timestamps.txt") << "2011-09-26 13:02:45.000000002\n";

  const KittiSequenceLayout layout = loadKittiSequenceLayout(root_, "2011_09_26_drive_0005_sync");

  ASSERT_EQ(layout.image_frames.size(), 1U);
  ASSERT_EQ(layout.lidar_frames.size(), 1U);
  EXPECT_EQ(layout.image_frames[0].frame.frame_id, 0U);
  EXPECT_EQ(layout.lidar_frames[0].frame.frame_id, 0U);
  EXPECT_EQ(layout.lidar_frames[0].timestamp.timestamp.civilTime() -
                layout.image_frames[0].timestamp.timestamp.civilTime(),
            std::chrono::nanoseconds(1));
}

TEST_F(KittiIoTest, RejectsPerStreamFrameTimestampCountMismatch) {
  const std::filesystem::path drive = root_ / "2011_09_26_drive_0005_sync";
  const std::filesystem::path image_root = drive / "image_02";
  const std::filesystem::path lidar_root = drive / "velodyne_points";
  ASSERT_TRUE(std::filesystem::create_directories(image_root / "data"));
  ASSERT_TRUE(std::filesystem::create_directories(lidar_root / "data"));
  std::ofstream(image_root / "data/0000000000.png") << "enumeration only";
  std::ofstream(lidar_root / "data/0000000000.bin") << "enumeration only";
  std::ofstream(image_root / "timestamps.txt") << "2011-09-26 13:02:45.000000001\n"
                                                  "2011-09-26 13:02:45.000000002\n";
  std::ofstream(lidar_root / "timestamps.txt") << "2011-09-26 13:02:45.000000001\n";

  try {
    static_cast<void>(loadKittiSequenceLayout(root_, "2011_09_26_drive_0005_sync"));
    FAIL() << "expected frame/timestamp mismatch";
  } catch (const IoError& error) {
    EXPECT_EQ(error.code(), IoErrorCode::kInvalidLayout);
  }
}

} // namespace
} // namespace sfr::io
