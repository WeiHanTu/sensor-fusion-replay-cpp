#ifndef SFR_IO_IO_ERROR_HPP_
#define SFR_IO_IO_ERROR_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

namespace sfr::io {

/// Stable categories for local input, layout, and decode failures.
enum class IoErrorCode : std::uint8_t {
  kFileOpen,
  kMalformedLine,
  kMissingKey,
  kDuplicateKey,
  kWrongValueCount,
  kNonFiniteValue,
  kInvalidCalibration,
  kInvalidLayout,
  kInvalidFrameFile,
  kInvalidTimestamp,
  kLimitExceeded,
  kUnsupportedPlatform,
};

/// Typed I/O exception with a machine-checkable category.
class IoError final : public std::runtime_error {
public:
  /// Copies `message` into `std::runtime_error` and stores `code`.
  IoError(IoErrorCode code, const std::string& message);

  /// Returns the stored error category.
  [[nodiscard]] IoErrorCode code() const noexcept;

private:
  IoErrorCode code_;
};

} // namespace sfr::io

#endif // SFR_IO_IO_ERROR_HPP_
