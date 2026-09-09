#ifndef SFR_IO_IO_ERROR_HPP_
#define SFR_IO_IO_ERROR_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

namespace sfr::io {

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

class IoError final : public std::runtime_error {
public:
  IoError(IoErrorCode code, const std::string& message);

  [[nodiscard]] IoErrorCode code() const noexcept;

private:
  IoErrorCode code_;
};

} // namespace sfr::io

#endif // SFR_IO_IO_ERROR_HPP_
