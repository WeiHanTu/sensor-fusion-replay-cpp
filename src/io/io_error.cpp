#include "sfr/io/io_error.hpp"

namespace sfr::io {

IoError::IoError(IoErrorCode code, const std::string& message)
    : std::runtime_error(message), code_(code) {}

IoErrorCode IoError::code() const noexcept { return code_; }

} // namespace sfr::io
