#ifndef SFR_GEOMETRY_FRAME_GRAPH_HPP_
#define SFR_GEOMETRY_FRAME_GRAPH_HPP_

#include <optional>
#include <vector>

#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

class FrameGraph final {
public:
  static constexpr double kConsistencyTolerance = 1e-9;

  void addTransform(const RigidTransform& T_target_source);

  [[nodiscard]] std::optional<RigidTransform> lookup(const FrameId& target_frame,
                                                     const FrameId& source_frame) const;
  [[nodiscard]] std::size_t edgeCount() const noexcept;

private:
  std::vector<RigidTransform> edges_;
};

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_FRAME_GRAPH_HPP_
