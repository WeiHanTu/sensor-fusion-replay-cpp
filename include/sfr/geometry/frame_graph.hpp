#ifndef SFR_GEOMETRY_FRAME_GRAPH_HPP_
#define SFR_GEOMETRY_FRAME_GRAPH_HPP_

#include <optional>
#include <vector>

#include "sfr/geometry/rigid_transform.hpp"

namespace sfr::geometry {

/// Owns a validated set of directed rigid-transform edges.
///
/// Lookup may traverse stored edges in either direction. Stored insertion order
/// provides deterministic breadth-first traversal; adding duplicate, redundant,
/// or path-inconsistent edges fails closed.
class FrameGraph final {
public:
  static constexpr double kConsistencyTolerance = 1e-9;

  /// Copies one directed edge after checking graph consistency.
  ///
  /// @throws GeometryError For self, duplicate, redundant, or inconsistent edges.
  void addTransform(const RigidTransform& T_target_source);

  /// Returns the composed shortest transform from `source_frame` to `target_frame`.
  ///
  /// Equal frames return identity. Missing connectivity is expected absence and
  /// returns `std::nullopt`; the graph retains ownership of all stored edges.
  [[nodiscard]] std::optional<RigidTransform> lookup(const FrameId& target_frame,
                                                     const FrameId& source_frame) const;

  /// Returns the number of directly stored directed edges in O(1) time.
  [[nodiscard]] std::size_t edgeCount() const noexcept;

private:
  std::vector<RigidTransform> edges_;
};

} // namespace sfr::geometry

#endif // SFR_GEOMETRY_FRAME_GRAPH_HPP_
