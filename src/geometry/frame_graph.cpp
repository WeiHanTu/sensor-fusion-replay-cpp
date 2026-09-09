#include "sfr/geometry/frame_graph.hpp"

#include <map>
#include <queue>
#include <sstream>

namespace sfr::geometry {

namespace {

[[nodiscard]] bool transformsAgree(const RigidTransform& first, const RigidTransform& second) {
  return (first.rotation() - second.rotation()).norm() <= FrameGraph::kConsistencyTolerance &&
         (first.translationMeters() - second.translationMeters()).norm() <=
             FrameGraph::kConsistencyTolerance;
}

} // namespace

void FrameGraph::addTransform(const RigidTransform& T_target_source) {
  if (T_target_source.targetFrame() == T_target_source.sourceFrame()) {
    throw GeometryError(GeometryErrorCode::kRedundantEdge,
                        "a frame graph edge must connect two distinct frames");
  }

  for (const RigidTransform& edge : edges_) {
    if (edge.targetFrame() == T_target_source.targetFrame() &&
        edge.sourceFrame() == T_target_source.sourceFrame()) {
      throw GeometryError(GeometryErrorCode::kDuplicateEdge, "duplicate directed frame graph edge");
    }
  }

  const std::optional<RigidTransform> existing =
      lookup(T_target_source.targetFrame(), T_target_source.sourceFrame());
  if (existing.has_value()) {
    if (transformsAgree(*existing, T_target_source)) {
      throw GeometryError(GeometryErrorCode::kRedundantEdge,
                          "edge is redundant with an existing frame graph path");
    }
    throw GeometryError(GeometryErrorCode::kInconsistentEdge,
                        "edge is inconsistent with an existing frame graph path");
  }

  edges_.push_back(T_target_source);
}

std::optional<RigidTransform> FrameGraph::lookup(const FrameId& target_frame,
                                                 const FrameId& source_frame) const {
  if (target_frame == source_frame) {
    return RigidTransform::Identity(source_frame);
  }

  std::queue<FrameId> frontier;
  std::map<FrameId, RigidTransform> T_visited_source;
  frontier.push(source_frame);
  T_visited_source.emplace(source_frame, RigidTransform::Identity(source_frame));

  while (!frontier.empty()) {
    const FrameId current_frame = frontier.front();
    frontier.pop();
    const RigidTransform& T_current_source = T_visited_source.at(current_frame);

    for (const RigidTransform& edge : edges_) {
      std::optional<RigidTransform> T_neighbor_current;
      if (edge.sourceFrame() == current_frame) {
        T_neighbor_current = edge;
      } else if (edge.targetFrame() == current_frame) {
        T_neighbor_current = edge.inverse();
      } else {
        continue;
      }

      const FrameId& neighbor_frame = T_neighbor_current->targetFrame();
      if (T_visited_source.contains(neighbor_frame)) {
        continue;
      }

      RigidTransform T_neighbor_source = compose(*T_neighbor_current, T_current_source);
      if (neighbor_frame == target_frame) {
        return T_neighbor_source;
      }
      frontier.push(neighbor_frame);
      T_visited_source.emplace(neighbor_frame, std::move(T_neighbor_source));
    }
  }

  return std::nullopt;
}

std::size_t FrameGraph::edgeCount() const noexcept { return edges_.size(); }

} // namespace sfr::geometry
