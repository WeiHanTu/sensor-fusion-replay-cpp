#include "sfr/viz/extrinsic_recovery_artifact.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <sstream>
#include <utility>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "sfr/core/build_info.hpp"

namespace sfr::viz {
namespace {
using Json = nlohmann::json;
using namespace geometry;

[[noreturn]] void invalid(const std::string& message) {
  throw ArtifactError(ArtifactErrorCode::kInvalidRequest, message);
}

class Staging final {
public:
  explicit Staging(std::filesystem::path path) : path_(std::move(path)) {
    if (!std::filesystem::create_directory(path_)) {
      throw ArtifactError(ArtifactErrorCode::kExistingRun, "recovery stage already exists");
    }
  }
  Staging(const Staging&) = delete;
  Staging& operator=(const Staging&) = delete;
  ~Staging() {
    if (!committed_) {
      std::error_code error;
      std::filesystem::remove_all(path_, error);
    }
  }
  void commit() noexcept { committed_ = true; }

private:
  std::filesystem::path path_;
  bool committed_{false};
};

[[nodiscard]] double milliseconds(const std::chrono::nanoseconds duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}
[[nodiscard]] Json number(const std::optional<double>& value) {
  if (value.has_value() && !std::isfinite(value.value())) {
    invalid("non-finite report number");
  }
  return value.has_value() ? Json(value.value()) : Json(nullptr);
}
[[nodiscard]] Json counts(const ProjectionCounts& c) {
  if (c.input_points != c.visible_points + c.non_finite_input + c.behind_or_too_near +
                            c.non_positive_homogeneous_depth + c.outside_image) {
    invalid("projection accounting mismatch");
  }
  return {{"input_points", c.input_points},
          {"visible", c.visible_points},
          {"invalid", c.input_points - c.visible_points},
          {"non_finite_input", c.non_finite_input},
          {"behind_or_too_near", c.behind_or_too_near},
          {"non_positive_homogeneous_depth", c.non_positive_homogeneous_depth},
          {"outside_image", c.outside_image}};
}
[[nodiscard]] Json parameters(const ExtrinsicCorrection& c) {
  return {{"yaw_camera_y_deg", c.yaw_camera_y_rad * 180.0 / std::numbers::pi},
          {"yaw_camera_y_rad", c.yaw_camera_y_rad},
          {"translation_camera_x_m", c.translation_camera_x_m}};
}
template <typename Matrix> [[nodiscard]] Json matrix(const Matrix& m) {
  Json rows = Json::array();
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    Json row = Json::array();
    for (Eigen::Index c = 0; c < m.cols(); ++c) {
      row.push_back(m(r, c));
    }
    rows.push_back(row);
  }
  return rows;
}
[[nodiscard]] Json transform(const RigidTransform& t) {
  return {{"rotation", matrix(t.rotation())},
          {"translation_m", matrix(t.translationMeters())},
          {"target", "camera_rect_00"},
          {"source", "lidar"}};
}
[[nodiscard]] std::string status(const RecoverySearchStatus value) {
  switch (value) {
  case RecoverySearchStatus::kGridComplete:
    return "grid_complete";
  case RecoverySearchStatus::kBoundLimited:
    return "bound_limited";
  case RecoverySearchStatus::kRefinementLimited:
    return "refinement_limited";
  case RecoverySearchStatus::kNoValidCandidate:
    return "no_valid_candidate";
  }
  invalid("unknown search status");
}
[[nodiscard]] std::string projectionStatus(const ProjectionStatus value) {
  switch (value) {
  case ProjectionStatus::kVisible:
    return "visible";
  case ProjectionStatus::kNonFiniteInput:
    return "non_finite_input";
  case ProjectionStatus::kBehindOrTooNear:
    return "behind_or_too_near";
  case ProjectionStatus::kNonPositiveHomogeneousDepth:
    return "non_positive_homogeneous_depth";
  case ProjectionStatus::kOutsideImage:
    return "outside_image";
  }
  invalid("unknown projection status");
}
[[nodiscard]] Json metrics(const RecoveryResidualMetrics& m) {
  return {{"frame_id", m.frame_id},
          {"counts", counts(m.counts)},
          {"zero_weight", m.zero_weight},
          {"common_visible", m.common_visible},
          {"median_px", number(m.median_px)},
          {"p95_px", number(m.p95_px)},
          {"rmse_px", number(m.rmse_px)},
          {"samples", m.counts.visible_points},
          {"conditional_visible_only", m.counts.input_points != m.counts.visible_points}};
}
[[nodiscard]] Json table(const RecoveryTransformMetrics& t) {
  Json frames = Json::array();
  for (const auto& f : t.frames) {
    frames.push_back(metrics(f));
  }
  return {{"pooled", metrics(t.pooled)}, {"frames", frames}};
}
[[nodiscard]] Json comparison(const RecoveryComparison& c) {
  return {{"nominal", table(c.nominal)},
          {"recovered", c.recovered.has_value() ? table(c.recovered.value()) : Json(nullptr)},
          {"oracle", table(c.oracle)},
          {"primary_weighting", "unweighted_all_retained_records"},
          {"percentile", "nearest_rank_clamped"},
          {"points_evaluated", c.points_evaluated}};
}
[[nodiscard]] Json residuals(const RecoveryTransformMetrics& t) {
  Json rows = Json::array();
  for (const auto& r : t.residuals) {
    rows.push_back(
        {{"frame_id", r.frame_id},
         {"source_index", r.source_index},
         {"weight", r.weight},
         {"target_uv", Json::array({r.target.u_px, r.target.v_px})},
         {"residual_uv", r.predicted.has_value()
                             ? Json::array({r.target.u_px - r.predicted.value().u_px,
                                            r.target.v_px - r.predicted.value().v_px})
                             : Json(nullptr)},
         {"status", projectionStatus(r.status)},
         {"radial_error_px", number(r.radial_error_px)},
         {"predicted_uv", r.predicted.has_value()
                              ? Json::array({r.predicted.value().u_px, r.predicted.value().v_px})
                              : Json(nullptr)}});
  }
  return rows;
}
[[nodiscard]] Json residualComparison(const RecoveryComparison& c) {
  return {{"nominal", residuals(c.nominal)},
          {"recovered", c.recovered.has_value() ? residuals(c.recovered.value()) : Json(nullptr)},
          {"oracle", residuals(c.oracle)}};
}
[[nodiscard]] Json axis(const RecoveryAxisGrid& a, const double unit_scale) {
  return {{"minimum", a.minimum * unit_scale},
          {"maximum", a.maximum * unit_scale},
          {"coarse_step", a.coarse_step * unit_scale},
          {"fine_step", a.fine_step * unit_scale}};
}
void writeJson(const std::filesystem::path& file, const Json& value) {
  std::ofstream stream(file);
  stream << value.dump(2) << '\n';
  stream.close();
  if (!stream) {
    throw ArtifactError(ArtifactErrorCode::kWriteFailure, "unable to close JSON output");
  }
}
void writeImage(const std::filesystem::path& file, const cv::Mat& image) {
  if (!cv::imwrite(file.string(), image)) {
    throw ArtifactError(ArtifactErrorCode::kImageEncoding, "image encoding failed");
  }
}
void label(cv::Mat& image, const std::string& text, const int x, const int y,
           const double scale = 0.5, const cv::Scalar& color = {235, 235, 235}) {
  cv::putText(image, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}
[[nodiscard]] std::string formatted(const double value) {
  std::ostringstream s;
  s.precision(5);
  s << value;
  return s.str();
}
[[nodiscard]] cv::Mat lossPlot(const ExtrinsicRecoveryArtifactRequest& request) {
  cv::Mat image(620, 1200, CV_8UC3, cv::Scalar(24, 24, 24));
  const std::string loss =
      request.config.loss.kind == RecoveryLoss::kHuber ? "Huber" : "squared L2";
  label(image,
        "Authored synthetic loss surface | " + loss +
            " | delta=" + formatted(request.config.loss.huber_delta_px) + " px",
        20, 28, 0.6);
  for (int panel = 0; panel < 2; ++panel) {
    std::vector<const RecoveryCandidateRow*> rows;
    std::set<std::int64_t> ys, xs;
    double min_loss = std::numeric_limits<double>::infinity();
    double max_loss = 0.0;
    for (const auto& row : request.search.candidates) {
      if (panel == 0 ? row.coarse : row.fine) {
        rows.push_back(&row);
        ys.insert(row.yaw_tick);
        xs.insert(row.translation_tick);
        if (row.evaluation.loss_sum.has_value()) {
          const double l = std::log1p(row.evaluation.loss_sum.value());
          min_loss = std::min(min_loss, l);
          max_loss = std::max(max_loss, l);
        }
      }
    }
    const int left = 75 + panel * 600;
    label(image, panel == 0 ? "Coarse grid" : "Fine rectangle (no interpolation)", left, 70);
    if (rows.empty()) {
      label(image, "Unavailable: no valid coarse candidate", left, 120);
      continue;
    }
    std::vector<std::int64_t> yvalues(ys.begin(), ys.end()), xvalues(xs.begin(), xs.end());
    for (const auto* row : rows) {
      const auto yi =
          std::lower_bound(yvalues.begin(), yvalues.end(), row->yaw_tick) - yvalues.begin();
      const auto xi =
          std::lower_bound(xvalues.begin(), xvalues.end(), row->translation_tick) - xvalues.begin();
      const int x0 =
          left + static_cast<int>(xi * 450 / static_cast<std::ptrdiff_t>(xvalues.size()));
      const int x1 =
          left + static_cast<int>((xi + 1) * 450 / static_cast<std::ptrdiff_t>(xvalues.size()));
      const int y0 = 90 + static_cast<int>((static_cast<std::ptrdiff_t>(yvalues.size()) - yi - 1) *
                                           430 / static_cast<std::ptrdiff_t>(yvalues.size()));
      const int y1 = 90 + static_cast<int>((static_cast<std::ptrdiff_t>(yvalues.size()) - yi) *
                                           430 / static_cast<std::ptrdiff_t>(yvalues.size()));
      cv::Scalar color(0, 0, 0); // explicit rejected cell, never interpolated
      if (row->evaluation.loss_sum.has_value()) {
        const double fraction =
            max_loss > min_loss
                ? (std::log1p(row->evaluation.loss_sum.value()) - min_loss) / (max_loss - min_loss)
                : 0.0;
        color = {40.0 + 180.0 * fraction, 190.0 * (1.0 - fraction), 40.0 + 180.0 * fraction};
      }
      cv::rectangle(image, {x0, y0}, {x1 - 1, y1 - 1}, color, cv::FILLED);
      if (!row->evaluation.loss_sum.has_value()) {
        cv::line(image, {x0, y0}, {x1 - 1, y1 - 1}, {30, 30, 255}, 1);
        cv::line(image, {x0, y1 - 1}, {x1 - 1, y0}, {30, 30, 255}, 1);
      }
      if (request.search.best.has_value() &&
          row->evaluation.correction.yaw_camera_y_rad ==
              request.search.best.value().yaw_camera_y_rad &&
          row->evaluation.correction.translation_camera_x_m ==
              request.search.best.value().translation_camera_x_m) {
        cv::circle(image, {(x0 + x1) / 2, (y0 + y1) / 2}, 7, {255, 255, 255}, 2);
      }
    }
    const auto yawValue = [&](const std::int64_t tick) {
      return (request.config.yaw.minimum +
              static_cast<double>(tick) * request.config.yaw.fine_step) *
             180.0 / std::numbers::pi;
    };
    const auto txValue = [&](const std::int64_t tick) {
      return request.config.translation.minimum +
             static_cast<double>(tick) * request.config.translation.fine_step;
    };
    label(image, formatted(yawValue(yvalues.back())), left - 58, 105);
    label(image, formatted(yawValue(yvalues.front())), left - 58, 519);
    label(image, formatted(txValue(xvalues.front())), left, 545);
    label(image, formatted(txValue(xvalues.back())), left + 380, 545);
    label(image, "camera +x translation (m)", left + 110, 570);
    label(image, "yaw +y (deg)", left, 85, 0.4);
    label(image,
          std::isfinite(min_loss) ? "log1p(loss): " + formatted(min_loss) + " green -> " +
                                        formatted(max_loss) + " purple"
                                  : "No finite loss: every cell rejected",
          left, 597, 0.4);
  }
  label(image,
        "White circle: selected | black/red X: rejected | oracle: yaw=" +
            formatted(request.hidden.yaw_camera_y_rad * 180.0 / std::numbers::pi) +
            " deg tx=" + formatted(request.hidden.translation_camera_x_m) + " m",
        20, 48, 0.45);
  return image;
}
[[nodiscard]] cv::Mat residualPlot(const ExtrinsicRecoveryArtifactRequest& request) {
  cv::Mat image(430, 1200, CV_8UC3, cv::Scalar(24, 24, 24));
  label(image, "Unweighted radial error (px) | clean oracle targets | authored synthetic frames",
        20, 30, 0.65);
  for (int panel = 0; panel < 2; ++panel) {
    const auto& c = panel == 0 ? request.train_clean : request.validation_clean;
    const std::array<const RecoveryTransformMetrics*, 3> tables{
        &c.nominal, c.recovered.has_value() ? &c.recovered.value() : nullptr, &c.oracle};
    const std::array<std::string, 3> names{"nominal", "recovered", "oracle"};
    const int left = 35 + 600 * panel;
    label(image, panel == 0 ? "Train frames (optimization data)" : "Held-out frames (never fitted)",
          left, 65, 0.55);
    double maximum = 1.0;
    for (const auto* t : tables) {
      if (t != nullptr && t->pooled.p95_px.has_value()) {
        maximum = std::max(maximum, t->pooled.p95_px.value());
      }
    }
    for (std::size_t col = 0; col < tables.size(); ++col) {
      const int x = left + static_cast<int>(col) * 180;
      label(image, names[col], x, 95);
      const auto* t = tables[col];
      if (t == nullptr) {
        label(image, "Unavailable", x, 135);
        continue;
      }
      const std::array<std::optional<double>, 3> values{t->pooled.median_px, t->pooled.p95_px,
                                                        t->pooled.rmse_px};
      const std::array<std::string, 3> labels{"median", "p95", "RMSE"};
      for (std::size_t m = 0; m < values.size(); ++m) {
        const int y = 125 + static_cast<int>(m) * 70;
        label(image,
              labels[m] + ": " + (values[m].has_value() ? formatted(values[m].value()) : "null"), x,
              y, 0.43);
        if (values[m].has_value()) {
          cv::rectangle(
              image, {x, y + 10},
              {x + static_cast<int>(140.0 * std::min(1.0, values[m].value() / maximum)), y + 27},
              {60, 190, 120}, cv::FILLED);
        }
      }
      label(image, "visible=" + std::to_string(t->pooled.counts.visible_points), x, 350, 0.42);
      label(image,
            "invalid=" +
                std::to_string(t->pooled.counts.input_points - t->pooled.counts.visible_points),
            x, 375, 0.42,
            t->pooled.counts.visible_points == t->pooled.counts.input_points
                ? cv::Scalar(235, 235, 235)
                : cv::Scalar(50, 50, 255));
    }
    label(image, "Bar full scale=" + formatted(maximum) + " px", left, 410, 0.43);
  }
  return image;
}

} // namespace

std::filesystem::path writeExtrinsicRecoveryArtifacts(const ExtrinsicRecoveryArtifactRequest& r) {
  try {
    if (r.run_id.empty() || r.run_id == "." || r.run_id == ".." ||
        !std::ranges::all_of(r.run_id, [](const unsigned char c) {
          return std::isalnum(c) != 0 || c == '-' || c == '_';
        })) {
      invalid("run ID must be a safe filename component");
    }
    const auto input = std::filesystem::weakly_canonical(r.fixture_root);
    const auto output = std::filesystem::weakly_canonical(r.output_root / r.run_id);
    const auto relative = output.lexically_relative(input);
    if (relative.empty() || (relative.begin() != relative.end() && *relative.begin() != "..")) {
      invalid("output may not resolve inside fixture");
    }
    std::set<std::uint64_t> train, validation;
    for (const auto& f : r.training) {
      if (!train.insert(f.frame_id).second) {
        invalid("duplicate training frame");
      }
    }
    for (const auto& f : r.validation) {
      if (!validation.insert(f.frame_id).second || train.contains(f.frame_id)) {
        invalid("overlapping/duplicate validation frame");
      }
    }
    if (train.size() < 2U || validation.empty() ||
        std::vector<std::uint64_t>(train.begin(), train.end()) != r.search.consumed_frame_ids) {
      invalid("split/consumed training IDs mismatch");
    }
    if (r.search.points_evaluated !=
        r.search.candidates.size() * r.train_observed.nominal.pooled.counts.input_points) {
      invalid("search point accounting mismatch");
    }
    const auto destination = r.output_root / r.run_id;
    const auto temporary = r.output_root / (r.run_id + ".tmp");
    if (std::filesystem::exists(destination) && !std::filesystem::is_empty(destination)) {
      if (!r.overwrite) {
        throw ArtifactError(ArtifactErrorCode::kExistingRun, "run already exists; use --overwrite");
      }
      std::ifstream old(destination / "report.json");
      Json previous;
      if (!old || !(old >> previous) ||
          previous.value("schema_name", "") != "sfr.extrinsic_recovery") {
        invalid("refusing to overwrite unrelated directory");
      }
    }
    std::filesystem::create_directories(r.output_root);
    Staging stage(temporary);
    const auto render_start = std::chrono::steady_clock::now();
    const auto surface_plot = lossPlot(r);
    const auto residual_plot = residualPlot(r);
    std::map<std::string, cv::Mat> overlays;
    Json overlay_accounting = Json::array();
    std::set<std::uint64_t> image_ids;
    for (const auto& image : r.images) {
      if (!image_ids.insert(image.frame_id).second) {
        invalid("duplicate frame image");
      }
      const auto find = [&](const auto& frames) {
        return std::ranges::find(frames, image.frame_id, &ExtrinsicObservationFrame::frame_id);
      };
      const auto ti = find(r.training);
      const auto vi = find(r.validation);
      if (ti == r.training.end() && vi == r.validation.end()) {
        invalid("image has no selected frame");
      }
      const auto& observations = ti != r.training.end() ? ti->observations : vi->observations;
      std::vector<core::PointXYZI> points;
      points.reserve(observations.size());
      for (const auto& observation : observations) {
        points.push_back(observation.point_lidar);
      }
      std::vector<std::pair<std::string, ExtrinsicCorrection>> models{{"nominal", {}},
                                                                      {"oracle", r.hidden}};
      if (r.search.best.has_value()) {
        models.emplace_back("recovered", r.search.best.value());
      }
      for (const auto& [name, correction] : models) {
        const auto projected = projectPointCloud(
            points, applyExtrinsicCorrection(r.nominal, correction), r.projection);
        auto rendered = renderDepthOverlay(image.image_bgr8, projected.visible_points);
        label(rendered.image_bgr8,
              "SYNTHETIC | frame=" + std::to_string(image.frame_id) + " | " + name +
                  (ti != r.training.end() ? " | TRAIN" : " | HELD OUT"),
              20, 56, 0.6);
        const std::string path = "overlays/" + std::to_string(image.frame_id) + "/" + name + ".png";
        overlays.emplace(path, rendered.image_bgr8);
        overlay_accounting.push_back({{"path", path},
                                      {"projection", counts(projected.counts)},
                                      {"rendered_pixels", rendered.rendered_pixels},
                                      {"occluded_points", rendered.occluded_points}});
      }
    }
    if (r.images.size() != train.size() + validation.size()) {
      invalid("one image per selected frame required");
    }
    const auto rendering_duration = std::chrono::steady_clock::now() - render_start;
    const auto write_start = std::chrono::steady_clock::now();
    Json candidate_rows = Json::array();
    for (const auto& row : r.search.candidates) {
      const auto& e = row.evaluation;
      if (e.positive_weight + e.zero_weight != e.projection_counts.input_points ||
          e.positive_weight_visible + e.positive_weight_invalid != e.positive_weight ||
          e.loss_sum.has_value() != (e.positive_weight_invalid == 0U && !e.numeric_failure)) {
        invalid("candidate visibility/weight/loss accounting mismatch");
      }
      candidate_rows.push_back(
          {{"yaw_tick", row.yaw_tick},
           {"translation_tick", row.translation_tick},
           {"parameters", parameters(e.correction)},
           {"coarse", row.coarse},
           {"fine", row.fine},
           {"loss_sum", number(e.loss_sum)},
           {"loss_mean", number(e.loss_mean)},
           {"rejection_reason",
            e.numeric_failure ? "numeric_failure"
                              : (e.positive_weight_invalid > 0U ? "active_invisible" : "none")},
           {"counts", counts(e.projection_counts)},
           {"positive_weight", e.positive_weight},
           {"zero_weight", e.zero_weight},
           {"positive_weight_visible", e.positive_weight_visible},
           {"positive_weight_invalid", e.positive_weight_invalid},
           {"total_weight", e.total_weight}});
    }
    writeJson(temporary / "loss_surface.json", {{"schema_name", "sfr.extrinsic_loss_surface"},
                                                {"schema_version", "1.0.0"},
                                                {"rows", candidate_rows}});
    writeJson(temporary / "residuals.json",
              {{"schema_name", "sfr.extrinsic_residuals"},
               {"schema_version", "1.0.0"},
               {"train",
                {{"observed", residualComparison(r.train_observed)},
                 {"clean", residualComparison(r.train_clean)}}},
               {"validation",
                {{"observed", residualComparison(r.validation_observed)},
                 {"clean", residualComparison(r.validation_clean)}}}});
    writeImage(temporary / "loss_surface.png", surface_plot);
    writeImage(temporary / "residual_comparison.png", residual_plot);
    for (const auto& [path, image] : overlays) {
      std::filesystem::create_directories((temporary / path).parent_path());
      writeImage(temporary / path, image);
    }
    const auto staging_duration = std::chrono::steady_clock::now() - write_start;
    const auto gate = assessExtrinsicRecovery(r.search, r.config, r.hidden, r.validation_clean);
    Json frame_metadata = Json::array();
    for (const auto& f : r.fixture.frames) {
      frame_metadata.push_back(
          {{"frame_id", f.frame_id},
           {"scene_identity", f.scene_identity},
           {"points_identity", f.points_identity},
           {"observations_identity", f.observations_identity},
           {"generation_counts", counts(f.generation_counts)},
           {"outlier_count", f.outlier_count},
           {"noise_sample_mean", Json::array({f.noise_mean.u_px, f.noise_mean.v_px})}});
    }
    const auto& d = r.search.identifiability;
    const auto& b = core::kBuildInfo;
    Json report{
        {"schema_name", "sfr.extrinsic_recovery"},
        {"schema_version", "1.0.0"},
        {"status", "complete"},
        {"run_id", r.run_id},
        {"claim_boundary", "project-authored synthetic known-correspondence two-parameter "
                           "recovery; not real/KITTI calibration, HIL, sim-to-real or production"},
        {"build",
         {{"git_commit", b.git_commit},
          {"git_dirty", b.git_dirty},
          {"build_type", b.build_type},
          {"compiler", b.compiler}}},
        {"host", {{"os", b.host_os}, {"arch", b.host_arch}, {"cpu", b.host_cpu}}},
        {"fixture",
         {{"id", r.fixture.id},
          {"version", r.fixture.version},
          {"content_identity", r.fixture.content_identity},
          {"generator_version", r.fixture.generator_version},
          {"seed", r.fixture.seed},
          {"geometry", r.fixture.geometry},
          {"provenance", "project-authored points/images/UV; no KITTI data"},
          {"frames", frame_metadata},
          {"noise",
           {{"kind", "paired_antithetic_uniform_splitmix64_top53"},
            {"theoretical_stddev_px", r.fixture.noise_stddev_px}}},
          {"outliers",
           {{"training_fraction", r.fixture.outlier_fraction},
            {"offset_uv_px", Json::array({40.0, -30.0})}}}}},
        {"model",
         {{"parameter_order", Json::array({"yaw_camera_y", "translation_camera_x"})},
          {"units", Json::array({"rad", "m"})},
          {"application", "Delta_camera=[Ry(yaw),(tx,0,0)]; T_candidate=Delta_camera*T_nominal"},
          {"nominal_correction", parameters({})},
          {"nominal_transform", transform(r.nominal)},
          {"hidden_parameters", parameters(r.hidden)},
          {"hidden_transform", transform(applyExtrinsicCorrection(r.nominal, r.hidden))},
          {"recovered_parameters",
           r.search.best.has_value() ? parameters(r.search.best.value()) : Json(nullptr)},
          {"recovered_transform", r.search.best.has_value() ? transform(applyExtrinsicCorrection(
                                                                  r.nominal, r.search.best.value()))
                                                            : Json(nullptr)},
          {"projection_matrix", matrix(r.projection.matrix())},
          {"dimensions",
           Json::array({r.projection.imageWidthPixels(), r.projection.imageHeightPixels()})},
          {"z_min", r.projection.minimumDepthMeters()}}},
        {"split",
         {{"train_frame_ids", train},
          {"validation_frame_ids", validation},
          {"consumed_train_ids", r.search.consumed_frame_ids},
          {"frame_content_identities", frame_metadata},
          {"disjoint_checked", true},
          {"held_out_loaded_after_search", true}}},
        {"search",
         {{"config",
           {{"yaw_deg", axis(r.config.yaw, 180.0 / std::numbers::pi)},
            {"translation_m", axis(r.config.translation, 1.0)},
            {"max_candidates", r.config.max_candidates},
            {"max_point_projections", r.config.max_point_projections},
            {"rank_relative_tolerance", r.config.rank_relative_tolerance},
            {"condition_limit", r.config.condition_limit},
            {"refinement_half_width", "one_coarse_step"},
            {"grid_integrality_tolerance", 1e-9}}},
          {"loss_name", r.config.loss.kind == RecoveryLoss::kHuber ? "huber" : "squared_l2"},
          {"huber_delta_px", r.config.loss.huber_delta_px},
          {"loss_sum", number(r.search.loss_sum)},
          {"loss_mean", number(r.search.loss_mean)},
          {"total_weight", r.search.total_weight},
          {"best_parameters",
           r.search.best.has_value() ? parameters(r.search.best.value()) : Json(nullptr)},
          {"result_status", status(r.search.status)},
          {"global_bound_hits",
           {{"yaw", r.search.yaw_bound_hit}, {"translation", r.search.translation_bound_hit}}},
          {"refinement_edge_hit", r.search.refinement_edge_hit},
          {"evaluations", r.search.candidates.size()},
          {"points_evaluated", r.search.points_evaluated},
          {"identifiability",
           {{"available", d.available},
            {"rank", d.rank},
            {"sigma_max", d.sigma_max},
            {"sigma_min", d.sigma_min},
            {"condition", number(d.condition)},
            {"poorly_identifiable", d.poorly_identifiable},
            {"difference_method",
             !d.available ? "unavailable"
                          : (d.used_one_sided_difference ? "one_sided_or_mixed" : "central")},
            {"points_evaluated", d.points_evaluated}}}}},
        {"parameter_errors",
         {{"signed_yaw_deg", number(gate.signed_yaw_error_deg)},
          {"absolute_yaw_deg", gate.signed_yaw_error_deg.has_value()
                                   ? Json(std::abs(gate.signed_yaw_error_deg.value()))
                                   : Json(nullptr)},
          {"signed_translation_m", number(gate.signed_translation_error_m)},
          {"absolute_translation_m", gate.signed_translation_error_m.has_value()
                                         ? Json(std::abs(gate.signed_translation_error_m.value()))
                                         : Json(nullptr)}}},
        {"metrics",
         {{"train",
           {{"observed", comparison(r.train_observed)}, {"clean", comparison(r.train_clean)}}},
          {"validation",
           {{"observed", comparison(r.validation_observed)},
            {"clean", comparison(r.validation_clean)}}}}},
        {"timing",
         {{"clock", "steady_clock"},
          {"loading_ms", milliseconds(r.loading_duration)},
          {"search_ms", milliseconds(r.search.search_duration)},
          {"diagnostics_ms", milliseconds(r.search.diagnostics_duration)},
          {"evaluation_ms", milliseconds(r.evaluation_duration)},
          {"reproducibility_ms", milliseconds(r.reproducibility_duration)},
          {"rendering_ms", milliseconds(rendering_duration)},
          {"staging_write_ms", milliseconds(staging_duration)},
          {"publication_scope",
           "staging_write excludes final report write/rename; no fsync; overwrite "
           "remove-then-rename is nontransactional; not runtime sojourn"}}},
        {"reproducibility",
         {{"checked", r.deterministic_replay_checked},
          {"candidate_evaluations",
           r.deterministic_replay_checked ? r.search.candidates.size() : 0U},
          {"point_projections", r.deterministic_replay_checked ? r.search.points_evaluated : 0U},
          {"equal",
           r.deterministic_replay_checked ? Json(r.deterministic_replay_equal) : Json(nullptr)},
          {"comparison", "canonical candidate keys, memberships, losses, counts and selected "
                         "result; timing/provenance excluded"}}},
        {"accounting",
         {{"candidate_points_match", true},
          {"projection_partitions_checked", true},
          {"overlays", overlay_accounting}}},
        {"acceptance",
         {{"profile", "frozen_noiseless_V1"},
          {"parameter_gate", gate.parameter_gate},
          {"full_coverage", gate.full_coverage},
          {"held_out_gate", gate.held_out_gate},
          {"search_and_identifiability_gate", gate.search_and_identifiability_gate},
          {"passed", gate.passed},
          {"interpretation",
           "noisy/outlier/diagnostic cases report this strict clean gate without retuning"}}},
        {"artifacts",
         {{"surface", "loss_surface.json"},
          {"residuals", "residuals.json"},
          {"surface_plot", "loss_surface.png"},
          {"residual_plot", "residual_comparison.png"},
          {"overlays", overlay_accounting},
          {"recovered_available", r.search.best.has_value()}}}};
    writeJson(temporary / "report.json", report);
    if (std::filesystem::exists(destination)) {
      std::filesystem::remove_all(destination);
    }
    std::filesystem::rename(temporary, destination);
    stage.commit();
    return destination;
  } catch (const std::filesystem::filesystem_error& error) {
    throw ArtifactError(ArtifactErrorCode::kFilesystem, error.what());
  } catch (const cv::Exception& error) {
    throw ArtifactError(ArtifactErrorCode::kImageEncoding, error.what());
  } catch (const Json::exception& error) {
    throw ArtifactError(ArtifactErrorCode::kInvalidRequest, error.what());
  }
}

} // namespace sfr::viz
