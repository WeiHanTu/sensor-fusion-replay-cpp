#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "recovery_workflow.hpp"
#include "sfr/io/io_error.hpp"

namespace {
using Json = nlohmann::json;
using namespace sfr;

struct Case final {
  std::string id;
  std::string dataset_id;
  io::ExtrinsicFixtureConfig fixture;
  geometry::ExtrinsicSearchConfig search;
  std::string expected; // recovery, bound, invalid, diagnostic, offgrid, noise, paired
};
[[nodiscard]] io::ExtrinsicFixtureConfig authored(const double yaw_deg, const double tx) {
  io::ExtrinsicFixtureConfig config;
  config.hidden = {yaw_deg * std::numbers::pi / 180.0, tx};
  return config;
}
[[nodiscard]] std::vector<Case> cases() {
  std::vector<Case> result;
  const auto add = [&](const std::string& id, const io::ExtrinsicFixtureConfig& fixture,
                       const std::string& expected,
                       const geometry::ExtrinsicSearchConfig& search =
                           geometry::ExtrinsicSearchConfig{},
                       const std::string& dataset = "") {
    result.push_back({id, dataset.empty() ? id : dataset, fixture, search, expected});
  };
  geometry::ExtrinsicSearchConfig l2;
  l2.loss.kind = geometry::RecoveryLoss::kSquaredL2;
  add("A_zero", authored(0.0, 0.0), "recovery", l2);
  add("A_yaw_positive", authored(0.7, 0.0), "recovery", l2);
  add("A_yaw_negative", authored(-0.7, 0.0), "recovery", l2);
  add("A_tx_positive", authored(0.0, 0.04), "recovery", l2);
  add("A_tx_negative", authored(0.0, -0.04), "recovery", l2);
  add("A_joint_positive", authored(0.7, -0.04), "recovery", l2);
  add("A_joint_negative", authored(-0.6, 0.035), "recovery", l2);
  for (const std::uint64_t seed : {17U, 29U, 43U, 71U, 101U}) {
    auto config = authored(0.7, -0.04);
    config.seed = seed;
    config.noise_stddev_px = 1.0;
    add("B_seed_" + std::to_string(seed), config, "noise");
  }
  auto outliers = authored(0.7, -0.04);
  outliers.training_outlier_fraction = 0.10;
  add("C_l2", outliers, "paired", l2, "C_shared");
  add("C_huber", outliers, "paired", {}, "C_shared");
  add("D_rich", authored(0.7, -0.04), "recovery");
  auto narrow = authored(0.7, -0.04);
  narrow.geometry = io::AuthoredRecoveryGeometry::kNarrow;
  add("D_narrow", narrow, "diagnostic");
  auto yaw_bounds = geometry::ExtrinsicSearchConfig{};
  yaw_bounds.yaw.minimum = -0.5 * std::numbers::pi / 180.0;
  yaw_bounds.yaw.maximum = 0.5 * std::numbers::pi / 180.0;
  add("E_yaw_bound", authored(0.7, -0.04), "bound", yaw_bounds);
  auto tx_bounds = geometry::ExtrinsicSearchConfig{};
  tx_bounds.translation.minimum = -0.02;
  tx_bounds.translation.maximum = 0.02;
  add("E_tx_bound", authored(0.7, -0.04), "bound", tx_bounds);
  auto shifted = authored(0.7, -0.04);
  shifted.nominal_translation_m = {0.12, -0.04, 0.20};
  add("E_nominal_offset", shifted, "recovery");
  add("E_offgrid", authored(0.733, 0.0268), "offgrid");
  // Supplemental failure demonstration, not a retuning of any frozen gate.
  auto invisible = geometry::ExtrinsicSearchConfig{};
  invisible.yaw.minimum = 170.0 * std::numbers::pi / 180.0;
  invisible.yaw.maximum = 174.0 * std::numbers::pi / 180.0;
  add("E_no_valid_candidate", authored(0.7, -0.04), "invalid", invisible);
  return result;
}

[[nodiscard]] Json range(const std::vector<std::optional<double>>& samples) {
  std::vector<double> values;
  for (const auto& sample : samples) {
    if (sample.has_value()) {
      values.push_back(sample.value());
    }
  }
  if (values.empty()) {
    return {{"median", nullptr},
            {"minimum", nullptr},
            {"maximum", nullptr},
            {"samples", 0U},
            {"requested", samples.size()}};
  }
  std::ranges::sort(values);
  return {{"median", values[(values.size() - 1U) / 2U]},
          {"minimum", values.front()},
          {"maximum", values.back()},
          {"samples", values.size()},
          {"requested", samples.size()},
          {"conditional_on_available", values.size() != samples.size()}};
}
[[nodiscard]] std::optional<double> normalizedError(const geometry::RecoveryAcceptance& gate) {
  if (!gate.signed_yaw_error_deg.has_value() || !gate.signed_translation_error_m.has_value()) {
    return std::nullopt;
  }
  return std::hypot(gate.signed_yaw_error_deg.value() / 4.0,
                    gate.signed_translation_error_m.value() / 0.2);
}

[[nodiscard]] std::optional<double> recoveredRmse(const geometry::RecoveryComparison& comparison) {
  if (!comparison.recovered.has_value()) {
    return std::nullopt;
  }
  return comparison.recovered.value().pooled.rmse_px;
}
[[nodiscard]] std::optional<double> absolute(const std::optional<double>& value) {
  return value.has_value() ? std::optional<double>(std::abs(value.value())) : std::nullopt;
}
[[nodiscard]] Json nullable(const std::optional<double>& value) {
  return value.has_value() ? Json(value.value()) : Json(nullptr);
}

class CampaignStage final {
public:
  explicit CampaignStage(std::filesystem::path path) : path_(std::move(path)) {
    if (!std::filesystem::create_directory(path_)) {
      throw std::runtime_error("campaign stage already exists");
    }
  }
  CampaignStage(const CampaignStage&) = delete;
  CampaignStage& operator=(const CampaignStage&) = delete;
  ~CampaignStage() {
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
} // namespace

int main(const int argc, char** argv) {
  try {
    const cli::RecoveryArguments args(argc, argv, {"--output-dir", "--run-id"});
    if (args.help()) {
      std::cout << "run_extrinsic_campaign --output-dir PATH --run-id ID [--overwrite]\n"
                   "Frozen authored A-E campaign; retains missed noiseless gates and limitations.\n"
                   "Exit 0: required campaign gates passed; 4: retained required gate failures; 2: "
                   "syntax; 5: I/O.\n";
      return 0;
    }
    const std::filesystem::path parent = args.required("--output-dir");
    const auto run_id = args.value("--run-id", "campaign");
    if (run_id.empty() || !std::ranges::all_of(run_id, [](const unsigned char c) {
          return std::isalnum(c) != 0 || c == '-' || c == '_';
        })) {
      throw cli::ArgumentError("run ID must be alphanumeric with hyphen/underscore");
    }
    const auto destination = parent / run_id;
    const auto temporary = parent / (run_id + ".tmp");
    if (std::filesystem::exists(destination) && !std::filesystem::is_empty(destination)) {
      if (!args.overwrite()) {
        throw std::runtime_error("campaign exists; use --overwrite");
      }
      Json old;
      std::ifstream(destination / "campaign.json") >> old;
      if (old.value("schema_name", "") != "sfr.extrinsic_campaign") {
        throw std::runtime_error("refusing unrelated-directory overwrite");
      }
    }
    std::filesystem::create_directories(parent);
    CampaignStage stage(temporary);
    Json rows = Json::array();
    Json build, host;
    bool required_pass = true;
    std::vector<std::optional<double>> byaw, btx, byaw_abs, btx_abs, brmse;
    std::optional<double> l2_error, huber_error, l2_rmse, huber_rmse;
    std::string c_identity;
    for (const auto& experiment : cases()) {
      const auto dataset = temporary / "fixtures" / experiment.dataset_id;
      if (!std::filesystem::exists(dataset)) {
        io::generateExtrinsicFixture(dataset, experiment.fixture);
      }
      const auto run = cli::composeRecoveryRun(dataset, {0U, 1U}, {2U}, experiment.search,
                                               temporary / "runs", experiment.id);
      const auto path = viz::writeExtrinsicRecoveryArtifacts(run);
      const auto gate = geometry::assessExtrinsicRecovery(run.search, run.config, run.hidden,
                                                          run.validation_clean);
      Json report;
      std::ifstream(path / "report.json") >> report;
      build = report.at("build");
      host = report.at("host");
      bool case_gate = run.deterministic_replay_equal;
      if (experiment.expected == "recovery") {
        case_gate = case_gate && gate.passed;
      }
      if (experiment.expected == "bound") {
        case_gate = case_gate &&
                    run.search.status == geometry::RecoverySearchStatus::kBoundLimited &&
                    !gate.passed;
      }
      if (experiment.expected == "invalid") {
        case_gate = case_gate &&
                    run.search.status == geometry::RecoverySearchStatus::kNoValidCandidate &&
                    !gate.passed;
      }
      if (experiment.expected == "diagnostic") {
        case_gate = case_gate && !gate.passed && run.search.identifiability.available;
      }
      required_pass = required_pass && case_gate;
      rows.push_back({{"case_id", experiment.id},
                      {"expectation", experiment.expected},
                      {"required_case_gate", case_gate},
                      {"noiseless_gate", report.at("acceptance")},
                      {"parameter_errors", report.at("parameter_errors")},
                      {"search_status", report.at("search").at("result_status")},
                      {"identifiability", report.at("search").at("identifiability")},
                      {"held_out_clean", report.at("metrics").at("validation").at("clean")},
                      {"training_observed", report.at("metrics").at("train").at("observed")},
                      {"search_ms", report.at("timing").at("search_ms")},
                      {"evaluations", run.search.candidates.size()},
                      {"points_evaluated", run.search.points_evaluated},
                      {"fixture_identity", run.fixture.content_identity},
                      {"report", "runs/" + experiment.id + "/report.json"}});
      if (experiment.expected == "noise") {
        byaw.push_back(gate.signed_yaw_error_deg);
        btx.push_back(gate.signed_translation_error_m);
        byaw_abs.push_back(absolute(gate.signed_yaw_error_deg));
        btx_abs.push_back(absolute(gate.signed_translation_error_m));
        brmse.push_back(recoveredRmse(run.validation_clean));
      }
      if (experiment.id == "C_l2") {
        l2_error = normalizedError(gate);
        l2_rmse = recoveredRmse(run.validation_clean);
        c_identity = run.fixture.content_identity;
      }
      if (experiment.id == "C_huber") {
        huber_error = normalizedError(gate);
        huber_rmse = recoveredRmse(run.validation_clean);
        required_pass = required_pass && run.fixture.content_identity == c_identity;
      }
      std::cout << experiment.id << ": strict noiseless gate=" << (gate.passed ? "pass" : "miss")
                << ", required case gate=" << (case_gate ? "pass" : "FAIL") << '\n';
    }
    const bool robust = huber_error.has_value() && l2_error.has_value() && huber_rmse.has_value() &&
                        l2_rmse.has_value() && huber_error.value() < l2_error.value() &&
                        huber_rmse.value() < l2_rmse.value();
    required_pass = required_pass && robust;
    Json summary{
        {"schema_name", "sfr.extrinsic_campaign"},
        {"schema_version", "1.0.0"},
        {"status", "complete"},
        {"build", build},
        {"host", host},
        {"required_gates_passed", required_pass},
        {"claim_boundary", "authored synthetic two-parameter known-correspondence recovery only"},
        {"cases", rows},
        {"B_noise",
         {{"all_fixed_seeds_retained", true},
          {"seeds", Json::array({17, 29, 43, 71, 101})},
          {"signed_yaw_error_deg", range(byaw)},
          {"absolute_yaw_error_deg", range(byaw_abs)},
          {"signed_translation_error_m", range(btx)},
          {"absolute_translation_error_m", range(btx_abs)},
          {"held_out_clean_rmse_px", range(brmse)}}},
        {"C_outliers",
         {{"same_fixture_identity", c_identity},
          {"l2_normalized_parameter_error", nullable(l2_error)},
          {"huber_normalized_parameter_error", nullable(huber_error)},
          {"l2_held_out_clean_rmse_px", nullable(l2_rmse)},
          {"huber_held_out_clean_rmse_px", nullable(huber_rmse)},
          {"required_robustness_gate", robust}}},
        {"limitations", "D/E/offgrid/noisy strict-gate misses are retained; no universal grid "
                        "accuracy or physical calibration claim"},
        {"publication",
         "exclusive stage; no fsync; explicit overwrite removes old destination before rename"}};
    std::ofstream stream(temporary / "campaign.json");
    stream << summary.dump(2) << '\n';
    stream.close();
    if (!stream) {
      throw std::runtime_error("campaign summary write failed");
    }
    if (std::filesystem::exists(destination)) {
      std::filesystem::remove_all(destination);
    }
    std::filesystem::rename(temporary, destination);
    stage.commit();
    std::cout << "published campaign: " << destination / "campaign.json" << '\n';
    return required_pass ? 0 : 4;
  } catch (const cli::ArgumentError& error) {
    std::cerr << "configuration: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "campaign: " << error.what() << '\n';
    return 5;
  }
}
