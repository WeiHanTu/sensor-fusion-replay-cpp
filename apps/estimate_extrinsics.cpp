#include <chrono>
#include <iostream>
#include <numbers>
#include <set>

#include "recovery_workflow.hpp"
#include "sfr/io/io_error.hpp"

int main(const int argc, char** argv) {
  try {
    const sfr::cli::RecoveryArguments args(
        argc, argv,
        {"--dataset-root",   "--train-frames",    "--validation-frames", "--parameters",
         "--loss",           "--huber-delta-px",  "--yaw-min-deg",       "--yaw-max-deg",
         "--yaw-coarse-deg", "--yaw-fine-deg",    "--tx-min-m",          "--tx-max-m",
         "--tx-coarse-m",    "--tx-fine-m",       "--max-candidates",    "--max-point-projections",
         "--rank-tolerance", "--condition-limit", "--output-dir",        "--run-id"});
    if (args.help()) {
      std::cout << "estimate_extrinsics --dataset-root PATH --output-dir PATH\n"
                   "  --train-frames 0,1 --validation-frames 2\n"
                   "  --parameters yaw_camera_y,translation_camera_x --loss huber|l2\n"
                   "  --huber-delta-px 2 --yaw-min-deg -2 --yaw-max-deg 2\n"
                   "  --yaw-coarse-deg 0.1 --yaw-fine-deg 0.01\n"
                   "  --tx-min-m -0.1 --tx-max-m 0.1 --tx-coarse-m 0.005 --tx-fine-m 0.0005\n"
                   "  --max-candidates 100000 --max-point-projections 100000000\n"
                   "  --rank-tolerance 1e-6 --condition-limit 10000 --run-id ID --overwrite\n"
                   "Exit: 0 completed valid grid (acceptance in report), 2 config, 3 input,\n"
                   "      4 search/identifiability limitation after report, 5 output.\n"
                   "Authored synthetic correspondences only; not physical/KITTI calibration.\n";
      return 0;
    }
    if (args.value("--parameters", "yaw_camera_y,translation_camera_x") !=
        "yaw_camera_y,translation_camera_x") {
      throw sfr::cli::ArgumentError(
          "V1 supports only yaw_camera_y,translation_camera_x in that order");
    }
    sfr::geometry::ExtrinsicSearchConfig config;
    const double radians = std::numbers::pi / 180.0;
    config.yaw = {args.number("--yaw-min-deg", -2.0) * radians,
                  args.number("--yaw-max-deg", 2.0) * radians,
                  args.number("--yaw-coarse-deg", 0.1) * radians,
                  args.number("--yaw-fine-deg", 0.01) * radians};
    config.translation = {args.number("--tx-min-m", -0.1), args.number("--tx-max-m", 0.1),
                          args.number("--tx-coarse-m", 0.005), args.number("--tx-fine-m", 0.0005)};
    const auto loss = args.value("--loss", "huber");
    if (loss != "huber" && loss != "l2") {
      throw sfr::cli::ArgumentError("loss must be huber or l2");
    }
    config.loss = {loss == "huber" ? sfr::geometry::RecoveryLoss::kHuber
                                   : sfr::geometry::RecoveryLoss::kSquaredL2,
                   args.number("--huber-delta-px", 2.0)};
    config.max_candidates = args.unsignedValue("--max-candidates", 100000U);
    config.max_point_projections = args.unsignedValue("--max-point-projections", 100000000U);
    config.rank_relative_tolerance = args.number("--rank-tolerance", 1e-6);
    config.condition_limit = args.number("--condition-limit", 1e4);
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    const auto run = sfr::cli::composeRecoveryRun(
        args.required("--dataset-root"), args.ids("--train-frames", "0,1"),
        args.ids("--validation-frames", "2"), config, args.required("--output-dir"),
        args.value("--run-id", "synthetic_recovery_" + std::to_string(stamp)), args.overwrite());
    const auto path = sfr::viz::writeExtrinsicRecoveryArtifacts(run);
    std::cout << "published synthetic recovery report: " << path / "report.json" << '\n';
    return sfr::cli::recoveryExitCode(run.search);
  } catch (const sfr::cli::ArgumentError& error) {
    std::cerr << "configuration: " << error.what() << '\n';
    return 2;
  } catch (const sfr::io::IoError& error) {
    std::cerr << "input: " << error.what() << '\n';
    return 3;
  } catch (const sfr::geometry::GeometryError& error) {
    std::cerr << "geometry: " << error.what() << '\n';
    return 4;
  } catch (const std::exception& error) {
    std::cerr << "output: " << error.what() << '\n';
    return 5;
  }
}
