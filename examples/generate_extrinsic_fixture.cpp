#include <iostream>
#include <numbers>

#include "recovery_arguments.hpp"
#include "sfr/io/extrinsic_fixture.hpp"
#include "sfr/io/io_error.hpp"

int main(const int argc, char** argv) {
  try {
    const sfr::cli::RecoveryArguments arguments(
        argc, argv,
        {"--output-dir", "--hidden-yaw-deg", "--hidden-tx-m", "--seed", "--noise-stddev-px",
         "--outlier-fraction", "--geometry", "--nominal-tx-m", "--nominal-ty-m", "--nominal-tz-m"});
    if (arguments.help()) {
      std::cout
          << "Usage: generate_extrinsic_fixture --output-dir <authored-root> [options]\n"
             "Authored synthetic 3D-to-2D correspondences, never KITTI calibration evidence.\n"
             "Defaults: hidden-yaw-deg=0, hidden-tx-m=0, seed=17, noise-stddev-px=0,\n"
             "outlier-fraction=0, geometry=rich, nominal-tx/ty/tz-m=0.\n"
             "Options: --hidden-yaw-deg --hidden-tx-m --seed --noise-stddev-px\n"
             "--outlier-fraction --geometry rich|narrow --nominal-tx-m --nominal-ty-m\n"
             "--nominal-tz-m --overwrite --help\n";
      return 0;
    }
    sfr::io::ExtrinsicFixtureConfig config;
    config.hidden = {arguments.number("--hidden-yaw-deg", 0.0) * std::numbers::pi / 180.0,
                     arguments.number("--hidden-tx-m", 0.0)};
    config.seed = arguments.unsignedValue("--seed", 17U);
    config.noise_stddev_px = arguments.number("--noise-stddev-px", 0.0);
    config.training_outlier_fraction = arguments.number("--outlier-fraction", 0.0);
    config.nominal_translation_m = {arguments.number("--nominal-tx-m", 0.0),
                                    arguments.number("--nominal-ty-m", 0.0),
                                    arguments.number("--nominal-tz-m", 0.0)};
    const auto geometry = arguments.value("--geometry", "rich");
    if (geometry != "rich" && geometry != "narrow") {
      throw sfr::cli::ArgumentError("geometry must be rich or narrow");
    }
    config.geometry = geometry == "rich" ? sfr::io::AuthoredRecoveryGeometry::kRich
                                         : sfr::io::AuthoredRecoveryGeometry::kNarrow;
    const auto output = arguments.required("--output-dir");
    sfr::io::generateExtrinsicFixture(output, config, arguments.overwrite());
    std::cout << "created project-authored extrinsic fixture: " << output << '\n';
    return 0;
  } catch (const sfr::cli::ArgumentError& error) {
    std::cerr << "argument error: " << error.what() << '\n';
    return 2;
  } catch (const sfr::io::IoError& error) {
    std::cerr << "fixture error: " << error.what() << '\n';
    return error.code() == sfr::io::IoErrorCode::kInvalidCalibration ? 2 : 5;
  } catch (const std::exception& error) {
    std::cerr << "output error: " << error.what() << '\n';
    return 5;
  }
}
