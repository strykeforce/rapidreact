#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <opencv2/imgcodecs.hpp>

#include "catch2/catch.hpp"
#include "config/pipeline_config.h"
#include "hub_pipeline.h"
#include "hub_target_data.h"

using namespace rr;
using namespace deadeye;
using hsv_t = std::array<int, 2>;
using filter_t = std::array<double, 2>;


TEST_CASE("HubPipeline processes test image 1-1.jpg") {
  hsv_t hue{0, 105};
  hsv_t sat{157, 255};
  hsv_t val{153, 255};

  filter_t area{0, 1.0};
  filter_t solidity{0, 1.0};
  filter_t aspect{0, 20.0};

  FilterConfig filter_config{area, solidity, aspect};
  LogConfig log_config;

  // CaptureConfig capture_config{CaptureType::file, 1280, 720, 30, {}};
  PipelineConfig pipeline_config{0, hue, sat, val, filter_config, log_config};

  std::unique_ptr<Pipeline> hpl = std::make_unique<HubPipeline>(2767, "test");

  cv::Mat frame = cv::imread(DEADEYE_TEST_DATA "1-1.jpg");
  REQUIRE(frame.cols == 1280);
  REQUIRE(frame.rows == 720);

  SECTION("with no filters") {
    hpl->Configure(pipeline_config);
    auto td = hpl->ProcessFrame(frame);
    auto htd = dynamic_cast<HubTargetData*>(td.release());
    REQUIRE(htd->valid);
    REQUIRE(htd->targets.size() == 31);
    delete htd;
  }
}

#pragma clang diagnostic pop