#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include <opencv2/imgcodecs.hpp>

#include "catch2/catch.hpp"
#include "common.h"
#include "config/pipeline_config.h"
#include "hub_pipeline.h"
#include "hub_target_data.h"

using namespace rr;
using namespace deadeye;
using hsv_t = std::array<int, 2>;
using filter_t = std::array<double, 2>;

struct area_compare {
  bool operator()(const std::array<int, 5>& a,
                  const std::array<int, 5>& b) const {
    return a[4] > b[4];  // contour area is index 4
  }
} area_comparator;

struct x_compare {
  bool operator()(const std::array<int, 5>& a,
                  const std::array<int, 5>& b) const {
    return a[0] < b[0];  // bounding box x is index 0
  }
} x_comparator;

void print_target_areas(const TargetList& targets) {
  for (int i = 0; i < targets.size(); ++i) {
    WARN("target " << i << " area = " << targets[i][4]);
  }
}

void print_target_xy(const TargetList& targets) {
  for (int i = 0; i < targets.size(); ++i) {
    WARN("target " << i << " bb x = " << targets[i][0]
                   << ", y = " << targets[i][1]);
  }
}

TEST_CASE("HubPipeline processes test image 1-1.jpg") {
  hsv_t hue{0, 105};
  hsv_t sat{157, 255};
  hsv_t val{153, 255};

  filter_t area{0, 1.0};
  filter_t solidity{0, 1.0};
  filter_t aspect{0, 20.0};

  FilterConfig filter_config{area, solidity, aspect};
  LogConfig log_config;

  json j;

  PipelineConfig pipeline_config{0, hue, sat, val, filter_config, log_config};

  std::unique_ptr<Pipeline> hpl = std::make_unique<HubPipeline>(2767, "test");

  cv::Mat frame = cv::imread(DEADEYE_TEST_DATA "1-1.jpg");
  REQUIRE(frame.cols == 1280);
  REQUIRE(frame.rows == 720);

  SECTION("with no filters & maxTargets = 99") {
    j["maxTargets"] = 99;
    pipeline_config.config = j;
    hpl->Configure(pipeline_config);

    auto td = hpl->ProcessFrame(frame);
    auto htd = dynamic_cast<HubTargetData*>(td.release());
    REQUIRE(htd->valid);
    REQUIRE(htd->targets.size() == 31);

    REQUIRE(htd->Dump().size() == 552);

    //    print_target_areas(htd->targets);

    delete htd;
  }

  SECTION("with no filters & maxTargets = 5") {
    j["maxTargets"] = 5;
    pipeline_config.config = j;
    hpl->Configure(pipeline_config);

    auto td = hpl->ProcessFrame(frame);
    auto htd = dynamic_cast<HubTargetData*>(td.release());
    REQUIRE(htd->valid);
    REQUIRE(htd->targets.size() == j["maxTargets"]);

    REQUIRE(htd->Dump().size() == 136);

    //    print_target_areas(htd->targets);

    delete htd;
  }
}

TEST_CASE("targets are sorted by upper-left x-coord") {
  hsv_t hue{0, 105};
  hsv_t sat{157, 255};
  hsv_t val{153, 255};

  filter_t area{0, 1.0};
  filter_t solidity{0, 1.0};
  filter_t aspect{0, 20.0};

  FilterConfig filter_config{area, solidity, aspect};
  LogConfig log_config;

  PipelineConfig pipeline_config{0, hue, sat, val, filter_config, log_config};

  json j;
  j["maxTargets"] = 5;
  pipeline_config.config = j;

  std::unique_ptr<Pipeline> hpl = std::make_unique<HubPipeline>(2767, "test");

  cv::Mat frame = cv::imread(DEADEYE_TEST_DATA "1-1.jpg");
  REQUIRE(frame.cols == 1280);
  REQUIRE(frame.rows == 720);

  hpl->Configure(pipeline_config);

  auto td = hpl->ProcessFrame(frame);
  auto htd = dynamic_cast<HubTargetData*>(td.release());
  REQUIRE(htd->valid);
  REQUIRE(htd->targets.size() == j["maxTargets"]);

  REQUIRE(
      std::is_sorted(htd->targets.begin(), htd->targets.end(), x_comparator));

  //      print_target_xy(htd->targets);

  delete htd;
}

TEST_CASE("HubTargetData doesn't exceed maximum payload size") {
  hsv_t hue{0, 255};
  hsv_t sat{1, 255};
  hsv_t val{0, 255};

  filter_t area{0, 1.0};
  filter_t solidity{0, 1.0};
  filter_t aspect{0, 20.0};

  FilterConfig filter_config{area, solidity, aspect};
  LogConfig log_config;

  json j;

  PipelineConfig pipeline_config{0, hue, sat, val, filter_config, log_config};

  std::unique_ptr<Pipeline> hpl = std::make_unique<HubPipeline>(2767, "test");

  cv::Mat frame = cv::imread(DEADEYE_TEST_DATA "many_targets.jpg");
  REQUIRE(frame.cols == 1280);
  REQUIRE(frame.rows == 720);

  SECTION("with max targets") {
    j["maxTargets"] = 45;
    pipeline_config.config = j;
    hpl->Configure(pipeline_config);

    auto td = hpl->ProcessFrame(frame);
    auto htd = dynamic_cast<HubTargetData*>(td.release());
    REQUIRE(htd->valid);
    REQUIRE(htd->targets.size() == j["maxTargets"]);

    REQUIRE(htd->Dump().size() < TD_MAX_SIZE);
    delete htd;
  }

  SECTION("with max targets exceeded") {
    j["maxTargets"] = 99;
    pipeline_config.config = j;
    hpl->Configure(pipeline_config);

    auto td = hpl->ProcessFrame(frame);
    auto htd = dynamic_cast<HubTargetData*>(td.release());
    REQUIRE(htd->valid);
    REQUIRE(htd->targets.size() == 45);  //  kMaxTargetsAllowed = 45

    REQUIRE(htd->Dump().size() < TD_MAX_SIZE);
    delete htd;
  }
}

#pragma clang diagnostic pop