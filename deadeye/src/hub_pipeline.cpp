#include "hub_pipeline.h"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>
#include <utility>

#include "hub_target_data.h"

using namespace deadeye;
using namespace rr;

namespace {
constexpr int kMaxTargetsDefault = 10;
}

HubPipeline::HubPipeline(int inum, std::string name)
    : AbstractPipeline{inum, std::move(name)},
      max_targets_{kMaxTargetsDefault} {}

void HubPipeline::Configure(const PipelineConfig& config) {
  AbstractPipeline::Configure(config);
  json j = config.config;
  max_targets_ = j.value("maxTargets", kMaxTargetsDefault);
  spdlog::debug("{}: max targets = {}", *this, max_targets_);
}

// Target is center of contour bounding box.
std::unique_ptr<TargetData> HubPipeline::ProcessContours(
    Contours const& contours) {
  TargetList targets;
  int target_count = 0;

  for (const auto& contour : contours) {
    if (++target_count > max_targets_) break;
    cv::Rect bb = cv::boundingRect(contour);
    int area = static_cast<int>(std::round(cv::contourArea(contour)));
    targets.push_back({bb.x, bb.y, bb.width, bb.height, area});
  }

  return std::make_unique<HubTargetData>(id_, 0, !contours.empty(), targets);
}

std::string HubPipeline::ToString() const {
  return fmt::format("HubPipeline<{}, {}>", id_, capture_type_);
}
