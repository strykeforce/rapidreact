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
constexpr int kMaxTargetsAllowed = 45;
}  // namespace

HubPipeline::HubPipeline(int inum, std::string name)
    : AbstractPipeline{inum, std::move(name)},
      max_targets_{kMaxTargetsDefault},
      frame_x_center_{0} {}

void HubPipeline::Configure(const CaptureConfig& config) {
  AbstractPipeline::Configure(config);
  frame_x_center_ = config.width / 2;
}

void HubPipeline::Configure(const PipelineConfig& config) {
  AbstractPipeline::Configure(config);
  json j = config.config;
  max_targets_ = j.value("maxTargets", kMaxTargetsDefault);
  if (max_targets_ > kMaxTargetsAllowed) {
    spdlog::warn("maxTargets = {}, must be less than {}", max_targets_,
                 kMaxTargetsAllowed);
    max_targets_ = kMaxTargetsAllowed;
  }
  spdlog::debug("{}: max targets = {}", *this, max_targets_);
}

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

  // sort targets by bounding box x-coordinate, left to right
  std::sort(targets.begin(), targets.end(),
            [](const auto& a, const auto& b) { return a[0] < b[0]; });

  return std::make_unique<HubTargetData>(id_, 0, !targets.empty(), 0.0, 0.0,
                                         targets, frame_x_center_);
}

std::string HubPipeline::ToString() const {
  return fmt::format("HubPipeline<{}>", id_);
}
