#include "hub_pipeline.h"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>
#include <utility>

#include "hub_target_data.h"

using namespace deadeye;
using namespace rr;

[[maybe_unused]] HubPipeline::HubPipeline(int inum, std::string name)
    : AbstractPipeline{inum, std::move(name)} {}

void HubPipeline::Configure(const CaptureConfig& config) {
  capture_type_ = config.PipelineType();
}

// Target is center of contour bounding box.
std::unique_ptr<TargetData> HubPipeline::ProcessContours(
    Contours const& contours) {
  TargetList targets;

  for (const auto& contour : contours) {
    cv::Rect bb = cv::boundingRect(contour);
    int area = static_cast<int>(std::round(cv::contourArea(contour)));
    targets.push_back({bb.x, bb.y, bb.width, bb.height, area});
  }

  return std::make_unique<HubTargetData>(id_, 0, !contours.empty(), targets);
}

std::string HubPipeline::ToString() const {
  return fmt::format("HubPipeline<{}, {}>", id_, capture_type_);
}
