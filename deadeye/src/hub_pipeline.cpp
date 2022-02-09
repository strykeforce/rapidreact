#include "hub_pipeline.h"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>

#include "hub_target_data.h"

using namespace deadeye;
using namespace rr;

HubPipeline::HubPipeline(int inum, std::string name)
    : AbstractPipeline{inum, name} {}

void HubPipeline::Configure(const CaptureConfig& config) {
  capture_type_ = config.PipelineType();
}

// Target is center of contour bounding box.
std::unique_ptr<TargetData> HubPipeline::ProcessContours(
    Contours const& contours) {
  if (contours.empty())
    return std::make_unique<HubTargetData>(id_, 0, false,
                                           cv::Rect{0, 0, 0, 0});
  auto contour = contours[0];
  cv::Rect bb = cv::boundingRect(contour);
  return std::make_unique<HubTargetData>(id_, 0, true, bb);
}

std::string HubPipeline::ToString() const {
  return fmt::format("HubPipeline<{}, {}>", id_, capture_type_);
}
