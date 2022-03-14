#include "hub_target_data.h"

#include <fmt/core.h>

#include <opencv2/imgproc.hpp>
#include <utility>

#define X 0
#define Y 1
#define W 2
#define H 3

using namespace deadeye;
using namespace rr;
using json = nlohmann::json;

const char* HubTargetData::kErrorPixelsKey{"ep"};
const char* HubTargetData::kRangeKey{"r"};

namespace {
// minimum datagram size: IPv4 = 576 IPv6 = 1280
const cv::Scalar BB_COLOR{20, 255, 20};            // NOLINT
const cv::Scalar CROSS_HAIR_COLOR{200, 200, 200};  // NOLINT
const cv::Scalar MARKER_COLOR{255, 255, 255};      // NOLINT
}  // namespace

HubTargetData::HubTargetData(std::string_view id, int sn, bool valid,
                             double error_pixels, double range,
                             TargetList targets, int center)
    : TargetData{id, sn, valid},
      error_pixels{error_pixels},
      range{range},
      targets{std::move(targets)},
      frame_x_center_{center} {}

double HubTargetData::GetErrorPixels() const {
  if (targets.empty()) return 0.0;
  auto lt = targets.front();
  auto rt = targets.back();
  return (lt[X] + lt[W] + rt[X]) / 2.0 - frame_x_center_;
}

void HubTargetData::DrawMarkers(cv::Mat& preview) const {
  // draw line down center of frame
  int center = preview.cols / 2;
  cv::line(preview, cv::Point{center, 0}, cv::Point{center, preview.rows},
           CROSS_HAIR_COLOR);

  if (targets.empty()) return;

  // draw target bounding boxes
  for (const auto& t : targets) {
    cv::Rect bb{t[0], t[1], t[2], t[3]};
    cv::rectangle(preview, bb, BB_COLOR, 2);
  }

  // draw center aim point of targets
  cv::Point targets_center{static_cast<int>(GetErrorPixels()) + frame_x_center_,
                           targets[0][Y] + targets[0][H] / 2};
  cv::drawMarker(preview, targets_center, MARKER_COLOR);
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-no-recursion"
std::string HubTargetData::Dump() const {
  json j = json{
      {TargetData::kIdKey, id},
      {TargetData::kSerialKey, serial},
      {TargetData::kValidKey, valid},
      {TargetData::kDataKey, targets},
      {HubTargetData::kErrorPixelsKey, error_pixels},
      {HubTargetData::kRangeKey, range},
  };

  return j.dump();
}
#pragma clang diagnostic pop

std::string HubTargetData::ToString() const {
  return fmt::format("id={} sn={} val={})", id, serial, valid);
}
