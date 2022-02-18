#include "hub_target_data.h"

#include <fmt/core.h>
#include <spdlog/spdlog.h>

#include <opencv2/imgproc.hpp>
#include <utility>

using namespace deadeye;
using namespace rr;
using json = nlohmann::json;

namespace {
// minimum datagram size: IPv4 = 576 IPv6 = 1280
const cv::Scalar BB_COLOR{20, 255, 20};            // NOLINT
const cv::Scalar CROSS_HAIR_COLOR{200, 200, 200};  // NOLINT
}  // namespace

HubTargetData::HubTargetData(std::string id, int sn, bool valid,
                             TargetList targets)
    : TargetData{std::move(id), sn, valid}, targets{std::move(targets)} {}

void HubTargetData::DrawMarkers(cv::Mat& preview) const {
  for (const auto& t : targets) {
    cv::Rect bb{t[0], t[1], t[2], t[3]};
    cv::rectangle(preview, bb, BB_COLOR, 2);
  }
  int center = preview.cols / 2;
  cv::line(preview, cv::Point{center, 0}, cv::Point{center, preview.rows},
           CROSS_HAIR_COLOR);
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "misc-no-recursion"
std::string HubTargetData::Dump() const {
  json j = json{{TargetData::kIdKey, id},
                {TargetData::kSerialKey, serial},
                {TargetData::kValidKey, valid},
                {TargetData::kDataKey, targets}};

  return j.dump();
}
#pragma clang diagnostic pop

std::string HubTargetData::ToString() const {
  return fmt::format("id={} sn={} val={})", id, serial, valid);
}
