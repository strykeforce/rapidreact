#pragma once
#include <nlohmann/json.hpp>
#include <opencv2/core/types.hpp>

#include "link/target_data.h"

using json = nlohmann::json;

namespace rr {
using TargetList = std::vector<std::array<int, 5>>;

struct HubTargetData : public deadeye::TargetData {
  static const char* kErrorPixelsKey;
  static const char* kRangeKey;

  TargetList targets;
  double error_pixels;
  double range;
  int frame_x_center_;

  HubTargetData(std::string_view id, int sn, bool valid, double error_pixels,
                double range, TargetList targets, int center);

  [[nodiscard]] double GetErrorPixels() const;
  void DrawMarkers(cv::Mat& preview) const override;
  [[nodiscard]] std::string Dump() const override;
  [[nodiscard]] std::string ToString() const override;
};
}  // namespace rr
