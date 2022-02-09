#pragma once
#include <nlohmann/json.hpp>
#include <opencv2/core/types.hpp>

#include "link/target_data.h"

using json = nlohmann::json;

namespace rr {
struct HubTargetData : public deadeye::TargetData {
  cv::Rect bb;
  cv::Point center;
  HubTargetData(std::string id, int sn, bool valid, cv::Rect bb);

  void DrawMarkers(cv::Mat& preview) const override;
  [[nodiscard]] std::string Dump() const override;
  [[nodiscard]] std::string ToString() const override;
};
}  // namespace rr
