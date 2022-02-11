#pragma once
#include <nlohmann/json.hpp>
#include <opencv2/core/types.hpp>

#include "link/target_data.h"

using json = nlohmann::json;

namespace rr {
using TargetList = std::vector<std::array<int, 5>>;
struct HubTargetData : public deadeye::TargetData {
  TargetList targets;

  HubTargetData(std::string id, int sn, bool valid, TargetList targets);

  void DrawMarkers(cv::Mat& preview) const override;
  [[nodiscard]] std::string Dump() const override;
  [[nodiscard]] std::string ToString() const override;
};
}  // namespace rr
