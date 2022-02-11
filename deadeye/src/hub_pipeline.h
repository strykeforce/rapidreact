#pragma once
#include "pipeline/abstract_pipeline.h"

namespace rr {

class HubPipeline : public deadeye::AbstractPipeline {
 public:
  [[maybe_unused]] HubPipeline(int inum, std::string name);

  void Configure(const deadeye::CaptureConfig& config) override;

  std::unique_ptr<deadeye::TargetData> ProcessContours(
      deadeye::Contours const& contours) final;

 protected:
  [[nodiscard]] std::string ToString() const final;

 private:
  std::string capture_type_{"unknown"};
};

}  // namespace rr
