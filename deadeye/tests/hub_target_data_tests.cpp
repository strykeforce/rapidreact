#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include "catch2/catch.hpp"
#include "common.h"
#include "hub_target_data.h"

using namespace rr;
using namespace deadeye;

TargetList GetTargetList(int count) {
  TargetList target_list;
  for (int i = 0; i < count; ++i) {
    // bb.x, bb.y. bb.w, bb.h, contour area
    target_list.push_back({4444, 333, 333, 22, 333});
  }
  return std::move(target_list);
}

TEST_CASE("target data JSON") {
  HubTargetData htd{"Z0", 1, true, 0.0, 0.0, GetTargetList(1), 640};

  json actual = json::parse(htd.Dump());

  json expected = R"(
{
  "id": "Z0",
  "sn": 1,
  "v": true,
  "d": [
    [ 4444, 333, 333, 22, 333 ]
  ],
  "ep": 0.0,
  "r": 0.0
}
)"_json;

  REQUIRE(actual == expected);
}

TEST_CASE("maximum sized target data") {
  HubTargetData htd{"Z0", 1, true, 0.0, 0.0, GetTargetList(43), 640};
  std::string htd_json = htd.Dump();
  INFO(htd_json);
  REQUIRE(htd_json.size() < TD_MAX_SIZE);
}

TEST_CASE("target data DrawMarkers works with no targets") {
  TargetList empty_target_list;
  HubTargetData htd{"Z0", 1, true, 0.0, 0.0, empty_target_list, 640};
  cv::Mat preview{720, 1280};
  htd.DrawMarkers(preview);
}

#pragma clang diagnostic pop