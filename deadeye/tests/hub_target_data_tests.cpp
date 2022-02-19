#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include "catch2/catch.hpp"
#include "hub_target_data.h"
#include "common.h"

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
  HubTargetData htd{"Z0",1,true, GetTargetList(1)};

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
  HubTargetData htd{"Z0",1,true, GetTargetList(43)};
  std::string htd_json = htd.Dump();
  INFO(htd_json);
  REQUIRE(htd_json.size() < TD_MAX_SIZE);
}

#pragma clang diagnostic pop