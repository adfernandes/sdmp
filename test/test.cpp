#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <vector>

#include "sdmp.hpp"

using namespace std;
using namespace sdmp;

namespace {

  MotionPlanPtr test_create() {
    auto motion_plan = bb8::simple::create(0.125, 4.0, 3.0);
    return motion_plan;
  }

}

// TODO These tests are not really exhaustive (Yet!)
//
// We need to test not just successes, but expected failures,
// such as if have invalid values within `MotionPlan` objects

TEST_CASE("create", "[sdmp::bb8::simple::create]") {

  auto motion_plan = test_create();

  REQUIRE(motion_plan.get() != nullptr);
  REQUIRE(motion_plan->has_bb8());
  REQUIRE(motion_plan->has_rectangle());
  REQUIRE(motion_plan->obstacle_size() == 0);
  REQUIRE(motion_plan->path_size() == 0);
  REQUIRE(bb8::simple::is_valid(*motion_plan));

}

TEST_CASE("add_circular_obstacle", "[sdmp::add_circular_obstacle]") {

  auto motion_plan = test_create();
  REQUIRE(motion_plan.get() != nullptr);
  REQUIRE(bb8::simple::is_valid(*motion_plan));

  for (int i = 3; i <= 9; i++) {
    const bool succeeded = add_circular_obstacle(*motion_plan, double(i), double(i), 1.5);
    REQUIRE(succeeded);
  }

  REQUIRE(bb8::simple::is_valid(*motion_plan));

}

TEST_CASE("find_path", "[sdmp::bb8::simple::find_path]") {

  auto motion_plan = test_create();
  REQUIRE(motion_plan.get() != nullptr);
  REQUIRE(bb8::simple::is_valid(*motion_plan));

  assert(motion_plan->rectangle().length() == int(motion_plan->rectangle().length()));
  assert(motion_plan->rectangle().width() == int(motion_plan->rectangle().width()));

  for (int i = 0; i <= motion_plan->rectangle().length(); i++) {
    for (int j = 0; j <= motion_plan->rectangle().width(); j++) {
      if (i == 0 && j == 0) continue;
      if (i == motion_plan->rectangle().length() && j == motion_plan->rectangle().width()) continue;
      const bool succeeded = add_circular_obstacle(*motion_plan, double(i), double(j), 0.25);
      REQUIRE(succeeded);
    }
  }

  REQUIRE(bb8::simple::is_valid(*motion_plan));

  const bool failed = sdmp::bb8::simple::find_path(*motion_plan,
      0.25, 0.25,
      motion_plan->rectangle().length() - 0.25, motion_plan->rectangle().width() - 0.25,
      3.0, 0.0);

  REQUIRE_FALSE(failed);

  // DEBUG cout << sdmp::save_json(*motion_plan) << endl;
  // DEBUG cout << sdmp::bb8::simple::save_gnuplot(*motion_plan);

}

// TODO Test 'save_json', 'load_json', and 'save_gnuplot' (doing so is more involved)
