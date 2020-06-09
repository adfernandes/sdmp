/*! \file
 * The Simple Droid Motion Planner (SDMP) Command Line Interface
 *
 * TODO Write me, with examples
 *
 */

#include <iostream>
#include <fstream>
#include <sstream>

#include "sdmp.hpp"

using namespace std;
using namespace sdmp;

// TODO Use a proper argument parsing library, such as "boost::program_options"

namespace {

  MotionPlanPtr motion_plan_from_std_cin() {
    stringstream buffer;
    while(cin >> buffer.rdbuf());
    return load_json(buffer.str());
  }

}

int main(int argc, char *argv[]) {

  if (argc == 2 && string(argv[1]) == "gnuplot" ) {

    auto motion_plan = motion_plan_from_std_cin();
    if (motion_plan == nullptr) return -1;

    cout << bb8::simple::save_gnuplot(*motion_plan);
    return 0;

  }

  if (argc == 8 && string(argv[1]) == "find_path") {

    auto motion_plan = motion_plan_from_std_cin();
    if (motion_plan == nullptr) return -1;

    const bool failed = bb8::simple::find_path(*motion_plan,
      stod(argv[2]), stod(argv[3]),
      stod(argv[4]), stod(argv[5]),
      stod(argv[6]),
      stod(argv[7]));

    if (failed) return -2;

    cout << save_json(*motion_plan) << endl;
    return 0;

  }

  return -3;

}
