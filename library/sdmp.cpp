#include "sdmp.hpp"

#include <google/protobuf/util/json_util.h>

// Adapted from https://ompl.kavrakilab.org/optimalPlanningTutorial.html

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathGeometric.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

#include <string>

using namespace std;
using namespace sdmp;

// ---------------------------------------------------------------------------
// Initializations that are run on library load

namespace {

  [[maybe_unused]]
  const bool is_initialized = []() {

    ompl::msg::noOutputHandler(); // no OMPL logging

    return true;

  }();

}

// ---------------------------------------------------------------------------

// See https://stackoverflow.com/questions/34906305/protocol-buffer3-and-json
// See https://developers.google.com/protocol-buffers/docs/reference/cpp/google.protobuf.util.json_util

MotionPlanPtr sdmp::load_json(const string &json_string) {

  using namespace google::protobuf::util;

  // Attempt to parse the given JSON string
  MotionPlanPtr motion_plan(new MotionPlan());
  auto result = JsonStringToMessage(json_string, motion_plan.get());

  // On failure, delete the object and set the pointer to null
  if (!result.ok()) {
    motion_plan.reset();
  }

  return motion_plan;

}

string sdmp::save_json(const MotionPlan &motion_plan) {

  using namespace google::protobuf::util;

  JsonPrintOptions options;
  options.always_print_primitive_fields = true;

  string json_string;
  MessageToJsonString(motion_plan, &json_string, options);

  return json_string;

}

// ---------------------------------------------------------------------------

string sdmp::bb8::simple::save_gnuplot(const MotionPlan &motion_plan)
{
  ostringstream commands;

  const double alpha = 0.25;

  commands << "set termoption enhanced" << endl;
  commands << "set title '{/:Bold Simple Droid Motion Plan}'" << endl;

  commands << "set xlabel '{/:Bold length} / {/:Italic m} / {/:Italic x}'" << endl;
  commands << "set ylabel '{/:Bold width} / {/:Italic n} / {/:Italic y}'"  << endl;
  commands << "set xrange [0:" << motion_plan.rectangle().length() << "]" << endl;
  commands << "set yrange [0:" << motion_plan.rectangle().width()  << "]" << endl;
  commands << "set size ratio " << motion_plan.rectangle().width() / motion_plan.rectangle().length() << endl;
  commands << "set grid" << endl;

  unsigned index = 0;

  for (int i = 0; i < motion_plan.path_size(); i++) {
    const double x(motion_plan.path(i).x()), y(motion_plan.path(i).y()), r(motion_plan.bb8().radius());
    commands << "set object " << (++index) << " circle at " << x << "," << y << " size " << r
      << " fillcolor rgb 'dark-orange' fillstyle transparent solid " << alpha << endl;
  }

  if (motion_plan.path_size() >= 2) {
    for (int i = 0; i < motion_plan.path_size() - 1; i++) {
      commands << "set arrow from " << motion_plan.path(i).x() << "," << motion_plan.path(i).y() << " to "
      << motion_plan.path(i + 1).x() << "," << motion_plan.path(i + 1).y()
      << " linewidth 2 linecolor rgb 'dark-green'" << endl;
    }
  }

  for (int i = 0; i < motion_plan.obstacle_size(); i++) {
    const auto circle(motion_plan.obstacle(i).circle());
    const double x(circle.coordinates().x()), y(circle.coordinates().y()), r(circle.radius());
    commands << "set object " << (++index) << " circle at " << x << "," << y << " size " << r
      << " fillcolor rgb 'black' fillstyle transparent solid " << alpha << endl;
  }

  commands << "plot NaN notitle" << endl; // this command generates the plot even though we technically have no data

  return commands.str();
}

// ---------------------------------------------------------------------------

MotionPlanPtr sdmp::bb8::simple::create(double droid_radius, double bounds_length, double bounds_width)
{
  MotionPlanPtr motion_plan(new MotionPlan());

  if (!isfinite(droid_radius) || droid_radius < 0.0 ||
      !isfinite(bounds_length) || bounds_length < 0.0 ||
      !isfinite(bounds_width) || bounds_width < 0.0 ) {
    motion_plan.reset();
    return motion_plan;
  }

  auto bb8(new BB8());
  bb8->set_radius(droid_radius);
  motion_plan->set_allocated_bb8(bb8);

  auto rectangle(new Rectangle());
  rectangle->set_length(bounds_length);
  rectangle->set_width(bounds_width);
  motion_plan->set_allocated_rectangle(rectangle);

  return motion_plan;
}

bool sdmp::add_circular_obstacle(MotionPlan &motion_plan, double location_x, double location_y, double radius) {

  if (!isfinite(location_x) || location_x < 0.0 ||
      !isfinite(location_y) || location_y < 0.0 ||
      !isfinite(radius) || radius < 0.0 ) {
    return false;
  }

  auto *obstacle = motion_plan.add_obstacle();
  auto *circle = new Circle();
  circle->set_radius(radius);
  auto *coordinates = new Coordinates();
  coordinates->set_x(location_x);
  coordinates->set_y(location_y);
  circle->set_allocated_coordinates(coordinates);
  obstacle->set_allocated_circle(circle);

  return true;
}

// ---------------------------------------------------------------------------

bool sdmp::bb8::simple::is_valid(const MotionPlan &motion_plan)
{
  if (!motion_plan.IsInitialized()) return false;

  if (!motion_plan.has_bb8()) return false;

  const auto &bb8(motion_plan.bb8());
  if (!bb8.IsInitialized()) return false;
  if (!isfinite(bb8.radius())) return false;
  if (bb8.radius() <= 0.0) return false;

  if (!motion_plan.has_rectangle()) return false;
  const auto &bounds(motion_plan.rectangle());
  if (!bounds.IsInitialized()) return false;
  if (!isfinite(bounds.length())) return false;
  if (bounds.length() <= 0.0) return false;
  if (!isfinite(bounds.width())) return false;
  if (bounds.width() <= 0.0) return false;

  for (int i = 0; i < motion_plan.obstacle_size(); i++) {

    const auto &obstacle(motion_plan.obstacle(i));
    if (!obstacle.IsInitialized()) return false;
    if (!obstacle.has_circle()) return false;

    const auto &circle(obstacle.circle());
    if (!circle.IsInitialized()) return false;
    if (!isfinite(circle.radius())) return false;
    if (circle.radius() <= 0) return false;

  }

  for (int i = 0; i < motion_plan.path_size(); i++) {

    const auto &path(motion_plan.path(i));
    if (!path.IsInitialized()) return false;
    if (!isfinite(path.x())) return false;
    if (path.x() < bb8.radius()) return false;
    if (path.x() > bounds.length() - bb8.radius()) return false;
    if (!isfinite(path.y())) return false;
    if (path.y() < bb8.radius()) return false;
    if (path.y() > bounds.width() - bb8.radius()) return false;

  }

  return true;
}

// ---------------------------------------------------------------------------

namespace {

// ---------------------------------------------------------------------------
// This class is used by OMPL to plan the motion

class ValidityChecker
    : public ob::StateValidityChecker {

  const MotionPlan &motion_plan;

 public:

  // Assumption on input: assert(is_valid(motion_plan))
  explicit ValidityChecker(const ob::SpaceInformationPtr &si, const MotionPlan &motion_plan)
      : ob::StateValidityChecker(si), motion_plan(motion_plan) { }

  // Returns whether the given state's position overlaps an obstacle or boundary
  bool isValid(const ob::State *state) const override {
    return this->clearance(state) > 0.0;
  }

  // Returns the distance from the given state's position to an obstacle or boundary
  double clearance(const ob::State *state) const override {

    // Get the Droid size and bounding rectangle

    const double r_d = motion_plan.bb8().radius();
    const double x_min = 0.0, y_min = 0.0;
    const double x_max = motion_plan.rectangle().length();
    const double y_max = motion_plan.rectangle().width();

    // Get the current (x,y) Droid coordinate position
    const auto *state2D = state->as<ob::RealVectorStateSpace::StateType>();
    double x = state2D->values[0];
    double y = state2D->values[1];

    // Calculate the minimum clearance to all obstacles and boundary

    double clearance = numeric_limits<double>::infinity();

    clearance = min(clearance, (x - r_d) - x_min);
    clearance = min(clearance, (y - r_d) - y_min);
    clearance = min(clearance, x_max - (x + r_d));
    clearance = min(clearance, y_max - (y + r_d));

    for (int i = 0; i < motion_plan.obstacle_size(); i++) {
      const auto obstacle = motion_plan.obstacle(i);
      const double r_o = obstacle.circle().radius();
      const double delta_x = x - obstacle.circle().coordinates().x();
      const double delta_y = y - obstacle.circle().coordinates().y();
      const double distance = hypot(delta_x, delta_y); // compute sqrt(x*x+y*y) without undue overflow or underflow
      clearance = min(clearance, distance - (r_d + r_o));
    }

    return clearance;

  }

};

} // end anonymous namespace -------------------------------------------------

int sdmp::bb8::simple::find_path(MotionPlan &motion_plan,
    double x_init, double y_init, double x_goal, double y_goal,
    double timeout_seconds, double length_threshold)
{
  motion_plan.clear_path();

  if (!isfinite(x_init)) return -1;
  if (!isfinite(y_init)) return -1;
  if (!isfinite(x_goal)) return -1;
  if (!isfinite(y_goal)) return -1;
  if (!isfinite(timeout_seconds)) return -1;
  if (timeout_seconds <= 0.0) return -1; // timeout must be positive
  if (!isfinite(length_threshold)) return -1;
  if (length_threshold < 0.0) return -1; // zero means 'keep trying'

  // The `ValidityChecker` object will verify that initial and goal coordinates are valid

  // Construct the robot state space in which we're planning.
  ob::StateSpacePtr space(new ob::RealVectorStateSpace());

  // Set the dimension bounds for $R^2$ as given
  auto *r2ss = space->as<ob::RealVectorStateSpace>();
  r2ss->addDimension("length/m/x", 0.0, motion_plan.rectangle().length());
  r2ss->addDimension("width/n/y", 0.0, motion_plan.rectangle().width());
  r2ss->setup();

  // Construct a space information instance for this state space
  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

  // Set the object used to check which states in the space are valid
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si, motion_plan)));

  // Setup the SpaceInformation
  si->setup();

  // Set our robot's starting state
  ob::ScopedState<> start(space);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] = x_init;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = y_init;

  // Set our robot's goal state
  ob::ScopedState<> goal(space);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = x_goal;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = y_goal;

  // Create a problem instance
  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

  // Set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

  // Set the optimization objective and threshold
  ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
  obj->setCostThreshold(ob::Cost(length_threshold));
  pdef->setOptimizationObjective(obj);

  // Construct our optimizing planner using the RRTstar algorithm.
  ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));

  // Set the problem instance for our planner to solve
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();

  // Attempt to solve the planning problem within one second of planning time
  ob::PlannerStatus solved = optimizingPlanner->solve(timeout_seconds);

  const ob::PlannerStatus::StatusType status(solved); // TODO Use this for more informative error conditions
  if (!solved) return -2;

  // Save the output path to the motion plan

  assert(r2ss->getDimension() == 2);
  assert(motion_plan.path_size() == 0);

  const auto path = pdef->getSolutionPath()->as<og::PathGeometric>();
  const auto states = path->getStates();
  for (auto state : states) {
    auto motion_path = motion_plan.add_path();
    const auto &values = *state->as<ob::RealVectorStateSpace::StateType>();
    motion_path->set_x(values[0]);
    motion_path->set_y(values[1]);
  }

  // Done
  return 0;
}

// ---------------------------------------------------------------------------
