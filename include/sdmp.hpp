/*! \file
 * The Simple Droid Motion Planner (SDMP) Library
 *
 * An extensible library for simple Droid motion planning.
 *
 * A `C++17` compiler is mandatory (`C++14` _might_ be okay).
 */

#pragma once // https://en.wikipedia.org/wiki/Pragma_once#Portability

static_assert(__cplusplus >= 201703L, "c++17 support is required");

#include "sdmp.pb.h" // https://developers.google.com/protocol-buffers/docs/reference/cpp-generated

#include <iostream>

// ---------------------------------------------------------------------------

/**
 * \namespace sdmp
 * The namespace of the library.
 */
namespace sdmp {

/**
 * \var typedef std::shared_ptr<sdmp::MotionPlan> MotionPlanPtr
 * \brief A convenience `typedef` for shared ownership
 */
typedef std::shared_ptr<MotionPlan> MotionPlanPtr;

/**
 * \brief
 * Load a JSON representation of a `MotionPlan`
 *
 * Load a JSON representation of a `MotionPlan`.
 * Note that Protocol Buffers can be used **directly**
 * to load a binary representation, if desired.
 *
 * @param json the `string` to load from
 * @return `nullptr` indicates failure
 */
MotionPlanPtr load_json(const std::string &json);

/**
 * \brief
 * Save a JSON representation of a `MotionPlan`
 *
 * Save a JSON representation of a `MotionPlan`.
 * Note that Protocol Buffers can be used **directly**
 * to save a binary representation, if desired.
 *
 * @param motion_plan the `MotionPlan` to load from
 * @return non-zero indicates failure
 */
std::string save_json(const MotionPlan &motion_plan);

/**
 * \brief Add a circular obstacle to a `MotionPlan`
 *
 * Add a circular obstacle to a `MotionPlan` object.
 * Arguments **are** checked for validity.
 *
 * Make sure that
 * \ref is_valid "is_valid(const MotionPlan &motion_plan)"
 * is called _after_ the `MotionPlan` is **fully** built.
 *
 * (Or another validity checker, depending on the
 * specific type of `MotionPlan` message.)
 *
 * @param the motion plan to act on
 * @param location_x
 * @param location_y
 * @param radius
 * @return `true` if all values were valid and the obstacle was added
 */
bool add_circular_obstacle(MotionPlan &motion_plan, double location_x, double location_y, double radius);

/**
 * \namespace sdmp::bb8
 * Specialized code to handle the `BB8` Droid family.
 */
namespace bb8 {

/**
 * \namespace sdmp::bb8::simple
 * Specialized code to handle "simple" motion plans.
 *
 * For a `BB8` droid, a "simple" motion plan has
 * only circular obstacles and rectangular bounds.
 */
namespace simple {

/**
 * \brief Create a simple `BB8` motion plan.
 *
 * Create a simple `BB8` motion plan with rectangular
 * bounds and no obstacles. Arguments **are**
 * checked for validity.
 *
 * Make sure that
 * \ref is_valid "is_valid(const MotionPlan &motion_plan)"
 * is called _after_ the `MotionPlan` is **fully** built.
 *
 * \note `MotionPlan` objects have "move" semantics, so
 * Copy Elision and Return Value Optimization are assumed.
 *
 * @param droid_radius
 * @param bounds_length
 * @param bounds_width
 * @return `nullptr` indicates failure
 */
MotionPlanPtr create(double droid_radius, double bounds_length, double bounds_width);

/**
 * \brief Check that a motion plan is valid.
 *
 * Verifies that all motion plan objects and
 * parameters are valid, sane, and consistent.
 *
 * It does **not** check that the `path` is a
 * valid solution, only that it is plausible,
 * meaning that each segment is within the
 * requested bounds.
 *
 * \todo
 * Return a more detailed reason code, in
 * case of failure, for diagnostics.
 *
 * @param motion_plan the motion plan to check
 * @return whether or not the plan is valid
 */
bool is_valid(const MotionPlan &motion_plan);

/**
 * \brief Find a path for the motion plan.
 *
 * Compute (and **replace**) the path for the motion plan.
 *
 * The `motion_plan.path` is _always_ cleared when this
 * function is called, before any new path is computed.
 *
 * Make sure that
 * \ref is_valid "is_valid(const MotionPlan &motion_plan)"
 * is `true` _before_ this function is called.
 *
 * \todo
 * Better error codes and code descriptions.
 *
 * @param x_init the initial 'x' coordinate
 * @param y_init the initial 'y' coordinate
 * @param x_goal the 'x' goal coordinate
 * @param y_goal the 'y' goal coordinate
 * @param motion_plan the `MotionPlan` to use
 * @param timeout_seconds take this long at most
 * @param length_threshold return any path shorter than this,
 *                       use zero for the shortest path found
 *                       until timeout
 * @return zero for success
 */
int find_path(MotionPlan &motion_plan,
              double x_init, double y_init, double x_goal, double y_goal,
              double timeout_seconds, double length_threshold = 0.0);

/**
 * \brief Save the motion plan as `GnuPlot` code.
 *
 * Save the motion plan as `GnuPlot` commands for visualization.
 *
 * @param motion_plan
 * @return
 */
std::string save_gnuplot(const MotionPlan &motion_plan);

} // end namespace sdmp::bb8::simple

} // end namespace sdmp::bb8

} // end namespace sdmp

// ---------------------------------------------------------------------------
