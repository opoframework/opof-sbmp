#include "ompl_core/LinearProjection.hpp"
#include "ompl_core/PathBasisSampler.hpp"
#include "ompl_core/Utils.hpp"

#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

static PyObject *solve(PyObject *self, PyObject *args) {
  PyObject *topology;
  PyObject *space_params;
  PyObject *sampler;
  PyObject *validity_checker;
  PyObject *projection;
  PyObject *planner_name;
  PyObject *planner_params;
  PyObject *start;
  PyObject *goal;
  PyObject *timeout;
  PyArg_UnpackTuple(args, "ref", 10, 10, &topology, &space_params, &sampler,
                    &validity_checker, &projection, &planner_name,
                    &planner_params, &start, &goal, &timeout);

  std::string planner_name_s = core::as_string(planner_name);

  // For dictionary iteration.
  PyObject *key, *value;
  Py_ssize_t pos = 0;

  // Construct state space from given topology.
  if (!PyList_Check(topology)) {
    throw std::invalid_argument("Incorrect topology format");
  }
  std::shared_ptr<ompl::base::StateSpace> ss =
      core::create_compound_space(topology);

  // Set space params.
  while (PyDict_Next(space_params, &pos, &key, &value)) {
    std::string k = core::as_string(key);
    if (PyObject_TypeCheck(value, &PyFloat_Type)) {
      ss->params()[k] = PyFloat_AsDouble(value);
    } else if (PyObject_TypeCheck(value, &PyBool_Type)) {
      ss->params()[k] = PyObject_IsTrue(value);
    }
  }

  // Set state sampler.
  if (sampler != Py_None) {
    if (core::as_string(PyTuple_GetItem(sampler, 0)) == "PathBasis") {
      ss->setStateSamplerAllocator(
          [&sampler](const ompl::base::StateSpace *ss) {
            return std::make_shared<core::PathBasisSampler>(
                ss, PyTuple_GetItem(sampler, 1), PyTuple_GetItem(sampler, 2));
          });
    } else {
      throw std::invalid_argument("Unsupported sampler");
    }
  }

  // Set projection.
  if (projection != Py_None) {
    if (core::as_string(PyTuple_GetItem(projection, 0)) == "Linear") {
      ss->registerDefaultProjection(std::make_shared<LinearProjection>(
          ss.get(), PyTuple_GetItem(projection, 1)));
    } else {
      throw std::invalid_argument("Unsupported projection");
    }
  }

  // Set state validity checker.
  auto si = std::make_shared<ompl::base::SpaceInformation>(ss);
  si->setStateValidityChecker(
      [&validity_checker, &ss](const ompl::base::State *state) {
        // Call validity checker.
        PyObject *args = PyTuple_New(1);
        PyTuple_SetItem(args, 0, core::state_to_list(ss.get(), state));
        PyObject *result = core::call(validity_checker, args);
        bool result_bool = PyObject_IsTrue(result);
        Py_DECREF(args);
        Py_DECREF(result);
        return result_bool;
      });
  si->setup();

  // Setup planner.
  ompl::base::PlannerPtr planner;
  if (planner_name_s == "RRTConnect") {
    planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  } else if (planner_name_s == "BiEST") {
    planner = std::make_shared<ompl::geometric::BiEST>(si);
  } else if (planner_name_s == "BKPIECE1") {
    planner = std::make_shared<ompl::geometric::BKPIECE1>(si);
  } else if (planner_name_s == "LBKPIECE1") {
    planner = std::make_shared<ompl::geometric::LBKPIECE1>(si);
  }

  // Set planner params.
  while (PyDict_Next(planner_params, &pos, &key, &value)) {
    std::string k = core::as_string(key);
    if (PyObject_TypeCheck(value, &PyFloat_Type)) {
      planner->params()[k] = PyFloat_AsDouble(value);
    } else if (PyObject_TypeCheck(value, &PyBool_Type)) {
      planner->params()[k] = PyObject_IsTrue(value);
    }
  }

  // Setup planning problem.
  auto pd = std::make_shared<ompl::base::ProblemDefinition>(si);
  auto start_state = ss->allocState();
  core::list_to_state(ss.get(), start_state, start);
  ss->enforceBounds(start_state);
  auto goal_state = ss->allocState();
  core::list_to_state(ss.get(), goal_state, goal);
  ss->enforceBounds(goal_state);
  pd->setStartAndGoalStates(start_state, goal_state);
  planner->setProblemDefinition(pd);

  // Track planner calls and time.
  size_t iterations = 0;
  auto start_time = std::chrono::system_clock::now();
  auto end_time =
      start_time + std::chrono::milliseconds(
                       int(std::round(1000 * PyFloat_AsDouble(timeout))));
  ompl::base::PlannerTerminationCondition ptc([&iterations, &end_time]() {
    iterations++;
    return ompl::time::now() > end_time;
  });

  // Solve.
  bool success;
  try {
    planner->setup();
    success = planner->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION;
  } catch (core::OmplCoreException &e) {
    return nullptr;
  }
  auto finish_time = std::chrono::system_clock::now();
  ss->freeState(start_state);
  ss->freeState(goal_state);

  // Construct result path.
  PyObject *trajectory = PyList_New(0);
  /*
  if (pd->hasSolution()) {
    auto path = pd->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    //path->interpolate(100;
    for (size_t i = 0; i < path->getStateCount(); i++) {
      PyList_Append(trajectory, state_to_list(ss.get(), path->getState(i)));
    }
  }
  */

  // Construct result.
  PyObject *result = PyDict_New();
  PyDict_SetItem(result, core::to_string("success"),
                 PyLong_FromDouble(success ? 1.0 : 0.0));
  PyDict_SetItem(
      result, core::to_string("time"),
      PyFloat_FromDouble(
          success ? (std::chrono::duration_cast<std::chrono::milliseconds>(
                         finish_time - start_time)
                         .count() /
                     1000.0)
                  : std::numeric_limits<double>::quiet_NaN()));
  PyDict_SetItem(result, core::to_string("iterations"),
                 PyLong_FromLong(iterations));
  PyDict_SetItem(
      result, core::to_string("length"),
      PyFloat_FromDouble(success ? pd->getSolutionPath()->length()
                                 : std::numeric_limits<double>::quiet_NaN()));
  PyDict_SetItem(result, core::to_string("trajectory"), trajectory);
  core::decref_dict(result);

  return result;
}

// Exported methods are collected in a table
PyMethodDef methods[] = {
    {"solve", (PyCFunction)solve, METH_VARARGS,
     "Solves a given motion planning problem"},
    {NULL, NULL, 0, NULL} // Sentinel value ending the table
};

// A struct contains the definition of a module
PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "ompl_core", // Module name
    "Module containing compiled codes to run OMPL planners",
    -1, // Optional size of the module state memory
    methods,
    NULL, // Optional slot definitions
    NULL, // Optional traversal function
    NULL, // Optional clear function
    NULL  // Optional module deallocation function
};

// The module init function
PyMODINIT_FUNC PyInit_ompl_core(void) {
  ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);
  return PyModule_Create(&module);
}
