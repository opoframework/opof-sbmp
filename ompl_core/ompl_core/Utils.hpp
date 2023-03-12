#pragma once

#include <Python.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace core {

class OmplCoreException : public std::exception {
public:
  const char *what() { return "ompl_core exception"; }
};

inline PyObject *call(PyObject *f, PyObject *args) {
  PyObject *result = PyObject_CallObject(f, args);
  if (result == nullptr) {
    throw OmplCoreException();
  }
  return result;
}

inline std::string as_string(PyObject *x) {
  return std::string(PyBytes_AsString(PyUnicode_AsUTF8String(PyObject_Str(x))));
}

inline PyObject *to_string(const std::string &s) {
  return PyUnicode_DecodeUTF8(s.c_str(), s.length(), nullptr);
}

inline void print(PyObject *x) { std::cout << as_string(x) << std::endl; }

inline std::vector<double> state_to_vector(const ompl::base::StateSpace *ss,
                                           const ompl::base::State *state) {
  if (ss->isCompound()) {
    auto cs = ss->as<ompl::base::CompoundStateSpace>();
    auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
    std::vector<double> vec(cs->getDimension());
    for (size_t i = 0; i < cs->getDimension(); i++) {
      if (cs->getSubspace(i)->getType() ==
          ompl::base::STATE_SPACE_REAL_VECTOR) {
        vec[i] = cstate->components[i]
                     ->as<ompl::base::RealVectorStateSpace::StateType>()
                     ->values[0];
      } else if (cs->getSubspace(i)->getType() == ompl::base::STATE_SPACE_SO2) {
        vec[i] = cstate->components[i]
                     ->as<ompl::base::SO2StateSpace::StateType>()
                     ->value;
      } else {
        throw std::logic_error("Incorrect state space");
      }
    }
    return vec;
  } else if (ss->getType() == ompl::base::STATE_SPACE_REAL_VECTOR) {
    auto rs = ss->as<ompl::base::RealVectorStateSpace>();
    auto rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> vec(rs->getDimension());
    for (size_t i = 0; i < rs->getDimension(); i++) {
      vec[i] = rstate->values[i];
    }
    return vec;
  } else {
    throw std::logic_error("Unsupported state space");
  }
}

inline PyObject *state_to_list(const ompl::base::StateSpace *ss,
                               const ompl::base::State *state) {
  if (ss->isCompound()) {
    auto cs = ss->as<ompl::base::CompoundStateSpace>();
    auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
    PyObject *list = PyList_New(cs->getDimension());
    for (size_t i = 0; i < cs->getDimension(); i++) {
      if (cs->getSubspace(i)->getType() ==
          ompl::base::STATE_SPACE_REAL_VECTOR) {
        PyList_SetItem(
            list, i,
            PyFloat_FromDouble(
                cstate->components[i]
                    ->as<ompl::base::RealVectorStateSpace::StateType>()
                    ->values[0]));
      } else if (cs->getSubspace(i)->getType() == ompl::base::STATE_SPACE_SO2) {
        PyList_SetItem(
            list, i,
            PyFloat_FromDouble(cstate->components[i]
                                   ->as<ompl::base::SO2StateSpace::StateType>()
                                   ->value));
      } else {
        throw std::logic_error("Incorrect state space");
      }
    }
    return list;
  } else if (ss->getType() == ompl::base::STATE_SPACE_REAL_VECTOR) {
    auto rs = ss->as<ompl::base::RealVectorStateSpace>();
    auto rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    PyObject *list = PyList_New(rs->getDimension());
    for (size_t i = 0; i < rs->getDimension(); i++) {
      PyList_SetItem(list, i, PyFloat_FromDouble(rstate->values[i]));
    }
    return list;
  } else {
    throw std::logic_error("Unsupported state space");
  }
}

inline void list_to_state(const ompl::base::StateSpace *ss,
                          ompl::base::State *state, PyObject *list) {
  if (ss->isCompound()) {
    auto cs = ss->as<ompl::base::CompoundStateSpace>();
    auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
    for (size_t i = 0; i < cs->getDimension(); i++) {
      double value = PyFloat_AsDouble(PyList_GetItem(list, i));
      if (cs->getSubspace(i)->getType() ==
          ompl::base::STATE_SPACE_REAL_VECTOR) {
        cstate->components[i]
            ->as<ompl::base::RealVectorStateSpace::StateType>()
            ->values[0] = value;
      } else if (cs->getSubspace(i)->getType() == ompl::base::STATE_SPACE_SO2) {
        cstate->components[i]
            ->as<ompl::base::SO2StateSpace::StateType>()
            ->value = value;
      } else {
        throw std::logic_error("Incorrect state space");
      }
    }
  } else if (ss->getType() == ompl::base::STATE_SPACE_REAL_VECTOR) {
    auto rs = ss->as<ompl::base::RealVectorStateSpace>();
    auto rstate = state->as<ompl::base::RealVectorStateSpace::StateType>();
    for (size_t i = 0; i < rs->getDimension(); i++) {
      rstate->values[i] = PyFloat_AsDouble(PyList_GetItem(list, i));
    }
  } else {
    throw std::logic_error("Unsupported state space");
  }
}

// According to the source code,
// https://github.com/python/cpython/blob/1b55b65638254aa78b005fbf0b71fb02499f1852/Objects/dictobject.c#L1532,
// values are incref'ed which means we need to decref once on the inputs.
inline void decref_dict(PyObject *dict) {
  PyObject *keys = PyDict_Keys(dict);
  for (size_t i = 0; i < PyList_Size(keys); i++) {
    Py_DECREF(PyDict_GetItem(dict, PyList_GetItem(keys, i)));
  }
  Py_DECREF(keys);
}

inline std::shared_ptr<ompl::base::CompoundStateSpace>
create_compound_space(PyObject *topology) {
  auto ss = std::make_shared<ompl::base::CompoundStateSpace>();
  for (size_t i = 0; i < PyList_Size(topology); i++) {
    PyObject *entry = PyList_GetItem(topology, i);
    const std::string type = core::as_string(PyTuple_GetItem(entry, 0));
    if (type == "P") {
      auto subspace = std::make_shared<ompl::base::RealVectorStateSpace>(1);
      if (PyTuple_Size(entry) != 3) {
        throw std::invalid_argument("Incorrect topology format");
      }
      subspace->setBounds(PyFloat_AsDouble(PyTuple_GetItem(entry, 1)),
                          PyFloat_AsDouble(PyTuple_GetItem(entry, 2)));
      ss->addSubspace(subspace, 1.0);
    } else if (type == "R") {
      auto subspace = std::make_shared<ompl::base::SO2StateSpace>();
      if (PyTuple_Size(entry) != 1) {
        throw std::invalid_argument("Incorrect topology format");
      }
      ss->addSubspace(subspace, 1.0);
    } else {
      throw std::invalid_argument("Incorrect topology format");
    }
  }
  return ss;
}

} // namespace core
