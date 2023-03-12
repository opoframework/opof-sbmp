#include "ompl_core/Utils.hpp"
#include <ompl/base/ProjectionEvaluator.h>

class LinearProjection : public ompl::base::ProjectionEvaluator {
public:
  LinearProjection(const ompl::base::StateSpace *space, PyObject *weights)
      : ompl::base::ProjectionEvaluator(space), space_(space) {
    // Process weights.
    weights_ = std::vector<std::vector<float>>(PyList_Size(weights));
    for (size_t i = 0; i < PyList_Size(weights); i++) {
      weights_[i] = std::vector<float>(PyList_Size(PyList_GetItem(weights, i)));
      for (size_t j = 0; j < PyList_Size(PyList_GetItem(weights, i)); j++) {
        weights_[i][j] =
            PyFloat_AsDouble(PyList_GetItem(PyList_GetItem(weights, i), j));
      }
    }
  }

  unsigned int getDimension(void) const override { return weights_.size(); }

  void project(const ompl::base::State *state,
               Eigen::Ref<Eigen::VectorXd> projection) const override {
    std::vector<double> v = core::state_to_vector(space_, state);
    for (size_t i = 0; i < weights_.size(); i++) {
      projection(i) = 0;
      for (size_t j = 0; j < weights_[i].size(); j++) {
        projection(i) += weights_[i][j] * v[j];
      }
    }
  }

protected:
  const ompl::base::StateSpace *space_;

  std::vector<std::vector<float>> weights_;
};
