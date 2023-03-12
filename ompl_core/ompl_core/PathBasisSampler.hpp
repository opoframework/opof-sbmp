#include "ompl_core/Utils.hpp"
#include <ompl/base/StateSampler.h>
#include <random>

namespace core {

class PathBasisSampler : public ompl::base::StateSampler {
public:
  PathBasisSampler(const ompl::base::StateSpace *space, PyObject *basis,
                   PyObject *weights)
      : rng_(std::random_device()()), ompl::base::StateSampler(space),
        space_(space), default_sampler_(space->allocDefaultStateSampler()) {
    // Process basis.
    for (size_t i = 0; i < PyList_Size(basis); i++) {
      basis_.emplace_back();
      for (size_t j = 0; j < PyList_Size(PyList_GetItem(basis, i)); j++) {
        auto state = space_->allocState();
        core::list_to_state(space_, state,
                            PyList_GetItem(PyList_GetItem(basis, i), j));
        basis_.back().emplace_back(state);
      }
    }

    // Process weights.
    weights_ = std::vector<float>(PyList_Size(weights));
    for (size_t i = 0; i < PyList_Size(weights); i++) {
      weights_[i] = PyFloat_AsDouble(PyList_GetItem(weights, i));
    }
  }

  ~PathBasisSampler() {
    for (size_t i = 0; i < basis_.size(); i++) {
      for (size_t j = 0; j < basis_[i].size(); j++) {
        space_->freeState(basis_[i][j]);
      }
    }
  }

  void sampleUniform(ompl::base::State *state) override {
    size_t i = std::discrete_distribution<size_t>(weights_.begin(),
                                                  weights_.end())(rng_);
    size_t j =
        std::uniform_int_distribution<size_t>(0, basis_[i].size() - 1)(rng_);
    default_sampler_->sampleGaussian(state, basis_[i][j],
                                     0.2);
  }

  void sampleUniformNear(ompl::base::State *, const ompl::base::State *,
                         double) override {
    throw std::logic_error("Unsupported function");
  }

  void sampleGaussian(ompl::base::State *, const ompl::base::State *,
                      double) override {
    throw std::logic_error("Unsupported function");
  }

protected:
  std::mt19937 rng_;
  const ompl::base::StateSpace *space_;
  ompl::base::StateSamplerPtr default_sampler_;

  std::vector<std::vector<ompl::base::State *>> basis_;
  std::vector<float> weights_;
  PyObject *sampler_;
};

} // namespace core
