#include "drake/traj_opt/trajectory_optimizer.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "drake/common/profiler.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/systems/framework/diagram.h"
#include "drake/traj_opt/penta_diagonal_solver.h"
#include "drake/traj_opt/penta_diagonal_to_petsc_matrix.h"

using drake::multibody::fem::internal::PetscSolverStatus;
using drake::multibody::fem::internal::PetscSymmetricBlockSparseMatrix;

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace traj_opt {

using geometry::GeometryId;
using geometry::SignedDistancePair;
using internal::PentaDiagonalFactorization;
using internal::PentaDiagonalFactorizationStatus;
using math::RigidTransform;
using multibody::Body;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;
using multibody::SpatialForce;
using multibody::SpatialVelocity;
using systems::System;

template <typename T>
TrajectoryOptimizer<T>::TrajectoryOptimizer(const Diagram<T>* diagram,
                                            const MultibodyPlant<T>* plant,
                                            const ProblemDefinition& prob,
                                            const SolverParameters& params)
    : diagram_{diagram}, plant_(plant), prob_(prob), params_(params) {
  // Workaround for when a plant context is not provided.
  // Valid context should be obtained with EvalPlantContext() instead.
  // TODO(amcastro-tri): get rid of this.
  owned_context_ = diagram->CreateDefaultContext();
  context_ = &diagram->GetMutableSubsystemContext(*plant, owned_context_.get());

  // Define joint damping coefficients.
  joint_damping_ = VectorX<T>::Zero(plant_->num_velocities());
  for (JointIndex j(0); j < plant_->num_joints(); ++j) {
    const Joint<T>& joint = plant_->get_joint(j);
    const int velocity_start = joint.velocity_start();
    const int nv = joint.num_velocities();
    joint_damping_.segment(velocity_start, nv) = joint.damping_vector();
  }

  // Define unactuated degrees of freedom
  const MatrixX<T> B = plant_->MakeActuationMatrix();
  if (B.size() > 0) {
    // if B is of size zero, assume the system is fully actuated
    for (int i = 0; i < plant_->num_velocities(); ++i) {
      if (B.row(i).sum() == 0) {
        unactuated_dofs_.push_back(i);
      }
    }
  }

  // Must have a target position and velocity specified for each time step
  DRAKE_DEMAND(static_cast<int>(prob.q_nom.size()) == (num_steps() + 1));
  DRAKE_DEMAND(static_cast<int>(prob.v_nom.size()) == (num_steps() + 1));

  // Create an autodiff optimizer if we need exact gradients
  if constexpr (std::is_same_v<T, double>) {
    if ((params_.gradients_method == GradientsMethod::kAutoDiff) ||
        params_.exact_hessian) {
      diagram_ad_ = systems::System<double>::ToAutoDiffXd(*diagram);
      plant_ad_ = dynamic_cast<const MultibodyPlant<AutoDiffXd>*>(
          &diagram_ad_->GetSubsystemByName(plant->get_name()));
      DRAKE_DEMAND(plant_ad_ != nullptr);
      SolverParameters params_ad(params);
      if (params_.gradients_method == GradientsMethod::kAutoDiff) {
        // If we're using autodiff to compute gradients, then the autodiff copy
        // of the plant should not be able to compute its own gradients
        params_ad.gradients_method = GradientsMethod::kNoGradients;
      } else {
        params_ad.gradients_method = GradientsMethod::kCentralDifferences;
      }
      optimizer_ad_ = std::make_unique<TrajectoryOptimizer<AutoDiffXd>>(
          diagram_ad_.get(), plant_ad_, prob, params_ad);
      // TODO(vincekurtz): move state's destructor and possible other
      // implementation to the source?
      state_ad_ = std::unique_ptr<TrajectoryOptimizerState<AutoDiffXd>>(
          new TrajectoryOptimizerState<AutoDiffXd>(num_steps(), *diagram_ad_,
                                                   *plant_ad_,
                                                   num_equality_constraints()));
    }
  } else {
    if (params_.gradients_method == GradientsMethod::kAutoDiff) {
      throw std::runtime_error(
          "Analytical gradients not supported for "
          "TrajectoryOptimizer<AutoDiffXd>.");
    }
  }

  // Print warning if parallelization is requested with a slower gradient method
  if ((params_.gradients_method != GradientsMethod::kForwardDifferences) &&
      (params_.num_threads != 1)) {
    drake::log()->warn(
        "Parallel derivatives are currently supported only for forward "
        "differences, not for central differences or autodiff.");
  }
}

template <typename T>
const T TrajectoryOptimizer<T>::EvalCost(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().cost_up_to_date) {
    state.mutable_cache().cost = CalcCost(state);
    state.mutable_cache().cost_up_to_date = true;
  }
  return state.cache().cost;
}

template <typename T>
T TrajectoryOptimizer<T>::CalcCost(
    const TrajectoryOptimizerState<T>& state) const {
  INSTRUMENT_FUNCTION("Computes the total cost.");
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);
  T cost = CalcCost(state.q(), v, tau, &state.workspace);

  // Add a proximal operator term to the cost, if requested
  if (params_.proximal_operator) {
    const std::vector<VectorX<T>>& q = state.q();
    const std::vector<VectorX<T>>& q_last =
        state.proximal_operator_data().q_last;
    const std::vector<VectorX<T>>& H_diag =
        state.proximal_operator_data().H_diag;
    for (int t = 0; t <= num_steps(); ++t) {
      cost += T(0.5 * params_.rho_proximal * (q[t] - q_last[t]).transpose() *
                H_diag[t].asDiagonal() * (q[t] - q_last[t]));
    }
  }

  return cost;
}

template <typename T>
T TrajectoryOptimizer<T>::CalcCost(
    const std::vector<VectorX<T>>& q, const std::vector<VectorX<T>>& v,
    const std::vector<VectorX<T>>& tau,
    TrajectoryOptimizerWorkspace<T>* workspace) const {
  T cost = 0;
  VectorX<T>& q_err = workspace->q_size_tmp1;
  VectorX<T>& v_err = workspace->v_size_tmp1;

  // Running cost
  for (int t = 0; t < num_steps(); ++t) {
    q_err = q[t] - prob_.q_nom[t];
    v_err = v[t] - prob_.v_nom[t];
    cost += T(q_err.transpose() * prob_.Qq * q_err);
    cost += T(v_err.transpose() * prob_.Qv * v_err);
    cost += T(tau[t].transpose() * prob_.R * tau[t]);
  }

  // Scale running cost by dt (so the optimization problem we're solving doesn't
  // change so dramatically when we change the time step).
  cost *= time_step();

  // Terminal cost
  q_err = q[num_steps()] - prob_.q_nom[num_steps()];
  v_err = v[num_steps()] - prob_.v_nom[num_steps()];
  cost += T(q_err.transpose() * prob_.Qf_q * q_err);
  cost += T(v_err.transpose() * prob_.Qf_v * v_err);

  return cost;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcVelocities(
    const std::vector<VectorX<T>>& q, const std::vector<MatrixX<T>>& Nplus,
    std::vector<VectorX<T>>* v) const {
  // x = [x0, x1, ..., xT]
  DRAKE_DEMAND(static_cast<int>(q.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(Nplus.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(v->size()) == num_steps() + 1);

  v->at(0) = prob_.v_init;
  for (int t = 1; t <= num_steps(); ++t) {
    v->at(t) = Nplus[t] * (q[t] - q[t - 1]) / time_step();
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcAccelerations(
    const std::vector<VectorX<T>>& v, std::vector<VectorX<T>>* a) const {
  DRAKE_DEMAND(static_cast<int>(v.size()) == num_steps() + 1);
  DRAKE_DEMAND(static_cast<int>(a->size()) == num_steps());

  for (int t = 0; t < num_steps(); ++t) {
    a->at(t) = (v[t + 1] - v[t]) / time_step();
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamics(
    const TrajectoryOptimizerState<T>& state, const std::vector<VectorX<T>>& a,
    std::vector<VectorX<T>>* tau) const {
  // Generalized forces aren't defined for the last timestep
  // TODO(vincekurtz): additional checks that q_t, v_t, tau_t are the right size
  // for the plant?
  DRAKE_DEMAND(static_cast<int>(a.size()) == num_steps());
  DRAKE_DEMAND(static_cast<int>(tau->size()) == num_steps());

#if defined(_OPENMP)
#pragma omp parallel for num_threads(params_.num_threads)
#endif
  for (int t = 0; t < num_steps(); ++t) {
    TrajectoryOptimizerWorkspace<T>& workspace =
        state.per_timestep_workspace[t];
    const Context<T>& context_tp = EvalPlantContext(state, t + 1);
    // All dynamics terms are treated implicitly, i.e.,
    // tau[t] = M(q[t+1]) * a[t] - k(q[t+1],v[t+1]) - f_ext[t+1]
    CalcInverseDynamicsSingleTimeStep(context_tp, a[t], &workspace,
                                      &tau->at(t));
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsSingleTimeStep(
    const Context<T>& context, const VectorX<T>& a,
    TrajectoryOptimizerWorkspace<T>* workspace, VectorX<T>* tau) const {
  plant().CalcForceElementsContribution(context, &workspace->f_ext);

  // Add in contact force contribution to f_ext
  if (plant().geometry_source_is_registered()) {
    // Only compute contact forces if the plant is connected to a scene graph
    // TODO(vincekurtz): perform this check earlier, and maybe print some
    // warnings to stdout if we're not connected (we do want to be able to run
    // problems w/o contact sometimes)
    CalcContactForceContribution(context, &workspace->f_ext);
  }

  // Inverse dynamics computes tau = M*a - k(q,v) - f_ext
  *tau = plant().CalcInverseDynamics(context, a, workspace->f_ext);
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContactForceContribution(
    const Context<T>& context, MultibodyForces<T>* forces) const {
  using std::abs;
  using std::exp;
  using std::log;
  using std::max;
  using std::pow;
  using std::sqrt;

  // Compliant contact parameters
  const double k = params_.contact_stiffness;
  const double sigma = params_.smoothing_factor;
  const double dissipation_velocity = params_.dissipation_velocity;

  // Friction parameters.
  const double vs = params_.stiction_velocity;     // Regularization.
  const double mu = params_.friction_coefficient;  // Coefficient of friction.

  // Compute the distance at which contact forces are zero: we don't need to do
  // any geometry queries beyond this distance
  const double eps = sqrt(std::numeric_limits<double>::epsilon());
  double threshold = -sigma * log(exp(eps / (sigma * k)) - 1.0);

  // Get signed distance pairs
  const geometry::QueryObject<T>& query_object =
      plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const drake::geometry::SceneGraphInspector<T>& inspector =
      query_object.inspector();
  const std::vector<SignedDistancePair<T>>& signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(threshold);

  for (const SignedDistancePair<T>& pair : signed_distance_pairs) {
    // Normal outwards from A.
    const Vector3<T> nhat = -pair.nhat_BA_W;

    // Get geometry and transformation data for the witness points
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index =
        plant().geometry_id_to_body_index().at(geometryA_id);
    const Body<T>& bodyA = plant().get_body(bodyA_index);
    const BodyIndex bodyB_index =
        plant().geometry_id_to_body_index().at(geometryB_id);
    const Body<T>& bodyB = plant().get_body(bodyB_index);

    // Body poses in world.
    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, bodyA);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, bodyB);

    // Geometry poses in body frames.
    const math::RigidTransform<T> X_AGa =
        inspector.GetPoseInFrame(geometryA_id).template cast<T>();
    const math::RigidTransform<T> X_BGb =
        inspector.GetPoseInFrame(geometryB_id).template cast<T>();

    // Position of the witness points in the world frame.
    const auto& p_GaCa_Ga = pair.p_ACa;
    const RigidTransform<T> X_WGa = X_WA * X_AGa;
    const Vector3<T> p_WCa_W = X_WGa * p_GaCa_Ga;
    const auto& p_GbCb_Gb = pair.p_BCb;
    const RigidTransform<T> X_WGb = X_WB * X_BGb;
    const Vector3<T> p_WCb_W = X_WGb * p_GbCb_Gb;

    // We define the (common, unique) contact point C as the midpoint between
    // witness points Ca and Cb.
    const Vector3<T> p_WC = 0.5 * (p_WCa_W + p_WCb_W);

    // Shift vectors.
    const Vector3<T> p_AC_W = p_WC - X_WA.translation();
    const Vector3<T> p_BC_W = p_WC - X_WB.translation();

    // Velocities.
    const SpatialVelocity<T>& V_WA =
        plant().EvalBodySpatialVelocityInWorld(context, bodyA);
    const SpatialVelocity<T>& V_WB =
        plant().EvalBodySpatialVelocityInWorld(context, bodyB);
    const SpatialVelocity<T> V_WAc = V_WA.Shift(p_AC_W);
    const SpatialVelocity<T> V_WBc = V_WB.Shift(p_BC_W);

    // Relative contact velocity.
    const Vector3<T> v_AcBc_W = V_WBc.translational() - V_WAc.translational();

    // Split into normal and tangential components.
    const T vn = nhat.dot(v_AcBc_W);
    const Vector3<T> vt = v_AcBc_W - vn * nhat;

    // Normal dissipation follows a smoothed Hunt and Crossley model
    T dissipation_factor = 0.0;
    const T s = vn / dissipation_velocity;
    if (s < 0) {
      dissipation_factor = 1 - s;
    } else if (s < 2) {
      dissipation_factor = (s - 2) * (s - 2) / 4;
    }

    // (Compliant) force in the normal direction increases linearly at a rate
    // of k Newtons per meter, with some smoothing defined by sigma.
    T compliant_fn;
    const T exponent = - pair.distance / sigma;
    if (exponent >= 37) {
      // If the exponent is going to be very large, replace with the
      // functional limit.
      // N.B. x = 37 is the first integer such that exp(x)+1 = exp(x) in
      // double precision.
      compliant_fn = -k * pair.distance;
    } else {
      compliant_fn = sigma * k * log(1 + exp(exponent));
    }
    const T fn = compliant_fn * dissipation_factor;

    // Tangential frictional component.
    // N.B. This model is algebraically equivalent to:
    //  ft = -mu*fn*sigmoid(||vt||/vs)*vt/||vt||.
    // with the algebraic sigmoid function defined as sigmoid(x) =
    // x/sqrt(1+x^2). The algebraic simplification is performed to avoid
    // division by zero when vt = 0 (or loss of precision when close to zero).
    const Vector3<T> that_regularized =
        -vt / sqrt(vs * vs + vt.squaredNorm());
    const Vector3<T> ft_BC_W = that_regularized * mu * fn;

    // Total contact force on B at C, expressed in W.
    const Vector3<T> f_BC_W = nhat * fn + ft_BC_W;

    // Spatial contact forces on bodies A and B.
    const SpatialForce<T> F_BC_W(Vector3<T>::Zero(), f_BC_W);
    const SpatialForce<T> F_BBo_W = F_BC_W.Shift(-p_BC_W);

    const SpatialForce<T> F_AC_W(Vector3<T>::Zero(), -f_BC_W);
    const SpatialForce<T> F_AAo_W = F_AC_W.Shift(-p_AC_W);

    // Add the forces into the given MultibodyForces
    forces->mutable_body_forces()[bodyA.node_index()] += F_AAo_W;
    forces->mutable_body_forces()[bodyB.node_index()] += F_BBo_W;
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcSdfData(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::SdfData* sdf_data) const {
  sdf_data->sdf_pairs.resize(num_steps());
  for (int t = 0; t < num_steps(); ++t) {
    const Context<T>& context = EvalPlantContext(state, t);
    const geometry::QueryObject<T>& query_object =
        plant()
            .get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<T>>(context);
    sdf_data->sdf_pairs[t] =
        query_object.ComputeSignedDistancePairwiseClosestPoints();
  }
  sdf_data->up_to_date = true;
}

template <typename T>
const std::vector<geometry::SignedDistancePair<T>>&
TrajectoryOptimizer<T>::EvalSignedDistancePairs(
    const TrajectoryOptimizerState<T>& state, int t) const {
  DRAKE_DEMAND(0 <= t && t < num_steps());
  if (!state.cache().sdf_data.up_to_date) {
    CalcSdfData(state, &state.mutable_cache().sdf_data);
  }
  return state.cache().sdf_data.sdf_pairs[t];
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContactJacobian(
    const Context<T>& context,
    const std::vector<geometry::SignedDistancePair<T>>& sdf_pairs,
    MatrixX<T>* J, std::vector<math::RotationMatrix<T>>* R_WC,
    std::vector<std::pair<BodyIndex, BodyIndex>>* body_pairs) const {
  const geometry::QueryObject<T>& query_object =
      plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const drake::geometry::SceneGraphInspector<T>& inspector =
      query_object.inspector();

  // TODO(amcastro-tri): consider moving into workspace to avoid heap
  // allocating.
  const int nv = plant().num_velocities();
  Matrix3X<T> Jv_WAc_W(3, nv);
  Matrix3X<T> Jv_WBc_W(3, nv);
  const int nc = sdf_pairs.size();
  J->resize(3 * nc, nv);
  R_WC->resize(nc);
  body_pairs->resize(nc);

  int ic = 0;
  const Frame<T>& frame_W = plant().world_frame();
  for (const SignedDistancePair<T>& pair : sdf_pairs) {
    // Normal outwards from A.
    const Vector3<T> nhat_W = -pair.nhat_BA_W;

    // Get geometry and transformation data for the witness points
    const GeometryId geometryA_id = pair.id_A;
    const GeometryId geometryB_id = pair.id_B;

    const BodyIndex bodyA_index =
        plant().geometry_id_to_body_index().at(geometryA_id);
    const Body<T>& bodyA = plant().get_body(bodyA_index);
    const BodyIndex bodyB_index =
        plant().geometry_id_to_body_index().at(geometryB_id);
    const Body<T>& bodyB = plant().get_body(bodyB_index);

    body_pairs->at(ic) = std::make_pair(bodyA_index, bodyB_index);

    // Body poses in world.
    const math::RigidTransform<T>& X_WA =
        plant().EvalBodyPoseInWorld(context, bodyA);
    const math::RigidTransform<T>& X_WB =
        plant().EvalBodyPoseInWorld(context, bodyB);

    // Geometry poses in body frames.
    const math::RigidTransform<T> X_AGa =
        inspector.GetPoseInFrame(geometryA_id).template cast<T>();
    const math::RigidTransform<T> X_BGb =
        inspector.GetPoseInFrame(geometryB_id).template cast<T>();

    // Position of the witness points in the world frame.
    const auto& p_GaCa_Ga = pair.p_ACa;
    const RigidTransform<T> X_WGa = X_WA * X_AGa;
    const Vector3<T> p_WCa_W = X_WGa * p_GaCa_Ga;
    const auto& p_GbCb_Gb = pair.p_BCb;
    const RigidTransform<T> X_WGb = X_WB * X_BGb;
    const Vector3<T> p_WCb_W = X_WGb * p_GbCb_Gb;

    // We define the (common, unique) contact point C as the midpoint between
    // witness points Ca and Cb.
    const Vector3<T> p_WC = 0.5 * (p_WCa_W + p_WCb_W);

    // Since v_AcBc_W = v_WBc - v_WAc the relative velocity Jacobian will be:
    //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
    // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
    const Vector3<T> p_AoC_A = X_WA.inverse() * p_WC;
    plant().CalcJacobianTranslationalVelocity(
        context, multibody::JacobianWrtVariable::kV, bodyA.body_frame(),
        p_AoC_A, frame_W, frame_W, &Jv_WAc_W);
    const Vector3<T> p_BoC_B = X_WB.inverse() * p_WC;
    plant().CalcJacobianTranslationalVelocity(
        context, multibody::JacobianWrtVariable::kV, bodyB.body_frame(),
        p_BoC_B, frame_W, frame_W, &Jv_WBc_W);

    // Define a contact frame C at the contact point such that the z-axis Cz
    // equals nhat_W. The tangent vectors are arbitrary, with the only
    // requirement being that they form a valid right handed basis with nhat_W.
    R_WC->at(ic) = math::RotationMatrix<T>::MakeFromOneVector(nhat_W, 2);

    // Contact Jacobian J_AcBc_C, expressed in the contact frame C.
    // That is, vc = J * v stores the contact velocities expressed in the
    // contact frame C. Similarly for contact forces, they are expressed in this
    // same frame C.
    J->middleRows(3 * ic, 3) =
        R_WC->at(ic).matrix().transpose() * (Jv_WBc_W - Jv_WAc_W);

    ++ic;
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContactJacobianData(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::ContactJacobianData*
        contact_jacobian_data) const {
  // Resize contact data accordingly.
  // We resize to include all pairs, even for positive distances for which the
  // contact forces will be zero.
  contact_jacobian_data->J.resize(num_steps());
  contact_jacobian_data->R_WC.resize(num_steps());
  contact_jacobian_data->body_pairs.resize(num_steps());

  for (int t = 0; t < num_steps(); ++t) {
    const Context<T>& context = EvalPlantContext(state, t);
    const std::vector<geometry::SignedDistancePair<T>>& sdf_pairs =
        EvalSignedDistancePairs(state, t);
    CalcContactJacobian(context, sdf_pairs, &contact_jacobian_data->J[t],
                        &contact_jacobian_data->R_WC[t],
                        &contact_jacobian_data->body_pairs[t]);
  }
}

template <typename T>
const typename TrajectoryOptimizerCache<T>::ContactJacobianData&
TrajectoryOptimizer<T>::EvalContactJacobianData(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().contact_jacobian_data.up_to_date) {
    CalcContactJacobianData(state,
                            &state.mutable_cache().contact_jacobian_data);
  }
  return state.cache().contact_jacobian_data;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartials(
    const TrajectoryOptimizerState<T>& state,
    InverseDynamicsPartials<T>* id_partials) const {
  INSTRUMENT_FUNCTION("Computes dtau/dq.");
  switch (params_.gradients_method) {
    case GradientsMethod::kForwardDifferences: {
      CalcInverseDynamicsPartialsFiniteDiff(state, id_partials);
      break;
    }
    case GradientsMethod::kCentralDifferences: {
      CalcInverseDynamicsPartialsCentralDiff(state, id_partials);
      break;
    }
    case GradientsMethod::kCentralDifferences4: {
      // N.B. this function uses either 2nd or 4th order central differences,
      // depending on the value of params_.gradients_method
      CalcInverseDynamicsPartialsCentralDiff(state, id_partials);
      break;
    }
    case GradientsMethod::kAutoDiff: {
      if constexpr (std::is_same_v<T, double>) {
        CalcInverseDynamicsPartialsAutoDiff(state, id_partials);
      } else {
        throw std::runtime_error(
            "Analytical gradients not supported for "
            "TrajectoryOptimizer<AutoDiffXd>.");
      }
      break;
    }
    case GradientsMethod::kNoGradients: {
      throw std::runtime_error(
          "This optimizer was instantiated with GradientsMethod::kNoGradients "
          "and therefore the computation of gradients is not enabled.");
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsFiniteDiff(
    const TrajectoryOptimizerState<T>& state,
    InverseDynamicsPartials<T>* id_partials) const {
  using std::abs;
  using std::max;
  // Check that id_partials has been allocated correctly.
  DRAKE_DEMAND(id_partials->size() == num_steps());

  // Get the trajectory data
  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& a = EvalA(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);

  // Get references to the partials that we'll be setting
  std::vector<MatrixX<T>>& dtau_dqm = id_partials->dtau_dqm;
  std::vector<MatrixX<T>>& dtau_dqt = id_partials->dtau_dqt;
  std::vector<MatrixX<T>>& dtau_dqp = id_partials->dtau_dqp;

  // Get kinematic mapping matrices for each time step
  const std::vector<MatrixX<T>>& Nplus = EvalNplus(state);

  // Allocate small perturbations to q, v, and a at each time step
  const double eps = sqrt(std::numeric_limits<double>::epsilon());
  std::vector<T> dq_is(num_steps() + 1);
  std::vector<T> dv_is(num_steps() + 1);
  std::vector<T> da_is(num_steps() + 1);

#if defined(_OPENMP)
#pragma omp parallel for num_threads(params_.num_threads)
#endif
  for (int t = 1; t <= num_steps(); ++t) {
    // N.B. A perturbation of qt propagates to tau[t-1], tau[t] and tau[t+1].
    // Therefore we compute one column of grad_tau at a time. That is, once the
    // loop on position indices i is over, we effectively computed the t-th
    // column of grad_tau.

    // N.B. we need a separate workspace for each timestep, otherwise threads
    // will fight over the same workspace
    TrajectoryOptimizerWorkspace<T>& workspace =
        state.per_timestep_workspace[t];

    // Get references to perturbed versions of q, v, tau, and a, at (t-1, t, t).
    // These are all of the quantities that change when we perturb q_t.
    VectorX<T>& q_eps_t = workspace.q_size_tmp1;
    VectorX<T>& v_eps_t = workspace.v_size_tmp1;
    VectorX<T>& v_eps_tp = workspace.v_size_tmp2;
    VectorX<T>& a_eps_tm = workspace.a_size_tmp1;
    VectorX<T>& a_eps_t = workspace.a_size_tmp2;
    VectorX<T>& tau_eps_tm = workspace.tau_size_tmp1;
    VectorX<T>& tau_eps_t = workspace.tau_size_tmp2;

    // Mass matrix, for analytical computation of dtau[t+1]/dq[t]
    MatrixX<T>& M = workspace.mass_matrix_size_tmp;

    // Small perturbations
    T& dq_i = dq_is[t];
    T& dv_i = dv_is[t];
    T& da_i = da_is[t];

    // Set perturbed versions of variables
    q_eps_t = q[t];
    v_eps_t = v[t];
    if (t < num_steps()) {
      // v[num_steps + 1] is not defined
      v_eps_tp = v[t + 1];
      // a[num_steps] is not defined
      a_eps_t = a[t];
    }
    a_eps_tm = a[t - 1];

    // Get a context for this time step
    Context<T>& context_t = GetMutablePlantContext(state, t);

    for (int i = 0; i < plant().num_positions(); ++i) {
      // Determine perturbation sizes to avoid losing precision to floating
      // point error
      dq_i = eps * max(1.0, abs(q_eps_t(i)));

      // Make dqt_i exactly representable to minimize floating point error
      const T temp = q_eps_t(i) + dq_i;
      dq_i = temp - q_eps_t(i);

      dv_i = dq_i / time_step();
      da_i = dv_i / time_step();

      // Perturb q_t[i], v_t[i], and a_t[i]
      q_eps_t(i) += dq_i;

      v_eps_t += dv_i * Nplus[t].col(i);
      a_eps_tm += da_i * Nplus[t].col(i);
      if (t < num_steps()) {
        v_eps_tp -= dv_i * Nplus[t + 1].col(i);
        a_eps_t -= da_i * (Nplus[t + 1].col(i) + Nplus[t].col(i));
      }

      // Compute perturbed tau(q) and calculate the nonzero entries of dtau/dq
      // via finite differencing

      // tau[t-1] = ID(q[t], v[t], a[t-1])
      plant().SetPositions(&context_t, q_eps_t);
      plant().SetVelocities(&context_t, v_eps_t);
      CalcInverseDynamicsSingleTimeStep(context_t, a_eps_tm, &workspace,
                                        &tau_eps_tm);
      dtau_dqp[t - 1].col(i) = (tau_eps_tm - tau[t - 1]) / dq_i;

      // tau[t] = ID(q[t+1], v[t+1], a[t])
      if (t < num_steps()) {
        plant().SetPositions(&context_t, q[t + 1]);
        plant().SetVelocities(&context_t, v_eps_tp);
        CalcInverseDynamicsSingleTimeStep(context_t, a_eps_t, &workspace,
                                          &tau_eps_t);
        dtau_dqt[t].col(i) = (tau_eps_t - tau[t]) / dq_i;
      }

      // Unperturb q_t[i], v_t[i], and a_t[i]
      q_eps_t = q[t];
      v_eps_t = v[t];
      a_eps_tm = a[t - 1];
      if (t < num_steps()) {
        v_eps_tp = v[t + 1];
        a_eps_t = a[t];
      }
    }

    // tau[t+1] = ID(q[t+2], v[t+2], a[t+1])
    // N.B. Since tau[t+1] depends on q only through a, we can compute
    // dtau_dqm[t+1] analytically rather than relying on column-wise forward
    // differences
    if (t < num_steps() - 1) {
      plant().SetPositions(&context_t, q[t + 2]);
      plant().SetVelocities(&context_t, v[t + 2]);
      plant().CalcMassMatrix(context_t, &M);
      dtau_dqm[t + 1] = 1 / time_step() / time_step() * M * Nplus[t + 1];
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsCentralDiff(
    const TrajectoryOptimizerState<T>& state,
    InverseDynamicsPartials<T>* id_partials) const {
  // Check that id_partials has been allocated correctly.
  DRAKE_DEMAND(id_partials->size() == num_steps());

  // Get references to the partials that we'll be setting
  std::vector<MatrixX<T>>& dtau_dqm = id_partials->dtau_dqm;
  std::vector<MatrixX<T>>& dtau_dqt = id_partials->dtau_dqt;
  std::vector<MatrixX<T>>& dtau_dqp = id_partials->dtau_dqp;

  for (int t = 1; t <= num_steps(); ++t) {
    // N.B. A perturbation of qt propagates to tau[t-1], tau[t] and tau[t+1].
    // Therefore we compute one column of grad_tau at a time. That is, once the
    // loop on position indices i is over, we effectively computed the t-th
    // column of grad_tau.

    // Compute derivatives of tau[t-1], tau[t] and tau[t+1] w.r.t. q[t].
    MatrixX<T>* dtaup_dqt = t < num_steps() - 1 ? &dtau_dqm[t + 1] : nullptr;
    MatrixX<T>* dtaut_dqt = t < num_steps() ? &dtau_dqt[t] : nullptr;
    MatrixX<T>* dtaum_dqt = t > 0 ? &dtau_dqp[t - 1] : nullptr;
    CalcInverseDynamicsPartialsWrtQtCentralDiff(t, state, dtaum_dqt, dtaut_dqt,
                                                dtaup_dqt);
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsWrtQtCentralDiff(
    int t, const TrajectoryOptimizerState<T>& state, MatrixX<T>* dtaum_dqt,
    MatrixX<T>* dtaut_dqt, MatrixX<T>* dtaup_dqt) const {
  using std::abs;
  using std::max;

  if (t < num_steps() - 1) DRAKE_DEMAND(dtaup_dqt != nullptr);
  if (t < num_steps()) DRAKE_DEMAND(dtaut_dqt != nullptr);
  if (t > 0) DRAKE_DEMAND(dtaum_dqt != nullptr);

  // Flag indicating whether we're doing 4th order central differences. If
  // false, we just do regular 2nd order central differences
  bool fourth_order =
      (params_.gradients_method == GradientsMethod::kCentralDifferences4);

  // Get kinematic mapping matrices for each time step
  const std::vector<MatrixX<T>>& Nplus = EvalNplus(state);

  TrajectoryOptimizerWorkspace<T>& workspace = state.workspace;

  // Get references to perturbed versions of q, v, tau, and a, at (t-1, t, t).
  // These are all of the quantities that change when we perturb q_t.
  VectorX<T>& q_emm_t = workspace.q_size_tmp1;
  VectorX<T>& q_em_t = workspace.q_size_tmp2;
  VectorX<T>& q_ep_t = workspace.q_size_tmp3;
  VectorX<T>& q_epp_t = workspace.q_size_tmp4;

  VectorX<T>& v_emm_t = workspace.v_size_tmp1;
  VectorX<T>& v_em_t = workspace.v_size_tmp2;
  VectorX<T>& v_ep_t = workspace.v_size_tmp3;
  VectorX<T>& v_epp_t = workspace.v_size_tmp4;
  VectorX<T>& v_emm_tp = workspace.v_size_tmp5;
  VectorX<T>& v_em_tp = workspace.v_size_tmp6;
  VectorX<T>& v_ep_tp = workspace.v_size_tmp7;
  VectorX<T>& v_epp_tp = workspace.v_size_tmp8;

  VectorX<T>& a_emm_tm = workspace.a_size_tmp1;
  VectorX<T>& a_em_tm = workspace.a_size_tmp2;
  VectorX<T>& a_ep_tm = workspace.a_size_tmp3;
  VectorX<T>& a_epp_tm = workspace.a_size_tmp4;
  VectorX<T>& a_emm_t = workspace.a_size_tmp5;
  VectorX<T>& a_em_t = workspace.a_size_tmp6;
  VectorX<T>& a_ep_t = workspace.a_size_tmp7;
  VectorX<T>& a_epp_t = workspace.a_size_tmp8;
  VectorX<T>& a_emm_tp = workspace.a_size_tmp9;
  VectorX<T>& a_em_tp = workspace.a_size_tmp10;
  VectorX<T>& a_ep_tp = workspace.a_size_tmp11;
  VectorX<T>& a_epp_tp = workspace.a_size_tmp12;

  // TODO(vincekurtz): we only need a single set of {tau_emm, tau_m, tau_ep,
  // tau_epp}, regardless of time step (so no distinction for _tm, _t, _tp).
  // Then we can reuse them below since they get overriden.
  VectorX<T>& tau_emm_tm = workspace.tau_size_tmp1;
  VectorX<T>& tau_em_tm = workspace.tau_size_tmp2;
  VectorX<T>& tau_ep_tm = workspace.tau_size_tmp3;
  VectorX<T>& tau_epp_tm = workspace.tau_size_tmp4;
  VectorX<T>& tau_emm_t = workspace.tau_size_tmp5;
  VectorX<T>& tau_em_t = workspace.tau_size_tmp6;
  VectorX<T>& tau_ep_t = workspace.tau_size_tmp7;
  VectorX<T>& tau_epp_t = workspace.tau_size_tmp8;
  VectorX<T>& tau_emm_tp = workspace.tau_size_tmp9;
  VectorX<T>& tau_em_tp = workspace.tau_size_tmp10;
  VectorX<T>& tau_ep_tp = workspace.tau_size_tmp11;
  VectorX<T>& tau_epp_tp = workspace.tau_size_tmp12;

  // Get the trajectory data
  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& a = EvalA(state);

  // Set perturbed versions of variables
  q_emm_t = q[t];
  q_em_t = q[t];
  q_ep_t = q[t];
  q_epp_t = q[t];
  v_emm_t = v[t];
  v_em_t = v[t];
  v_ep_t = v[t];
  v_epp_t = v[t];
  if (t > 0) {
    // a[-1] is undefined
    a_emm_tm = a[t - 1];
    a_em_tm = a_emm_tm;
    a_ep_tm = a_emm_tm;
    a_epp_tm = a_emm_tm;
  }
  if (t < num_steps()) {
    // v[num_steps + 1] is not defined
    v_emm_tp = v[t + 1];
    v_em_tp = v_emm_tp;
    v_ep_tp = v_emm_tp;
    v_epp_tp = v_emm_tp;
    // a[num_steps] is not defined
    a_emm_t = a[t];
    a_em_t = a_emm_t;
    a_ep_t = a_emm_t;
    a_epp_t = a_emm_t;
  }
  if (t < num_steps() - 1) {
    // a[num_steps + 1] is not defined
    a_emm_tp = a[t + 1];
    a_em_tp = a_emm_tp;
    a_ep_tp = a_emm_tp;
    a_epp_tp = a_emm_tp;
  }

  // Compute small perturbations
  // N.B. the theoretically optimal step size of eps^(1/3) appears to give worse
  // performance than a smaller step size of eps^(1/2), possibly because the
  // curvature scale (x_c) of problems with contact may be significantly smaller
  // than the standard approximation of x_c = max(1,x).
  const double eps = sqrt(std::numeric_limits<double>::epsilon());

  for (int i = 0; i < plant().num_positions(); ++i) {
    // Determine perturbation sizes to avoid losing precision to floating
    // point error
    T dq = eps * max(1.0, abs(q_ep_t(i)));  // N.B. q_em_t(i) == q_ep_t(i)

    // Make dq exactly representable to minimize floating point error
    T temp = q_ep_t(i) + dq;
    dq = temp - q_ep_t(i);

    const T dv = dq / time_step();
    const T da = dv / time_step();

    // Perturb q_t[i], v_t[i], and a_t[i]
    q_em_t(i) -= dq;
    q_ep_t(i) += dq;
    if (fourth_order) {
      q_emm_t(i) -= 2.0 * dq;
      q_epp_t(i) += 2.0 * dq;
    }

    if (t > 0) {
      v_em_t -= dv * Nplus[t].col(i);
      v_ep_t += dv * Nplus[t].col(i);
      a_em_tm -= da * Nplus[t].col(i);
      a_ep_tm += da * Nplus[t].col(i);
      if (fourth_order) {
        v_emm_t -= 2.0 * dv * Nplus[t].col(i);
        v_epp_t += 2.0 * dv * Nplus[t].col(i);
        a_emm_tm -= 2.0 * da * Nplus[t].col(i);
        a_epp_tm += 2.0 * da * Nplus[t].col(i);
      }
    }

    if (t < num_steps()) {
      v_em_tp += dv * Nplus[t + 1].col(i);
      v_ep_tp -= dv * Nplus[t + 1].col(i);
      a_em_t += da * (Nplus[t + 1].col(i) + Nplus[t].col(i));
      a_ep_t -= da * (Nplus[t + 1].col(i) + Nplus[t].col(i));
      if (fourth_order) {
        v_emm_tp += 2.0 * dv * Nplus[t + 1].col(i);
        v_epp_tp -= 2.0 * dv * Nplus[t + 1].col(i);
        a_emm_t += 2.0 * da * (Nplus[t + 1].col(i) + Nplus[t].col(i));
        a_epp_t -= 2.0 * da * (Nplus[t + 1].col(i) + Nplus[t].col(i));
      }
    }

    if (t < num_steps() - 1) {
      a_em_tp -= da * Nplus[t + 1].col(i);
      a_ep_tp += da * Nplus[t + 1].col(i);
      if (fourth_order) {
        a_emm_tp -= 2.0 * da * Nplus[t + 1].col(i);
        a_epp_tp += 2.0 * da * Nplus[t + 1].col(i);
      }
    }

    // Compute perturbed tau(q) and calculate the nonzero entries of dtau/dq
    // via finite differencing
    if (t > 0) {
      // tau[t-1] = ID(q[t], v[t], a[t-1])
      plant().SetPositions(context_, q_em_t);
      plant().SetVelocities(context_, v_em_t);
      CalcInverseDynamicsSingleTimeStep(*context_, a_em_tm, &workspace,
                                        &tau_em_tm);
      plant().SetPositions(context_, q_ep_t);
      plant().SetVelocities(context_, v_ep_t);
      CalcInverseDynamicsSingleTimeStep(*context_, a_ep_tm, &workspace,
                                        &tau_ep_tm);
      if (fourth_order) {
        plant().SetPositions(context_, q_emm_t);
        plant().SetVelocities(context_, v_emm_t);
        CalcInverseDynamicsSingleTimeStep(*context_, a_emm_tm, &workspace,
                                          &tau_emm_tm);
        plant().SetPositions(context_, q_epp_t);
        plant().SetVelocities(context_, v_epp_t);
        CalcInverseDynamicsSingleTimeStep(*context_, a_epp_tm, &workspace,
                                          &tau_epp_tm);
        dtaum_dqt->col(i) = 2.0 / 3.0 * (tau_ep_tm - tau_em_tm) / dq -
                            1.0 / 12.0 * (tau_epp_tm - tau_emm_tm) / dq;
      } else {
        dtaum_dqt->col(i) = 0.5 * (tau_ep_tm - tau_em_tm) / dq;
      }
    }
    if (t < num_steps()) {
      // tau[t] = ID(q[t+1], v[t+1], a[t])
      plant().SetPositions(context_, q[t + 1]);
      plant().SetVelocities(context_, v_ep_tp);
      CalcInverseDynamicsSingleTimeStep(*context_, a_ep_t, &workspace,
                                        &tau_ep_t);

      plant().SetPositions(context_, q[t + 1]);
      plant().SetVelocities(context_, v_em_tp);
      CalcInverseDynamicsSingleTimeStep(*context_, a_em_t, &workspace,
                                        &tau_em_t);
      if (fourth_order) {
        plant().SetPositions(context_, q[t + 1]);
        plant().SetVelocities(context_, v_epp_tp);
        CalcInverseDynamicsSingleTimeStep(*context_, a_epp_t, &workspace,
                                          &tau_epp_t);

        plant().SetPositions(context_, q[t + 1]);
        plant().SetVelocities(context_, v_emm_tp);
        CalcInverseDynamicsSingleTimeStep(*context_, a_emm_t, &workspace,
                                          &tau_emm_t);
        dtaut_dqt->col(i) = 2.0 / 3.0 * (tau_ep_t - tau_em_t) / dq -
                            1.0 / 12.0 * (tau_epp_t - tau_emm_t) / dq;
      } else {
        dtaut_dqt->col(i) = 0.5 * (tau_ep_t - tau_em_t) / dq;
      }
    }
    if (t < num_steps() - 1) {
      // tau[t+1] = ID(q[t+2], v[t+2], a[t+1])
      plant().SetPositions(context_, q[t + 2]);
      plant().SetVelocities(context_, v[t + 2]);
      CalcInverseDynamicsSingleTimeStep(*context_, a_em_tp, &workspace,
                                        &tau_em_tp);
      plant().SetPositions(context_, q[t + 2]);
      plant().SetVelocities(context_, v[t + 2]);
      CalcInverseDynamicsSingleTimeStep(*context_, a_ep_tp, &workspace,
                                        &tau_ep_tp);
      if (fourth_order) {
        plant().SetPositions(context_, q[t + 2]);
        plant().SetVelocities(context_, v[t + 2]);
        CalcInverseDynamicsSingleTimeStep(*context_, a_emm_tp, &workspace,
                                          &tau_emm_tp);
        plant().SetPositions(context_, q[t + 2]);
        plant().SetVelocities(context_, v[t + 2]);
        CalcInverseDynamicsSingleTimeStep(*context_, a_epp_tp, &workspace,
                                          &tau_epp_tp);
        dtaup_dqt->col(i) = 2.0 / 3.0 * (tau_ep_tp - tau_em_tp) / dq -
                            1.0 / 12.0 * (tau_epp_tp - tau_emm_tp) / dq;
      } else {
        dtaup_dqt->col(i) = 0.5 * (tau_ep_tp - tau_em_tp) / dq;
      }
    }

    // Unperturb q_t[i], v_t[i], and a_t[i]
    q_em_t = q[t];
    q_ep_t = q[t];
    v_em_t = v[t];
    v_ep_t = v[t];
    if (fourth_order) {
      q_emm_t = q[t];
      q_epp_t = q[t];
      v_emm_t = v[t];
      v_epp_t = v[t];
    }

    if (t < num_steps()) {
      v_em_tp = v[t + 1];
      v_ep_tp = v[t + 1];
      a_em_t = a[t];
      a_ep_t = a[t];

      if (fourth_order) {
        v_emm_tp = v[t + 1];
        v_epp_tp = v[t + 1];
        a_emm_t = a[t];
        a_epp_t = a[t];
      }
    }
    if (t < num_steps() - 1) {
      a_em_tp = a[t + 1];
      a_ep_tp = a[t + 1];

      if (fourth_order) {
        a_emm_tp = a[t + 1];
        a_epp_tp = a[t + 1];
      }
    }
    if (t > 0) {
      a_em_tm = a[t - 1];
      a_ep_tm = a[t - 1];

      if (fourth_order) {
        a_emm_tm = a[t - 1];
        a_epp_tm = a[t - 1];
      }
    }
  }
}

template <>
void TrajectoryOptimizer<AutoDiffXd>::CalcInverseDynamicsPartialsAutoDiff(
    const TrajectoryOptimizerState<double>&,
    InverseDynamicsPartials<double>*) const {}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsPartialsAutoDiff(
    const TrajectoryOptimizerState<double>& state,
    InverseDynamicsPartials<double>* id_partials) const {
  DRAKE_DEMAND(id_partials->size() == num_steps());

  // Get references to the partials that we'll be setting
  std::vector<MatrixX<double>>& dtau_dqm = id_partials->dtau_dqm;
  std::vector<MatrixX<double>>& dtau_dqt = id_partials->dtau_dqt;
  std::vector<MatrixX<double>>& dtau_dqp = id_partials->dtau_dqp;

  // Heap allocations.
  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps() + 1);
  VectorX<AutoDiffXd> tau_ad(plant().num_velocities());
  TrajectoryOptimizerWorkspace<AutoDiffXd> workspace_ad(num_steps(),
                                                        *plant_ad_);

  // Initialize q_ad. First with no derivatives, as a constant.
  const std::vector<VectorX<double>>& q = state.q();
  for (int t = 0; t <= num_steps(); ++t) {
    q_ad[t].resize(q[t].size());
    q_ad[t] = q[t];
  }

  // At each t we will compute derivatives of tau[t-1], tau[t] and tau[t+1 with
  // respect to q[t].
  for (int t = 1; t <= num_steps(); ++t) {
    // Set derivatives with respect to q[t].
    // q[t] will propagate directly to v[t], v[t+1], a[t-1], a[t] and a[t+1].
    q_ad[t] = math::InitializeAutoDiff(q[t]);
    state_ad_->set_q(q_ad);

    // N.B. All dynamics terms are treated implicitly, i.e.,
    // tau[t] = M(q[t+1]) * a[t] - k(q[t+1],v[t+1]) - f_ext[t+1]

    const std::vector<VectorX<AutoDiffXd>>& a_ad =
        optimizer_ad_->EvalA(*state_ad_);

    // dtau_dqt[t].
    if (t < num_steps()) {
      const Context<AutoDiffXd>& context_ad_tp =
          optimizer_ad_->EvalPlantContext(*state_ad_, t + 1);
      optimizer_ad_->CalcInverseDynamicsSingleTimeStep(context_ad_tp, a_ad[t],
                                                       &workspace_ad, &tau_ad);
      dtau_dqt[t] = math::ExtractGradient(tau_ad);
    }

    // dtau_dqt[t+1].
    if (t < num_steps() - 1) {
      const Context<AutoDiffXd>& context_ad_tpp =
          optimizer_ad_->EvalPlantContext(*state_ad_, t + 2);
      optimizer_ad_->CalcInverseDynamicsSingleTimeStep(
          context_ad_tpp, a_ad[t + 1], &workspace_ad, &tau_ad);
      dtau_dqm[t + 1] = math::ExtractGradient(tau_ad);
    }

    // dtau_dqt[t-1].
    if (t > 0) {
      const Context<AutoDiffXd>& context_ad_t =
          optimizer_ad_->EvalPlantContext(*state_ad_, t);
      optimizer_ad_->CalcInverseDynamicsSingleTimeStep(
          context_ad_t, a_ad[t - 1], &workspace_ad, &tau_ad);
      dtau_dqp[t - 1] = math::ExtractGradient(tau_ad);
    }

    // Unset derivatives.
    q_ad[t] = q[t];
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcVelocityPartials(
    const TrajectoryOptimizerState<T>& state,
    VelocityPartials<T>* v_partials) const {
  const std::vector<MatrixX<T>>& Nplus = EvalNplus(state);
  for (int t = 0; t <= num_steps(); ++t) {
    v_partials->dvt_dqt[t] = 1 / time_step() * Nplus[t];
    if (t > 0) {
      v_partials->dvt_dqm[t] = -1 / time_step() * Nplus[t];
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcGradientFiniteDiff(
    const TrajectoryOptimizerState<T>& state, EigenPtr<VectorX<T>> g) const {
  using std::abs;
  using std::max;

  // Perturbed versions of q
  std::vector<VectorX<T>>& q_plus = state.workspace.q_sequence_tmp1;
  std::vector<VectorX<T>>& q_minus = state.workspace.q_sequence_tmp2;
  q_plus = state.q();
  q_minus = state.q();

  // non-constant copy of state that we can perturb
  TrajectoryOptimizerState<T> state_eps = CreateState();

  // Set first block of g (derivatives w.r.t. q_0) to zero, since q0 = q_init
  // are constant.
  g->topRows(plant().num_positions()).setZero();

  // Iterate through rows of g using finite differences
  const double eps = cbrt(std::numeric_limits<double>::epsilon());
  T dqt_i;
  int j = plant().num_positions();
  for (int t = 1; t <= num_steps(); ++t) {
    for (int i = 0; i < plant().num_positions(); ++i) {
      // Set finite difference step size
      dqt_i = eps * max(1.0, abs(state.q()[t](i)));
      q_plus[t](i) += dqt_i;
      q_minus[t](i) -= dqt_i;

      // Set g_j = using central differences
      state_eps.set_q(q_plus);
      T L_plus = CalcCost(state_eps);
      state_eps.set_q(q_minus);
      T L_minus = CalcCost(state_eps);
      (*g)(j) = (L_plus - L_minus) / (2 * dqt_i);

      // reset our perturbed Q and move to the next row of g.
      q_plus[t](i) = state.q()[t](i);
      q_minus[t](i) = state.q()[t](i);
      ++j;
    }
  }
}

template <typename T>
void TrajectoryOptimizer<T>::CalcGradient(
    const TrajectoryOptimizerState<T>& state, EigenPtr<VectorX<T>> g) const {
  INSTRUMENT_FUNCTION("Assembly of the gradient.");
  const double dt = time_step();
  const int nq = plant().num_positions();
  TrajectoryOptimizerWorkspace<T>* workspace = &state.workspace;

  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  const std::vector<VectorX<T>>& tau = EvalTau(state);

  const VelocityPartials<T>& v_partials = EvalVelocityPartials(state);
  const InverseDynamicsPartials<T>& id_partials =
      EvalInverseDynamicsPartials(state);
  const std::vector<MatrixX<T>>& dvt_dqt = v_partials.dvt_dqt;
  const std::vector<MatrixX<T>>& dvt_dqm = v_partials.dvt_dqm;
  const std::vector<MatrixX<T>>& dtau_dqp = id_partials.dtau_dqp;
  const std::vector<MatrixX<T>>& dtau_dqt = id_partials.dtau_dqt;
  const std::vector<MatrixX<T>>& dtau_dqm = id_partials.dtau_dqm;

  // Set first block of g (derivatives w.r.t. q_0) to zero, since q0 = q_init
  // are constant.
  g->topRows(plant().num_positions()).setZero();

  // Scratch variables for storing intermediate cost terms
  VectorX<T>& qt_term = workspace->q_size_tmp1;
  VectorX<T>& vt_term = workspace->v_size_tmp1;
  VectorX<T>& vp_term = workspace->v_size_tmp2;
  VectorX<T>& taum_term = workspace->tau_size_tmp1;
  VectorX<T>& taut_term = workspace->tau_size_tmp2;
  VectorX<T>& taup_term = workspace->tau_size_tmp3;

  for (int t = 1; t < num_steps(); ++t) {
    // Contribution from position cost
    qt_term = (q[t] - prob_.q_nom[t]).transpose() * 2 * prob_.Qq * dt;

    // Contribution from velocity cost
    vt_term =
        (v[t] - prob_.v_nom[t]).transpose() * 2 * prob_.Qv * dt * dvt_dqt[t];
    if (t == num_steps() - 1) {
      // The terminal cost needs to be handled differently
      vp_term = (v[t + 1] - prob_.v_nom[t + 1]).transpose() * 2 * prob_.Qf_v *
                dvt_dqm[t + 1];
    } else {
      vp_term = (v[t + 1] - prob_.v_nom[t + 1]).transpose() * 2 * prob_.Qv *
                dt * dvt_dqm[t + 1];
    }

    // Contribution from control cost
    taum_term = tau[t - 1].transpose() * 2 * prob_.R * dt * dtau_dqp[t - 1];
    taut_term = tau[t].transpose() * 2 * prob_.R * dt * dtau_dqt[t];
    if (t == num_steps() - 1) {
      // There is no constrol input at the final timestep
      taup_term.setZero(nq);
    } else {
      taup_term = tau[t + 1].transpose() * 2 * prob_.R * dt * dtau_dqm[t + 1];
    }

    // Put it all together to get the gradient w.r.t q[t]
    g->segment(t * nq, nq) =
        qt_term + vt_term + vp_term + taum_term + taut_term + taup_term;
  }

  // Last step is different, because there is terminal cost and v[t+1] doesn't
  // exist
  taum_term = tau[num_steps() - 1].transpose() * 2 * prob_.R * dt *
              dtau_dqp[num_steps() - 1];
  qt_term =
      (q[num_steps()] - prob_.q_nom[num_steps()]).transpose() * 2 * prob_.Qf_q;
  vt_term = (v[num_steps()] - prob_.v_nom[num_steps()]).transpose() * 2 *
            prob_.Qf_v * dvt_dqt[num_steps()];
  g->tail(nq) = qt_term + vt_term + taum_term;

  // Add proximal operator term to the gradient, if requested
  if (params_.proximal_operator) {
    const std::vector<VectorX<T>>& q_last =
        state.proximal_operator_data().q_last;
    const std::vector<VectorX<T>>& H_diag =
        state.proximal_operator_data().H_diag;
    for (int t = 0; t <= num_steps(); ++t) {
      g->segment(t * nq, nq) +=
          params_.rho_proximal * H_diag[t].asDiagonal() * (q[t] - q_last[t]);
    }
  }
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalGradient(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().gradient_up_to_date) {
    CalcGradient(state, &state.mutable_cache().gradient);
    state.mutable_cache().gradient_up_to_date = true;
  }
  return state.cache().gradient;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcHessian(
    const TrajectoryOptimizerState<T>& state, PentaDiagonalMatrix<T>* H) const {
  DRAKE_DEMAND(H->is_symmetric());
  DRAKE_DEMAND(H->block_rows() == num_steps() + 1);
  DRAKE_DEMAND(H->block_size() == plant().num_positions());
  INSTRUMENT_FUNCTION("Assembly of the Hessian.");

  // Some convienient aliases
  const double dt = time_step();
  const MatrixX<T> Qq = 2 * prob_.Qq * dt;
  const MatrixX<T> Qv = 2 * prob_.Qv * dt;
  const MatrixX<T> R = 2 * prob_.R * dt;
  const MatrixX<T> Qf_q = 2 * prob_.Qf_q;
  const MatrixX<T> Qf_v = 2 * prob_.Qf_v;

  const VelocityPartials<T>& v_partials = EvalVelocityPartials(state);
  const InverseDynamicsPartials<T>& id_partials =
      EvalInverseDynamicsPartials(state);
  const std::vector<MatrixX<T>>& dvt_dqt = v_partials.dvt_dqt;
  const std::vector<MatrixX<T>>& dvt_dqm = v_partials.dvt_dqm;
  const std::vector<MatrixX<T>>& dtau_dqp = id_partials.dtau_dqp;
  const std::vector<MatrixX<T>>& dtau_dqt = id_partials.dtau_dqt;
  const std::vector<MatrixX<T>>& dtau_dqm = id_partials.dtau_dqm;

  // Get mutable references to the non-zero bands of the Hessian
  std::vector<MatrixX<T>>& A = H->mutable_A();  // 2 rows below diagonal
  std::vector<MatrixX<T>>& B = H->mutable_B();  // 1 row below diagonal
  std::vector<MatrixX<T>>& C = H->mutable_C();  // diagonal

  // Fill in the non-zero blocks
  C[0].setIdentity();  // Initial condition q0 fixed at t=0
  for (int t = 1; t < num_steps(); ++t) {
    // dg_t/dq_t
    MatrixX<T>& dgt_dqt = C[t];
    dgt_dqt = Qq;
    dgt_dqt += dvt_dqt[t].transpose() * Qv * dvt_dqt[t];
    dgt_dqt += dtau_dqp[t - 1].transpose() * R * dtau_dqp[t - 1];
    dgt_dqt += dtau_dqt[t].transpose() * R * dtau_dqt[t];
    if (t < num_steps() - 1) {
      dgt_dqt += dtau_dqm[t + 1].transpose() * R * dtau_dqm[t + 1];
      dgt_dqt += dvt_dqm[t + 1].transpose() * Qv * dvt_dqm[t + 1];
    } else {
      dgt_dqt += dvt_dqm[t + 1].transpose() * Qf_v * dvt_dqm[t + 1];
    }

    // dg_t/dq_{t+1}
    MatrixX<T>& dgt_dqp = B[t + 1];
    dgt_dqp = dtau_dqp[t].transpose() * R * dtau_dqt[t];
    if (t < num_steps() - 1) {
      dgt_dqp += dtau_dqt[t + 1].transpose() * R * dtau_dqm[t + 1];
      dgt_dqp += dvt_dqt[t + 1].transpose() * Qv * dvt_dqm[t + 1];
    } else {
      dgt_dqp += dvt_dqt[t + 1].transpose() * Qf_v * dvt_dqm[t + 1];
    }

    // dg_t/dq_{t+2}
    if (t < num_steps() - 1) {
      MatrixX<T>& dgt_dqpp = A[t + 2];
      dgt_dqpp = dtau_dqp[t + 1].transpose() * R * dtau_dqm[t + 1];
    }
  }

  // dg_t/dq_t for the final timestep
  MatrixX<T>& dgT_dqT = C[num_steps()];
  dgT_dqT = Qf_q;
  dgT_dqT += dvt_dqt[num_steps()].transpose() * Qf_v * dvt_dqt[num_steps()];
  dgT_dqT +=
      dtau_dqp[num_steps() - 1].transpose() * R * dtau_dqp[num_steps() - 1];

  // Add proximal operator terms to the Hessian, if requested
  if (params_.proximal_operator) {
    for (int t = 0; t <= num_steps(); ++t) {
      C[t] += params_.rho_proximal *
              state.proximal_operator_data().H_diag[t].asDiagonal();
    }
  }

  // Copy lower triangular part to upper triangular part
  H->MakeSymmetric();
}

template <typename T>
const PentaDiagonalMatrix<T>& TrajectoryOptimizer<T>::EvalHessian(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().hessian_up_to_date) {
    if (params_.exact_hessian) {
      CalcExactHessian(state, &state.mutable_cache().hessian);
    } else {
      CalcHessian(state, &state.mutable_cache().hessian);
    }
    state.mutable_cache().hessian_up_to_date = true;
  }
  return state.cache().hessian;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcScaledHessian(
    const TrajectoryOptimizerState<T>& state,
    PentaDiagonalMatrix<T>* Htilde) const {
  const PentaDiagonalMatrix<T>& H = EvalHessian(state);
  const VectorX<T>& D = EvalScaleFactors(state);
  *Htilde = PentaDiagonalMatrix<T>(H);
  Htilde->ScaleByDiagonal(D);
}

template <typename T>
const PentaDiagonalMatrix<T>& TrajectoryOptimizer<T>::EvalScaledHessian(
    const TrajectoryOptimizerState<T>& state) const {
  // Early exit if we're not using scaling
  if (!params_.scaling) return EvalHessian(state);

  if (!state.cache().scaled_hessian_up_to_date) {
    CalcScaledHessian(state, &state.mutable_cache().scaled_hessian);
    state.mutable_cache().scaled_hessian_up_to_date = true;
  }
  return state.cache().scaled_hessian;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcScaledGradient(
    const TrajectoryOptimizerState<T>& state, VectorX<T>* gtilde) const {
  const VectorX<T>& g = EvalGradient(state);
  const VectorX<T>& D = EvalScaleFactors(state);
  *gtilde = D.asDiagonal() * g;
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalScaledGradient(
    const TrajectoryOptimizerState<T>& state) const {
  // Early exit if we're not using scaling
  if (!params_.scaling) return EvalGradient(state);

  if (!state.cache().scaled_gradient_up_to_date) {
    CalcScaledGradient(state, &state.mutable_cache().scaled_gradient);
    state.mutable_cache().scaled_gradient_up_to_date = true;
  }
  return state.cache().scaled_gradient;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcScaleFactors(
    const TrajectoryOptimizerState<T>& state, VectorX<T>* D) const {
  using std::min;
  using std::sqrt;

  const PentaDiagonalMatrix<T>& H = EvalHessian(state);
  VectorX<T>& hessian_diag = state.workspace.num_vars_size_tmp1;
  H.ExtractDiagonal(&hessian_diag);

  for (int i = 0; i < D->size(); ++i) {
    switch (params_.scaling_method) {
      case ScalingMethod::kSqrt: {
        (*D)[i] = min(1.0, 1 / sqrt(hessian_diag[i]));
        break;
      }
      case ScalingMethod::kAdaptiveSqrt: {
        (*D)[i] = min((*D)[i], 1 / sqrt(hessian_diag[i]));
        break;
      }
      case ScalingMethod::kDoubleSqrt: {
        (*D)[i] = min(1.0, 1 / sqrt(sqrt(hessian_diag[i])));
        break;
      }
      case ScalingMethod::kAdaptiveDoubleSqrt: {
        (*D)[i] = min((*D)[i], 1 / sqrt(sqrt(hessian_diag[i])));
        break;
      }
    }
  }
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalScaleFactors(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().scale_factors_up_to_date) {
    CalcScaleFactors(state, &state.mutable_cache().scale_factors);
    state.mutable_cache().scale_factors_up_to_date = true;
  }
  return state.cache().scale_factors;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcEqualityConstraintViolations(
    const TrajectoryOptimizerState<T>& state, VectorX<T>* violations) const {
  INSTRUMENT_FUNCTION("Assemble torques on unactuated dofs.");
  const std::vector<VectorX<T>>& tau = EvalTau(state);
  const int num_unactuated_dofs = unactuated_dofs().size();

  for (int t = 0; t < num_steps(); ++t) {
    for (int j = 0; j < num_unactuated_dofs; ++j) {
      (*violations)(t * num_unactuated_dofs + j) = tau[t][unactuated_dofs()[j]];
    }
  }
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalEqualityConstraintViolations(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().constraint_violation_up_to_date) {
    CalcEqualityConstraintViolations(
        state, &state.mutable_cache().constraint_violation);
    state.mutable_cache().constraint_violation_up_to_date = true;
  }
  return state.cache().constraint_violation;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcEqualityConstraintJacobian(
    const TrajectoryOptimizerState<T>& state, MatrixX<T>* J) const {
  INSTRUMENT_FUNCTION("Assemble equality constraint Jacobian.");
  DRAKE_DEMAND(J->cols() == (num_steps() + 1) * plant().num_positions());
  DRAKE_DEMAND(J->rows() == num_equality_constraints());

  const InverseDynamicsPartials<T>& id_partials =
      EvalInverseDynamicsPartials(state);

  const int nq = plant().num_positions();
  const int n_steps = num_steps();
  const int n_unactuated = unactuated_dofs().size();

  for (int t = 0; t < n_steps; ++t) {
    for (int i = 0; i < n_unactuated; ++i) {
      // ∂hₜⁱ/∂qₜ₊₁
      J->block(t * n_unactuated + i, (t + 1) * nq, 1, nq) =
          id_partials.dtau_dqp[t].row(unactuated_dofs()[i]);

      // ∂hₜⁱ/∂qₜ
      if (t > 0) {
        J->block(t * n_unactuated + i, t * nq, 1, nq) =
            id_partials.dtau_dqt[t].row(unactuated_dofs()[i]);
      }

      // ∂hₜⁱ/∂qₜ₋₁
      if (t > 1) {
        J->block(t * n_unactuated + i, (t - 1) * nq, 1, nq) =
            id_partials.dtau_dqm[t].row(unactuated_dofs()[i]);
      }
    }
  }

  // With scaling enabled, the KKT conditions become
  //   [ DHD  DJ'][Δq] = [-g]
  //   [ JD    0 ][ λ]   [-h]
  // so we'll return the scaled version of the constraint Jacobian J̃ = JD
  if (params_.scaling) {
    const VectorX<T>& D = EvalScaleFactors(state);
    *J = (*J) * D.asDiagonal();
  }
}

template <typename T>
const MatrixX<T>& TrajectoryOptimizer<T>::EvalEqualityConstraintJacobian(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().constraint_jacobian_up_to_date) {
    CalcEqualityConstraintJacobian(state,
                                   &state.mutable_cache().constraint_jacobian);
    state.mutable_cache().constraint_jacobian_up_to_date = true;
  }
  return state.cache().constraint_jacobian;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcLagrangeMultipliers(
    const TrajectoryOptimizerState<T>&, VectorX<T>*) const {
  // We need to perform linear system solves to compute the lagrange
  // multipliers, so we don't support autodiff here.
  throw std::runtime_error("CalcLagrangeMultipliers() only supports T=double");
}

template <>
void TrajectoryOptimizer<double>::CalcLagrangeMultipliers(
    const TrajectoryOptimizerState<double>& state, VectorXd* lambda) const {
  INSTRUMENT_FUNCTION("Compute lagrange multipliers.");
  // λ = (J H⁻¹ Jᵀ)⁻¹ (h − J H⁻¹ g)
  const PentaDiagonalMatrix<double>& H = EvalScaledHessian(state);
  const VectorXd& g = EvalScaledGradient(state);
  const VectorXd& h = EvalEqualityConstraintViolations(state);
  const MatrixXd& J = EvalEqualityConstraintJacobian(state);

  // compute H⁻¹ Jᵀ
  // TODO(vincekurtz): add options for other linear systems solvers
  MatrixXd& Hinv_JT = state.workspace.num_vars_by_num_eq_cons_tmp;
  Hinv_JT = J.transpose();
  PentaDiagonalFactorization Hlu(H);
  DRAKE_DEMAND(Hlu.status() == PentaDiagonalFactorizationStatus::kSuccess);
  for (int i = 0; i < Hinv_JT.cols(); ++i) {
    // We need this variable to avoid taking the address of a temporary object
    auto ith_column = Hinv_JT.col(i);
    Hlu.SolveInPlace(&ith_column);
  }

  // TODO(vincekurtz): it may be possible to exploit the structure of JH⁻¹Jᵀ to
  // perform this step more efficiently.
  *lambda = (J * Hinv_JT).ldlt().solve(h - Hinv_JT.transpose() * g);
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalLagrangeMultipliers(
    const TrajectoryOptimizerState<T>& state) const {
  // We shouldn't be calling this unless equality constraints are enabled
  DRAKE_ASSERT(params_.equality_constraints);

  if (!state.cache().lagrange_multipliers_up_to_date) {
    CalcLagrangeMultipliers(state, &state.mutable_cache().lagrange_multipliers);
    state.mutable_cache().lagrange_multipliers_up_to_date = true;
  }
  return state.cache().lagrange_multipliers;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcMeritFunction(
    const TrajectoryOptimizerState<T>& state, T* merit) const {
  const T& L = EvalCost(state);
  const VectorX<T>& h = EvalEqualityConstraintViolations(state);
  const VectorX<T>& lambda = EvalLagrangeMultipliers(state);

  *merit = L + h.dot(lambda);
}

template <typename T>
const T TrajectoryOptimizer<T>::EvalMeritFunction(
    const TrajectoryOptimizerState<T>& state) const {
  // If we're not using equality constraints, the merit function is simply the
  // unconstrained cost.
  if (!params_.equality_constraints) return EvalCost(state);

  if (!state.cache().merit_up_to_date) {
    CalcMeritFunction(state, &state.mutable_cache().merit);
    state.mutable_cache().merit_up_to_date = true;
  }
  return state.cache().merit;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcMeritFunctionGradient(
    const TrajectoryOptimizerState<T>& state, VectorX<T>* g_tilde) const {
  const VectorX<T>& g = EvalScaledGradient(state);
  const VectorX<T>& lambda = EvalLagrangeMultipliers(state);
  const MatrixX<T>& J = EvalEqualityConstraintJacobian(state);

  *g_tilde = g + J.transpose() * lambda;
}

template <typename T>
const VectorX<T>& TrajectoryOptimizer<T>::EvalMeritFunctionGradient(
    const TrajectoryOptimizerState<T>& state) const {
  // If we're not using equality constraints, just return the regular gradient.
  if (!params_.equality_constraints) return EvalScaledGradient(state);

  if (!state.cache().merit_gradient_up_to_date) {
    CalcMeritFunctionGradient(state, &state.mutable_cache().merit_gradient);
    state.mutable_cache().merit_gradient_up_to_date = true;
  }
  return state.cache().merit_gradient;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcExactHessian(
    const TrajectoryOptimizerState<T>&, PentaDiagonalMatrix<T>*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::CalcExactHessian only supports T=double");
}

template <>
void TrajectoryOptimizer<double>::CalcExactHessian(
    const TrajectoryOptimizerState<double>& state,
    PentaDiagonalMatrix<double>* H) const {
  const int nq = plant().num_positions();
  const int num_vars = (num_steps() + 1) * nq;
  DRAKE_DEMAND(H->is_symmetric());
  DRAKE_DEMAND(H->block_rows() == num_steps() + 1);
  DRAKE_DEMAND(H->block_size() == nq);

  const std::vector<VectorX<double>>& q = state.q();
  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps() + 1,
                                        VectorX<AutoDiffXd>(nq));

  // Initialize q_ad
  int ad_idx = 0;
  for (int t = 0; t <= num_steps(); ++t) {
    for (int i = 0; i < nq; ++i) {
      q_ad[t].segment<1>(i) =
          math::InitializeAutoDiff(q[t].segment<1>(i), num_vars, ad_idx);
      ++ad_idx;
    }
  }
  state_ad_->set_q(q_ad);

  // Compute the autodiff gradient with finite differences
  const VectorX<AutoDiffXd>& g_ad = optimizer_ad_->EvalGradient(*state_ad_);

  // Extract the Hessian via autodiff
  MatrixXd H_dense = math::ExtractGradient(g_ad);
  H_dense.leftCols(nq).setZero();
  H_dense.block(0, 0, nq, nq).setIdentity();

  *H = H->MakeSymmetricFromLowerDense(H_dense, num_steps() + 1, nq);
}

template <typename T>
void TrajectoryOptimizer<T>::CalcCacheTrajectoryData(
    const TrajectoryOptimizerState<T>& state) const {
  TrajectoryOptimizerCache<T>& cache = state.mutable_cache();

  // The generalized positions that everything is computed from
  const std::vector<VectorX<T>>& q = state.q();

  // Compute corresponding generalized velocities
  std::vector<VectorX<T>>& v = cache.trajectory_data.v;
  const std::vector<MatrixX<T>>& Nplus = EvalNplus(state);
  CalcVelocities(q, Nplus, &v);

  // Compute corresponding generalized accelerations
  std::vector<VectorX<T>>& a = cache.trajectory_data.a;
  CalcAccelerations(v, &a);

  // Set cache invalidation flag
  cache.trajectory_data.up_to_date = true;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcInverseDynamicsCache(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::InverseDynamicsCache* cache) const {
  // Compute corresponding generalized torques
  const std::vector<VectorX<T>>& a = EvalA(state);
  CalcInverseDynamics(state, a, &cache->tau);

  // Set cache invalidation flag
  cache->up_to_date = true;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcContextCache(
    const TrajectoryOptimizerState<T>& state,
    typename TrajectoryOptimizerCache<T>::ContextCache* cache) const {
  const std::vector<VectorX<T>>& q = state.q();
  const std::vector<VectorX<T>>& v = EvalV(state);
  auto& plant_contexts = cache->plant_contexts;
  for (int t = 0; t <= num_steps(); ++t) {
    plant().SetPositions(plant_contexts[t], q[t]);
    plant().SetVelocities(plant_contexts[t], v[t]);
  }
  cache->up_to_date = true;
}

template <typename T>
const Context<T>& TrajectoryOptimizer<T>::EvalPlantContext(
    const TrajectoryOptimizerState<T>& state, int t) const {
  if (!state.cache().context_cache->up_to_date) {
    CalcContextCache(state, state.mutable_cache().context_cache.get());
  }
  return *state.cache().context_cache->plant_contexts[t];
}

template <typename T>
Context<T>& TrajectoryOptimizer<T>::GetMutablePlantContext(
    const TrajectoryOptimizerState<T>& state, int t) const {
  state.mutable_cache().context_cache->up_to_date = false;
  return *state.mutable_cache().context_cache->plant_contexts[t];
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalV(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date) CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.v;
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalA(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().trajectory_data.up_to_date) CalcCacheTrajectoryData(state);
  return state.cache().trajectory_data.a;
}

template <typename T>
const std::vector<VectorX<T>>& TrajectoryOptimizer<T>::EvalTau(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().inverse_dynamics_cache.up_to_date)
    CalcInverseDynamicsCache(state,
                             &state.mutable_cache().inverse_dynamics_cache);
  return state.cache().inverse_dynamics_cache.tau;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcCacheDerivativesData(
    const TrajectoryOptimizerState<T>& state) const {
  TrajectoryOptimizerCache<T>& cache = state.mutable_cache();

  // Some aliases
  InverseDynamicsPartials<T>& id_partials = cache.derivatives_data.id_partials;
  VelocityPartials<T>& v_partials = cache.derivatives_data.v_partials;

  // Compute partial derivatives of inverse dynamics d(tau)/d(q)
  CalcInverseDynamicsPartials(state, &id_partials);

  // Compute partial derivatives of velocities d(v)/d(q)
  CalcVelocityPartials(state, &v_partials);

  // Set cache invalidation flag
  cache.derivatives_data.up_to_date = true;
}

template <typename T>
const VelocityPartials<T>& TrajectoryOptimizer<T>::EvalVelocityPartials(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().derivatives_data.up_to_date)
    CalcCacheDerivativesData(state);
  return state.cache().derivatives_data.v_partials;
}

template <typename T>
const InverseDynamicsPartials<T>&
TrajectoryOptimizer<T>::EvalInverseDynamicsPartials(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().derivatives_data.up_to_date)
    CalcCacheDerivativesData(state);
  return state.cache().derivatives_data.id_partials;
}

template <typename T>
const std::vector<MatrixX<T>>& TrajectoryOptimizer<T>::EvalNplus(
    const TrajectoryOptimizerState<T>& state) const {
  if (!state.cache().n_plus_up_to_date) {
    CalcNplus(state, &state.mutable_cache().N_plus);
    state.mutable_cache().n_plus_up_to_date = true;
  }
  return state.cache().N_plus;
}

template <typename T>
void TrajectoryOptimizer<T>::CalcNplus(const TrajectoryOptimizerState<T>& state,
                                       std::vector<MatrixX<T>>* N_plus) const {
  DRAKE_DEMAND(static_cast<int>(N_plus->size()) == (num_steps() + 1));
  for (int t = 0; t <= num_steps(); ++t) {
    // Get a context storing q at time t
    // TODO(vincekurtz): consider using EvalPlantContext instead. In that case
    // we do need to be a bit careful, however, since EvalPlantContext requires
    // EvalV, which in turn requires EvalNplus.
    plant().SetPositions(context_, state.q()[t]);

    // Compute N+(q_t)
    plant().CalcNplusMatrix(*context_, &N_plus->at(t));
  }
}

template <typename T>
void TrajectoryOptimizer<T>::SaveLinePlotDataFirstVariable(
    TrajectoryOptimizerState<T>* scratch_state) const {
  std::ofstream data_file;
  data_file.open("lineplot_data.csv");
  data_file << "q, L, g, H\n";  // header

  // Establish sample points
  const double q_min = params_.lineplot_q_min;
  const double q_max = params_.lineplot_q_max;
  const double num_samples = 10000;
  const double dq = (q_max - q_min) / num_samples;

  const int nq = plant().num_positions();

  // Make a mutable copy of the decision variables
  std::vector<VectorX<T>> q = scratch_state->q();
  double qi = q_min;
  for (int i = 0; i < num_samples; ++i) {
    // Set the decision variables q
    q[1](0) = qi;
    scratch_state->set_q(q);

    // Compute cost, gradient, and Hessian for the first decision variable
    const T& L = EvalCost(*scratch_state);
    const T& g = EvalGradient(*scratch_state)[nq];
    const T& H = EvalHessian(*scratch_state).C()[1](0, 0);

    // Write to the file
    data_file << fmt::format("{}, {}, {}, {}\n", qi, L, g, H);

    // Move to the next value of q
    qi += dq;
  }
}

template <typename T>
void TrajectoryOptimizer<T>::SetupIterationDataFile() const {
  std::ofstream data_file;
  data_file.open("iteration_data.csv");
  data_file << "iter, q, cost, Delta, rho, dq\n";
  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveIterationData(
    const int iter, const double Delta, const double rho, const double dq,
    const TrajectoryOptimizerState<T>& state) const {
  std::ofstream data_file;
  data_file.open("iteration_data.csv", std::ios_base::app);

  const T& q = state.q()[1](0);  // assuming 1-DoF and 1 timestep
  const T& cost = EvalCost(state);

  data_file << fmt::format("{}, {}, {}, {}, {}, {}\n", iter, q, cost, Delta,
                           rho, dq);

  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveContourPlotDataFirstTwoVariables(
    TrajectoryOptimizerState<T>* scratch_state) const {
  using std::sqrt;
  std::ofstream data_file;
  data_file.open("contour_data.csv");
  data_file
      << "q1, q2, L, g1, g2, H11, H12, H21, H22, g_norm, H_norm\n";  // header

  // Establish sample points
  const double q1_min = params_.contour_q1_min;
  const double q1_max = params_.contour_q1_max;
  const double q2_min = params_.contour_q2_min;
  const double q2_max = params_.contour_q2_max;
  const int nq1 = 150;
  const int nq2 = 150;
  const double dq1 = (q1_max - q1_min) / nq1;
  const double dq2 = (q2_max - q2_min) / nq2;

  T cost;
  std::vector<VectorX<T>> q = scratch_state->q();
  double q1 = q1_min;
  for (int i = 0; i < nq1; ++i) {
    double q2 = q2_min;
    for (int j = 0; j < nq2; ++j) {
      // Update q
      q[1](0) = q1;
      q[1](1) = q2;

      // Compute L(q)
      scratch_state->set_q(q);
      cost = EvalCost(*scratch_state);

      const MatrixX<T> H = EvalHessian(*scratch_state).MakeDense();
      const VectorX<T> g = EvalGradient(*scratch_state);

      // Write to the file
      data_file << fmt::format("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n",
                               q1, q2, cost, g(2), g(3), H(2, 2), H(2, 3),
                               H(3, 2), H(3, 3), g.norm(),
                               H.block(2, 2, 2, 2).norm());

      q2 += dq2;
    }
    q1 += dq1;
  }

  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SetupQuadraticDataFile() const {
  std::ofstream data_file;
  data_file.open("quadratic_data.csv");
  data_file << "iter, q1, q2, dq1, dq2, Delta, cost , g1, g2, H11, H12, H21, "
               "H22, g_norm, H_norm\n";
  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveQuadraticDataFirstTwoVariables(
    const int iter, const double Delta, const VectorX<T>& dq,
    const TrajectoryOptimizerState<T>& state) const {
  std::ofstream data_file;
  data_file.open("quadratic_data.csv", std::ios_base::app);

  const int nq = plant().num_positions();
  const T q1 = state.q()[1](0);
  const T q2 = state.q()[1](1);
  const T dq1 = dq(nq);
  const T dq2 = dq(nq + 1);

  const VectorX<T>& g = EvalGradient(state);
  const MatrixX<T>& H = EvalHessian(state).MakeDense();
  const T g1 = g(nq);
  const T g2 = g(nq + 1);
  const T H11 = H(nq, nq);
  const T H12 = H(nq, nq + 1);
  const T H21 = H(nq + 1, nq);
  const T H22 = H(nq + 1, nq + 1);

  data_file << fmt::format(
      "{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n", iter, q1,
      q2, dq1, dq2, Delta, EvalCost(state), g1, g2, H11, H12, H21, H22,
      g.norm(), H.block(2, 2, 2, 2).norm());
  data_file.close();
}

template <typename T>
void TrajectoryOptimizer<T>::SaveLinesearchResidual(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state,
    const std::string filename) const {
  double alpha_min = -0.2;
  double alpha_max = 1.2;
  double dalpha = 0.01;

  std::ofstream data_file;
  data_file.open(filename);
  data_file << "alpha, cost, gradient, dq, L_prime \n";  // header

  double alpha = alpha_min;
  while (alpha <= alpha_max) {
    // Record the linesearch parameter alpha
    data_file << alpha << ", ";

    // Record the linesearch residual
    // phi(alpha) = L(q + alpha * dq) - L
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    if (params_.normalize_quaternions) NormalizeQuaternions(scratch_state);
    data_file << EvalCost(*scratch_state) - EvalCost(state) << ", ";

    // Record the norm of the gradient
    const VectorX<T>& g = EvalGradient(*scratch_state);
    data_file << g.norm() << ", ";

    // Record the norm of the search direction
    data_file << dq.norm() << ", ";

    // Record the derivative of the linesearch residual w.r.t alpha
    data_file << g.dot(dq) << "\n";

    alpha += dalpha;
  }
  data_file.close();
}

template <typename T>
std::tuple<double, int> TrajectoryOptimizer<T>::Linesearch(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  // The state's cache must be up to date, since we'll use the gradient and cost
  // information stored there.
  if (params_.linesearch_method == LinesearchMethod::kArmijo) {
    return ArmijoLinesearch(state, dq, scratch_state);
  } else if (params_.linesearch_method == LinesearchMethod::kBacktracking) {
    return BacktrackingLinesearch(state, dq, scratch_state);
  } else {
    throw std::runtime_error("Unknown linesearch method");
  }
}

template <typename T>
std::tuple<double, int> TrajectoryOptimizer<T>::BacktrackingLinesearch(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  using std::abs;

  // Compute the cost and gradient
  double mu = 0.0;
  if (params_.equality_constraints) {
    // Use an exact l1 penalty function as the merit function if equality
    // constraints are enforced exactly.
    // TODO(vincekurtz): add equality constraints to Armijo linesearch
    mu = 1e3;
  }
  const VectorX<T>& h = EvalEqualityConstraintViolations(state);
  const T L = EvalCost(state) + mu * h.cwiseAbs().sum();
  const VectorX<T>& g = EvalGradient(state);

  // Linesearch parameters
  const double c = 1e-4;
  const double rho = 0.8;

  double alpha = 1.0;
  T L_prime = g.transpose() * dq -
              mu * h.cwiseAbs().sum();  // gradient of L w.r.t. alpha

  // Make sure this is a descent direction
  DRAKE_DEMAND(L_prime <= 0);

  // Exit early with alpha = 1 when we are close to convergence
  const double convergence_threshold =
      std::sqrt(std::numeric_limits<double>::epsilon());
  if (abs(L_prime) / abs(L) <= convergence_threshold) {
    return {1.0, 0};
  }

  // Try with alpha = 1
  scratch_state->set_q(state.q());
  scratch_state->AddToQ(alpha * dq);
  if (params_.normalize_quaternions) NormalizeQuaternions(scratch_state);
  T L_old =
      EvalCost(*scratch_state) +
      mu * EvalEqualityConstraintViolations(*scratch_state).cwiseAbs().sum();

  // L_new stores cost at iteration i:   L(q + alpha_i * dq)
  // L_old stores cost at iteration i-1: L(q + alpha_{i-1} * dq)
  T L_new = L_old;

  // We'll keep reducing alpha until (1) we meet the Armijo convergence
  // criteria and (2) the cost increases, indicating that we're near a local
  // minimum.
  int i = 0;
  bool armijo_met = false;
  while (!(armijo_met && (L_new > L_old))) {
    // Save L_old = L(q + alpha_{i-1} * dq)
    L_old = L_new;

    // Reduce alpha
    alpha *= rho;

    // Compute L_new = L(q + alpha_i * dq)
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    if (params_.normalize_quaternions) NormalizeQuaternions(scratch_state);
    L_new =
        EvalCost(*scratch_state) +
        mu * EvalEqualityConstraintViolations(*scratch_state).cwiseAbs().sum();

    // Check the Armijo conditions
    if (L_new <= L + c * alpha * L_prime) {
      armijo_met = true;
    }

    ++i;
  }

  return {alpha / rho, i};
}

template <typename T>
std::tuple<double, int> TrajectoryOptimizer<T>::ArmijoLinesearch(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  using std::abs;

  // Compute the cost and gradient
  const T L = EvalCost(state);
  const VectorX<T>& g = EvalGradient(state);

  // Linesearch parameters
  const double c = 1e-4;
  const double rho = 0.8;

  double alpha = 1.0 / rho;        // get alpha = 1 on first iteration
  T L_prime = g.transpose() * dq;  // gradient of L w.r.t. alpha
  T L_new;                         // L(q + alpha * dq)

  // Make sure this is a descent direction
  DRAKE_DEMAND(L_prime <= 0);

  // Exit early with alpha = 1 when we are close to convergence
  const double convergence_threshold =
      10 * std::numeric_limits<double>::epsilon() / time_step() / time_step();
  if (abs(L_prime) / abs(L) <= convergence_threshold) {
    return {1.0, 0};
  }

  int i = 0;  // Iteration counter
  do {
    // Reduce alpha
    // N.B. we start with alpha = 1/rho, so we get alpha = 1 on the first
    // iteration.
    alpha *= rho;

    // Compute L_ls = L(q + alpha * dq)
    scratch_state->set_q(state.q());
    scratch_state->AddToQ(alpha * dq);
    if (params_.normalize_quaternions) NormalizeQuaternions(scratch_state);
    L_new = EvalCost(*scratch_state);

    ++i;
  } while ((L_new > L + c * alpha * L_prime) &&
           (i < params_.max_linesearch_iterations));

  return {alpha, i};
}

template <typename T>
T TrajectoryOptimizer<T>::CalcTrustRatio(
    const TrajectoryOptimizerState<T>& state, const VectorX<T>& dq,
    TrajectoryOptimizerState<T>* scratch_state) const {
  // Quantities at the current iteration (k)
  const T merit_k = EvalMeritFunction(state);
  const VectorX<T>& g_tilde_k = EvalMeritFunctionGradient(state);
  const PentaDiagonalMatrix<T>& H_k = EvalScaledHessian(state);

  // Quantities at the next iteration if we accept the step (kp = k+1)
  // TODO(vincekurtz): if we do end up accepting the step, it would be nice to
  // somehow reuse these cached quantities, which we're currently trashing
  scratch_state->set_q(state.q());
  scratch_state->AddToQ(dq);
  if (params_.normalize_quaternions) NormalizeQuaternions(scratch_state);
  T merit_kp = EvalCost(*scratch_state);
  if (params_.equality_constraints) {
    // N.B. We use λₖ rather than λₖ₊₁ to compute the merit function
    // ϕₖ₊₁ = L(qₖ₊₁) + h(qₖ₊₁)ᵀλₖ here because we are assuming that λ is
    // constant.
    const VectorX<T>& lambda_k = EvalLagrangeMultipliers(state);
    const VectorX<T>& h_kp = EvalEqualityConstraintViolations(*scratch_state);
    merit_kp += h_kp.dot(lambda_k);
  }

  // Compute predicted reduction in the merit function, −gᵀΔq − 1/2 ΔqᵀHΔq
  VectorX<T>& dq_scaled = state.workspace.num_vars_size_tmp1;
  if (params_.scaling) {
    const VectorX<T>& D = EvalScaleFactors(state);
    // TODO(vincekurtz): consider caching D^{-1}
    dq_scaled = D.cwiseInverse().asDiagonal() * dq;
  } else {
    dq_scaled = dq;
  }
  VectorX<T>& Hdq = state.workspace.num_vars_size_tmp2;
  H_k.MultiplyBy(dq_scaled, &Hdq);  // Hdq = H_k * dq
  const T hessian_term = 0.5 * dq_scaled.transpose() * Hdq;
  T gradient_term = g_tilde_k.dot(dq_scaled);
  const T predicted_reduction = -gradient_term - hessian_term;

  // Compute actual reduction in the merit function
  const T actual_reduction = merit_k - merit_kp;

  // Threshold for determining when the actual and predicted reduction in cost
  // are essentially zero. This is determined by the approximate level of
  // floating point error in our computation of the cost, L(q).
  const double eps =
      10 * std::numeric_limits<T>::epsilon() / time_step() / time_step();
  if ((predicted_reduction < eps) && (actual_reduction < eps)) {
    // Actual and predicted improvements are both essentially zero, so we set
    // the trust ratio to a value such that the step will be accepted, but the
    // size of the trust region will not change.
    return 0.5;
  }

  return actual_reduction / predicted_reduction;
}

template <typename T>
T TrajectoryOptimizer<T>::SolveDoglegQuadratic(const T& a, const T& b,
                                               const T& c) const {
  using std::sqrt;
  // Check that a is positive
  DRAKE_DEMAND(a > 0);

  T s;
  if (a < std::numeric_limits<double>::epsilon()) {
    // If a is essentially zero, just solve bx + c = 0
    s = -c / b;
  } else {
    // Normalize everything by a
    const T b_tilde = b / a;
    const T c_tilde = c / a;

    const T determinant = b_tilde * b_tilde - 4 * c_tilde;
    DRAKE_DEMAND(determinant > 0);  // We know a real root exists

    // We know that there is only one positive root, so we just take the big
    // root
    s = (-b_tilde + sqrt(determinant)) / 2;
  }

  // We know the solution is between zero and one
  DRAKE_DEMAND(0 < s);
  DRAKE_DEMAND(s < 1);

  return s;
}

template <typename T>
void TrajectoryOptimizer<T>::SolveLinearSystemInPlace(
    const PentaDiagonalMatrix<T>&, EigenPtr<VectorX<T>>) const {
  // Only T=double is supported here, since most of our solvers only support
  // double.
  throw std::runtime_error(
      "TrajectoryOptimizer::SolveLinearSystemInPlace() only supports T=double");
}

template <>
void TrajectoryOptimizer<double>::SolveLinearSystemInPlace(
    const PentaDiagonalMatrix<double>& H, EigenPtr<VectorX<double>> b) const {
  switch (params_.linear_solver) {
    case SolverParameters::LinearSolverType::kPentaDiagonalLu: {
      PentaDiagonalFactorization Hlu(H);
      DRAKE_DEMAND(Hlu.status() == PentaDiagonalFactorizationStatus::kSuccess);
      Hlu.SolveInPlace(b);
      break;
    }
    case SolverParameters::LinearSolverType::kDenseLdlt: {
      const MatrixX<double> Hdense = H.MakeDense();
      const auto& Hldlt = Hdense.ldlt();
      *b = Hldlt.solve(*b);
      DRAKE_DEMAND(Hldlt.info() == Eigen::Success);
      break;
    }
    case SolverParameters::LinearSolverType::kPetsc: {
      auto Hpetsc = internal::PentaDiagonalToPetscMatrix(H);
      Hpetsc->set_relative_tolerance(
          params_.petsc_parameters.relative_tolerance);
      PetscSolverStatus status =
          Hpetsc->SolveInPlace(params_.petsc_parameters.solver_type,
                               params_.petsc_parameters.preconditioner_type, b);
      DRAKE_DEMAND(status == PetscSolverStatus::kSuccess);
      break;
    }
  }
}

template <typename T>
bool TrajectoryOptimizer<T>::CalcDoglegPoint(const TrajectoryOptimizerState<T>&,
                                             const double, VectorX<T>*,
                                             VectorX<T>*) const {
  // Only T=double is supported here, since pentadigonal matrix factorization is
  // (sometimes) required to compute the dogleg point.
  throw std::runtime_error(
      "TrajectoryOptimizer::CalcDoglegPoint only supports T=double");
}

template <>
bool TrajectoryOptimizer<double>::CalcDoglegPoint(
    const TrajectoryOptimizerState<double>& state, const double Delta,
    VectorXd* dq, VectorXd* dqH) const {
  INSTRUMENT_FUNCTION("Find search direction with dogleg method.");

  // If params_.scaling = false, this returns the regular Hessian.
  const PentaDiagonalMatrix<double>& H = EvalScaledHessian(state);

  // If equality constraints are active, we'll use the gradient of the merit
  // function, g̃ = g + J'λ. This means the full step pH satisfies the KKT
  // conditions
  //     [H  J']*[pH] = [-g]
  //     [J  0 ] [ λ]   [-h]
  // while the shortened step pU minimizes the quadratic approximation in the
  // direction of -g - J'λ.
  const VectorXd& g = EvalMeritFunctionGradient(state);

  VectorXd& Hg = state.workspace.num_vars_size_tmp1;
  H.MultiplyBy(g, &Hg);
  const double gHg = g.transpose() * Hg;

  // Compute the full Gauss-Newton step
  // N.B. We can avoid computing pH when pU is the dog-leg solution.
  // However, we compute it here for logging stats since thus far the cost of
  // computing pH is negligible compared to other costs (namely the computation
  // of gradients of the inverse dynamics.)
  // TODO(amcastro-tri): move this to after pU whenever we make the cost of
  // gradients computation negligible.
  VectorXd& pH = state.workspace.num_vars_size_tmp2;

  pH = -g / Delta;  // normalize by Δ
  SolveLinearSystemInPlace(H, &pH);

  if (params_.debug_compare_against_dense) {
    // From experiments in penta_diagonal_solver_test.cc
    // (PentaDiagonalMatrixTest.SolvePentaDiagonal), LDLT is the most stable
    // solver to round-off errors. We therefore use it as a reference solution
    // for debugging.
    const VectorXd pH_dense = H.MakeDense().ldlt().solve(-g / Delta);
    std::cout << fmt::format("Sparse vs. Dense error: {}\n",
                             (pH - pH_dense).norm() / pH_dense.norm());
  }

  *dqH = pH * Delta;

  // Compute the unconstrained minimizer of m(δq) = L(q) + g(q)'*δq + 1/2
  // δq'*H(q)*δq along -g
  VectorXd& pU = state.workspace.num_vars_size_tmp3;
  pU = -(g.dot(g) / gHg) * g / Delta;  // normalize by Δ

  // Check if the trust region is smaller than this unconstrained minimizer
  if (1.0 <= pU.norm()) {
    // If so, δq is where the first leg of the dogleg path intersects the trust
    // region.
    *dq = (Delta / pU.norm()) * pU;
    if (params_.scaling) {
      *dq = EvalScaleFactors(state).asDiagonal() * (*dq);
    }
    return true;  // the trust region constraint is active
  }

  // Check if the trust region is large enough to just take the full Newton step
  if (1.0 >= pH.norm()) {
    *dq = pH * Delta;
    if (params_.scaling) {
      // TODO(vincekurtz): consider adding a MultiplyByScaleFactors method
      *dq = EvalScaleFactors(state).asDiagonal() * (*dq);
    }
    return false;  // the trust region constraint is not active
  }

  // Compute the intersection between the second leg of the dogleg path and the
  // trust region. We'll do this by solving the (scalar) quadratic
  //
  //    ‖ pU + s( pH − pU ) ‖² = y²
  //
  // for s ∈ (0,1),
  //
  // and setting
  //
  //    δq = pU + s( pH − pU ).
  //
  // Note that we normalize by Δ to minimize roundoff error.
  const double a = (pH - pU).dot(pH - pU);
  const double b = 2 * pU.dot(pH - pU);
  const double c = pU.dot(pU) - 1.0;
  const double s = SolveDoglegQuadratic(a, b, c);

  *dq = (pU + s * (pH - pU)) * Delta;
  if (params_.scaling) {
    *dq = EvalScaleFactors(state).asDiagonal() * (*dq);
  }
  return true;  // the trust region constraint is active
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::Solve(const std::vector<VectorX<T>>&,
                                         TrajectoryOptimizerSolution<T>*,
                                         TrajectoryOptimizerStats<T>*,
                                         ConvergenceReason*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::Solve only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::Solve(
    const std::vector<VectorXd>& q_guess,
    TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats, ConvergenceReason* reason) const {
  INSTRUMENT_FUNCTION("Main entry point.");

  // The guess must be consistent with the initial condition
  DRAKE_DEMAND(q_guess[0] == prob_.q_init);
  DRAKE_DEMAND(static_cast<int>(q_guess.size()) == num_steps() + 1);

  // stats must be empty
  DRAKE_DEMAND(stats->is_empty());

  if (params_.method == SolverMethod::kLinesearch) {
    return SolveWithLinesearch(q_guess, solution, stats);
  } else if (params_.method == SolverMethod::kTrustRegion) {
    return SolveWithTrustRegion(q_guess, solution, stats, reason);
  } else {
    throw std::runtime_error("Unsupported solver strategy!");
  }
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::SolveWithLinesearch(
    const std::vector<VectorX<T>>&, TrajectoryOptimizerSolution<T>*,
    TrajectoryOptimizerStats<T>*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::SolveWithLinesearch only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::SolveWithLinesearch(
    const std::vector<VectorXd>& q_guess,
    TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats) const {
  // Allocate a state variable
  TrajectoryOptimizerState<double> state = CreateState();
  state.set_q(q_guess);

  // Allocate a separate state variable for linesearch
  TrajectoryOptimizerState<double> scratch_state = CreateState();

  // Allocate cost and search direction
  double cost;
  VectorXd dq((num_steps() + 1) * plant().num_positions());

  // Set proximal operator data for the first iteration
  // N.B. since state.proximal_operator_data.H_diag is initialized to zero, this
  // first computation of the Hessian, which is for scaling purposes only, will
  // not include the proximal operator term.
  if (params_.proximal_operator) {
    state.set_proximal_operator_data(q_guess, EvalHessian(state));
    scratch_state.set_proximal_operator_data(q_guess, EvalHessian(state));
  }

  if (params_.verbose) {
    // Define printout data
    std::cout << "-------------------------------------------------------------"
                 "----------------------"
              << std::endl;
    std::cout << "|  iter  |   cost   |  alpha  |  LS_iters  |  time (s)  |  "
                 "|g|/cost  |    |h|     |"
              << std::endl;
    std::cout << "-------------------------------------------------------------"
                 "----------------------"
              << std::endl;
  }

  // Allocate timing variables
  auto start_time = std::chrono::high_resolution_clock::now();
  auto iter_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> iter_time;
  std::chrono::duration<double> solve_time;

  // Gauss-Newton iterations
  int k = 0;                       // iteration counter
  bool linesearch_failed = false;  // linesearch success flag
  do {
    iter_start_time = std::chrono::high_resolution_clock::now();

    // Compute the total cost
    cost = EvalCost(state);

    // Evaluate constraint violations (for logging)
    const VectorXd& h = EvalEqualityConstraintViolations(state);

    // Compute gradient and Hessian
    const VectorXd& g = EvalMeritFunctionGradient(state);
    const PentaDiagonalMatrix<double>& H = EvalHessian(state);

    // Compute the search direction. If equality constraints are active, this
    // solves the KKT conditions
    //    [H  J']*[dq] = [-g]
    //    [J  0 ] [ λ]   [-h]
    // for the search direction (since we've defined g as g + J'λ). Otherwise,
    // we solve H*dq = -g.
    dq = -g;
    SolveLinearSystemInPlace(H, &dq);

    // Solve the linsearch
    // N.B. we use a separate state variable since we will need to compute
    // L(q+alpha*dq) (at the very least), and we don't want to change state.q
    auto [alpha, ls_iters] = Linesearch(state, dq, &scratch_state);

    // Record linesearch data, if requested
    if (params_.linesearch_plot_every_iteration) {
      SaveLinesearchResidual(state, dq, &scratch_state,
                             fmt::format("linesearch_data_{}.csv", k));
    }

    if (ls_iters >= params_.max_linesearch_iterations) {
      linesearch_failed = true;

      if (params_.verbose) {
        std::cout << "LINESEARCH FAILED" << std::endl;
        std::cout << "Reached maximum linesearch iterations ("
                  << params_.max_linesearch_iterations << ")." << std::endl;
      }

      // Save the linesearch residual to a csv file so we can plot in python
      SaveLinesearchResidual(state, dq, &scratch_state);
    }

    // Compute the trust ratio (actual cost reduction / model cost reduction)
    double trust_ratio = CalcTrustRatio(state, alpha * dq, &scratch_state);

    // Update the decision variables
    state.AddToQ(alpha * dq);
    if (params_.normalize_quaternions) NormalizeQuaternions(&state);

    // Update the stored decision variables for the proximal operator cost
    if (params_.proximal_operator) {
      state.set_proximal_operator_data(state.q(), H);
      scratch_state.set_proximal_operator_data(state.q(), H);
    }

    iter_time = std::chrono::high_resolution_clock::now() - iter_start_time;

    // Nice little printout of our problem data
    if (params_.verbose) {
      printf("| %6d ", k);
      printf("| %8.3f ", cost);
      printf("| %7.4f ", alpha);
      printf("| %6d     ", ls_iters);
      printf("| %8.8f ", iter_time.count());
      printf("| %10.3e ", g.norm() / cost);
      printf("| %10.3e |\n", h.norm());
    }

    // Print additional debuging information
    if (params_.print_debug_data) {
      double condition_number = 1 / H.MakeDense().ldlt().rcond();
      double L_prime = g.transpose() * dq;
      std::cout << "Condition #: " << condition_number << std::endl;
      std::cout << "|| dq ||   : " << dq.norm() << std::endl;
      std::cout << "||  g ||   : " << g.norm() << std::endl;
      std::cout << "L'         : " << L_prime << std::endl;
      std::cout << "L          : " << cost << std::endl;
      std::cout << "L' / L     : " << L_prime / cost << std::endl;
      std::cout << "||diag(H)||: " << H.MakeDense().diagonal().norm()
                << std::endl;
      if (k > 0) {
        std::cout << "L[k] - L[k-1]: " << cost - stats->iteration_costs[k - 1]
                  << std::endl;
      }
    }

    const double dL_dq = g.dot(dq) / cost;

    // Record iteration data
    stats->push_data(iter_time.count(),  // iteration time
                     cost,               // cost
                     ls_iters,           // sub-problem iterations
                     alpha,              // linesearch parameter
                     NAN,                // trust region size
                     state.norm(),       // q norm
                     dq.norm(),          // step size
                     dq.norm(),          // step size
                     trust_ratio,        // trust ratio
                     g.norm(),           // gradient size
                     dL_dq,              // gradient along dq
                     h.norm(),           // equality constraint violation
                     cost);              // merit function

    ++k;
  } while (k < params_.max_iterations && !linesearch_failed);

  // End the problem data printout
  if (params_.verbose) {
    std::cout << "-------------------------------------------------------------"
                 "----------------------"
              << std::endl;
  }

  // Record the total solve time
  solve_time = std::chrono::high_resolution_clock::now() - start_time;
  stats->solve_time = solve_time.count();

  // Record the solution
  solution->q = state.q();
  solution->v = EvalV(state);
  solution->tau = EvalTau(state);

  if (linesearch_failed) {
    return SolverFlag::kLinesearchMaxIters;
  } else {
    return SolverFlag::kSuccess;
  }
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::SolveWithTrustRegion(
    const std::vector<VectorX<T>>&, TrajectoryOptimizerSolution<T>*,
    TrajectoryOptimizerStats<T>*, ConvergenceReason*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::SolveWithTrustRegion only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::SolveWithTrustRegion(
    const std::vector<VectorXd>& q_guess,
    TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats,
    ConvergenceReason* reason_out) const {
  INSTRUMENT_FUNCTION("Trust region solver.");

  // Set up a file to record iteration data for a contour plot, if requested
  if (params_.save_contour_data) {
    SetupQuadraticDataFile();
  }
  if (params_.save_lineplot_data) {
    SetupIterationDataFile();
  }

  // Allocate a warm start, which includes the initial guess along with state
  // variables and the trust region radius.
  WarmStart warm_start(num_steps(), diagram(), plant(),
                       num_equality_constraints(), q_guess, params_.Delta0);

  return SolveFromWarmStart(&warm_start, solution, stats, reason_out);
  return SolverFlag::kSuccess;
}

template <typename T>
SolverFlag TrajectoryOptimizer<T>::SolveFromWarmStart(
    WarmStart*, TrajectoryOptimizerSolution<T>*, TrajectoryOptimizerStats<T>*,
    ConvergenceReason*) const {
  throw std::runtime_error(
      "TrajectoryOptimizer::SolveFromWarmStart only supports T=double.");
}

template <>
SolverFlag TrajectoryOptimizer<double>::SolveFromWarmStart(
    WarmStart* warm_start, TrajectoryOptimizerSolution<double>* solution,
    TrajectoryOptimizerStats<double>* stats,
    ConvergenceReason* reason_out) const {
  using std::min;
  INSTRUMENT_FUNCTION("Solve with warm start.");

  // Allocate timing variables
  auto start_time = std::chrono::high_resolution_clock::now();
  auto iter_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> iter_time;
  std::chrono::duration<double> solve_time;

  // Warm-starting doesn't support the linesearch method
  DRAKE_DEMAND(params_.method == SolverMethod::kTrustRegion);

  // State variable stores q and everything that is computed from q
  TrajectoryOptimizerState<double>& state = warm_start->state;
  TrajectoryOptimizerState<double>& scratch_state = warm_start->scratch_state;

  // The update vector q_{k+1} = q_k + dq and full Newton step (for logging)
  VectorXd& dq = warm_start->dq;
  VectorXd& dqH = warm_start->dqH;

  // Trust region parameters
  const double Delta_max = params_.Delta_max;  // Maximum trust region size
  const double eta = 0.0;  // Trust ratio threshold - we accept steps if
                           // the trust ratio is above this threshold

  // Variables that we'll update throughout the main loop
  int k = 0;                          // iteration counter
  double& Delta = warm_start->Delta;  // trust region size
  double rho;                         // trust region ratio
  bool tr_constraint_active;          // flag for whether the trust region
                                      // constraint is active

  // Define printout strings
  const std::string separator_bar =
      "------------------------------------------------------------------------"
      "---------------------";
  const std::string printout_labels =
      "|  iter  |   cost   |    Δ    |    ρ    |  time (s)  |  |g|/cost  | "
      "dL_dq/cost |    |h|     |";

  double previous_cost = EvalCost(state);
  while (k < params_.max_iterations) {
    // Obtain the candiate update dq
    tr_constraint_active = CalcDoglegPoint(state, Delta, &dq, &dqH);

    if (params_.print_debug_data) {
      // Print some info about the Hessian
      const MatrixXd H = EvalHessian(state).MakeDense();
      const MatrixXd H_scaled = EvalScaledHessian(state).MakeDense();
      const double condition_number = 1 / H.ldlt().rcond();
      const double condition_number_scaled = 1 / H_scaled.ldlt().rcond();
      PRINT_VAR(condition_number);
      PRINT_VAR(condition_number_scaled);
    }

    // Compute some quantities for logging.
    // N.B. These should be computed before q is updated.
    const VectorXd& g = EvalMeritFunctionGradient(state);
    const VectorXd& h = EvalEqualityConstraintViolations(state);

    const double cost = EvalCost(state);
    const double merit = EvalMeritFunction(state);
    const double q_norm = state.norm();
    double dL_dq;
    if (params_.scaling) {
      const VectorXd& D = EvalScaleFactors(state);
      dL_dq = g.dot(D.cwiseInverse().asDiagonal() * dq) / cost;
    } else {
      dL_dq = g.dot(dq) / cost;
    }

    // Compute the trust region ratio
    rho = CalcTrustRatio(state, dq, &scratch_state);

    // With a positive definite Hessian, steps should not oppose the descent
    // direction
    if (!params_.exact_hessian) {
      DRAKE_DEMAND(dL_dq < std::numeric_limits<double>::epsilon());
    } else {
      // Reduce the trust region and reject the step if this is not a descent
      // direction
      if (dq.transpose() * g >= 0) {
        rho = -1.0;
      }
    }

    // Save data related to our quadratic approximation (for the first two
    // variables)
    if (params_.save_contour_data) {
      SaveQuadraticDataFirstTwoVariables(k, Delta, dq, state);
    }
    if (params_.save_lineplot_data) {
      SaveIterationData(k, Delta, rho, dq(1), state);
    }

    // If the ratio is large enough, accept the change
    if (rho > eta) {
      // Update the coefficients for the proximal operator cost
      if (params_.proximal_operator) {
        state.set_proximal_operator_data(state.q(), EvalHessian(state));
        scratch_state.set_proximal_operator_data(state.q(), EvalHessian(state));
      }

      state.AddToQ(dq);  // q += dq
      if (params_.normalize_quaternions) NormalizeQuaternions(&state);
    }
    // Else (rho <= eta), the trust region ratio is too small to accept dq, so
    // we'll need to so keep reducing the trust region. Note that the trust
    // region will be reduced in this case, since eta < 0.25.

    // N.B. if this is the case (q_{k+1} = q_k), we haven't touched state, so we
    // should be reusing the cached gradient and Hessian in the next iteration.
    // TODO(vincekurtz): should we be caching the factorization of the Hessian,
    // as well as the Hessian itself?

    // Compute iteration timing
    // N.B. this is in kind of a weird place because we want to record
    // statistics before updating the trust-region size. That ensures that
    // ‖ δq ‖ ≤ Δ in our logs.
    iter_time = std::chrono::high_resolution_clock::now() - iter_start_time;
    iter_start_time = std::chrono::high_resolution_clock::now();

    // Printout statistics from this iteration
    if (params_.verbose) {
      if ((k % 50) == 0) {
        // Refresh the labels for easy reading
        std::cout << separator_bar << std::endl;
        std::cout << printout_labels << std::endl;
        std::cout << separator_bar << std::endl;
      }
      std::cout << fmt::format(
          "| {:>6} | {:>8.3g} | {:>7.2} | {:>7.3} | {:>10.5} | {:>10.5} | "
          "{:>10.4} | {:>10.4} |\n",
          k, cost, Delta, rho, iter_time.count(), g.norm() / cost, dL_dq,
          h.norm());
    }

    // Record statistics from this iteration
    stats->push_data(iter_time.count(),  // iteration time
                     cost,               // cost
                     0,                  // linesearch iterations
                     NAN,                // linesearch parameter
                     Delta,              // trust region size
                     q_norm,             // q norm
                     dq.norm(),          // step size
                     dqH.norm(),         // Unconstrained step size
                     rho,                // trust region ratio
                     g.norm(),           // gradient size
                     dL_dq,              // gradient along dq
                     h.norm(),           // equality constraint violation
                     merit);             // merit function

    // Only check convergence criteria for valid steps.
    ConvergenceReason reason{
        ConvergenceReason::kNoConvergenceCriteriaSatisfied};
    if (params_.check_convergence && (rho > eta)) {
      reason = VerifyConvergenceCriteria(state, previous_cost, dq);
      previous_cost = EvalCost(state);
      if (reason_out) *reason_out = reason;
    }

    if (reason != ConvergenceReason::kNoConvergenceCriteriaSatisfied) {
      break;
    }

    // Update the size of the trust-region, if necessary
    if (rho < 0.25) {
      // If the ratio is small, our quadratic approximation is bad, so reduce
      // the trust region
      Delta *= 0.25;
    } else if ((rho > 0.75) && tr_constraint_active) {
      // If the ratio is large and we're at the boundary of the trust
      // region, increase the size of the trust region.
      Delta = min(2 * Delta, Delta_max);
    }

    ++k;
  }

  // Finish our printout
  if (params_.verbose) {
    std::cout << separator_bar << std::endl;
  }

  solve_time = std::chrono::high_resolution_clock::now() - start_time;
  stats->solve_time = solve_time.count();

  // Record the solution
  solution->q = state.q();
  solution->v = EvalV(state);
  solution->tau = EvalTau(state);

  // Record L(q) for various values of q so we can make plots
  if (params_.save_contour_data) {
    SaveContourPlotDataFirstTwoVariables(&scratch_state);
  }
  if (params_.save_lineplot_data) {
    SaveLinePlotDataFirstVariable(&scratch_state);
  }

  if (k == params_.max_iterations) return SolverFlag::kMaxIterationsReached;

  return SolverFlag::kSuccess;
}

template <typename T>
ConvergenceReason TrajectoryOptimizer<T>::VerifyConvergenceCriteria(
    const TrajectoryOptimizerState<T>& state, const T& previous_cost,
    const VectorX<T>& dq) const {
  using std::abs;

  const auto& tolerances = params_.convergence_tolerances;

  int reason(ConvergenceReason::kNoConvergenceCriteriaSatisfied);

  // Cost reduction criterion:
  //   |Lᵏ−Lᵏ⁺¹| < εₐ + εᵣ Lᵏ⁺¹
  const T cost = EvalCost(state);
  if (abs(previous_cost - cost) <
      tolerances.abs_cost_reduction + tolerances.rel_cost_reduction * cost) {
    reason |= ConvergenceReason::kCostReductionCriterionSatisfied;
  }

  // Gradient criterion:
  //   g⋅Δq < εₐ + εᵣ Lᵏ
  const VectorX<T>& g = EvalMeritFunctionGradient(state);
  if (abs(g.dot(dq)) < tolerances.abs_gradient_along_dq +
                           tolerances.rel_gradient_along_dq * cost) {
    reason |= ConvergenceReason::kGradientCriterionSatisfied;
  }

  // Relative state (q) change:
  //   ‖Δq‖ < εₐ + εᵣ‖qᵏ‖
  const T q_norm = state.norm();
  const T dq_norm = dq.norm();
  if (dq_norm <
      tolerances.abs_state_change + tolerances.rel_state_change * q_norm) {
    reason |= ConvergenceReason::kSateCriterionSatisfied;
  }

  return ConvergenceReason(reason);
}

template <typename T>
void TrajectoryOptimizer<T>::NormalizeQuaternions(
    TrajectoryOptimizerState<T>* state) const {
  std::vector<VectorX<T>>& q = state->mutable_q();
  DRAKE_DEMAND(static_cast<int>(q.size()) == (num_steps() + 1));
  // TODO(amcastro-tri): Store floating body indexes and avoid calling
  // GetFloatingBaseBodies().
  for (const BodyIndex& index : plant().GetFloatingBaseBodies()) {
    const Body<T>& body = plant().get_body(index);
    const int q_start = body.floating_positions_start();
    DRAKE_DEMAND(body.has_quaternion_dofs());
    for (int t = 0; t <= num_steps(); ++t) {
      auto body_qs = q[t].template segment<4>(q_start);
      body_qs.normalize();
    }
  }
}

}  // namespace traj_opt
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::traj_opt::TrajectoryOptimizer)
