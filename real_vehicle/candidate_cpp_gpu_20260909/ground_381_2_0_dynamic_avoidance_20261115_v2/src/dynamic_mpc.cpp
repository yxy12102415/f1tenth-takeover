// Dynamic bicycle model adapted from the previously ground-tested
// ground_fitzpatrick_dynamic_mpc_20260912_v2/src/ground_pp/core.hpp.
// Local-path geometry and longitudinal control remain in the Frenet controller.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

extern "C" int mpc_projected_iterations(int, int, const double*, const double*,
    const double*, double, double, double, double, double, double*);

namespace {
constexpr int N = 16, D = 4, S = (N+1)*D;
bool finite(const double* values, int count) {
  if (!values) return false;
  for (int i=0; i<count; ++i) if (!std::isfinite(values[i])) return false;
  return true;
}
bool discrete(double speed, double dt, const double* model,
              Eigen::Matrix4d& a, Eigen::Vector4d& b, Eigen::Vector4d& curvature_input) {
  if (!finite(model, 6) || !std::isfinite(speed) || speed <= 0.0 ||
      !std::isfinite(dt) || dt <= 0.0) return false;
  for (int i=0; i<6; ++i) if (model[i] <= 0.0) return false;
  const double m=model[0], iz=model[1], lf=model[2], lr=model[3], cf=model[4], cr=model[5];
  Eigen::Matrix4d continuous = Eigen::Matrix4d::Zero();
  continuous(0,1)=speed; continuous(0,2)=1.0; continuous(1,3)=1.0;
  continuous(2,2)=-(cf+cr)/(m*speed);
  continuous(2,3)=-speed-(cf*lf-cr*lr)/(m*speed);
  continuous(3,2)=-(cf*lf-cr*lr)/(iz*speed);
  continuous(3,3)=-(cf*lf*lf+cr*lr*lr)/(iz*speed);
  const Eigen::Matrix4d implicit=Eigen::Matrix4d::Identity()-dt*continuous;
  if (!implicit.allFinite() || std::abs(implicit.determinant()) < 1e-12) return false;
  const Eigen::PartialPivLU<Eigen::Matrix4d> factor(implicit);
  a=factor.solve(Eigen::Matrix4d::Identity());
  b=factor.solve(dt*Eigen::Vector4d(0., 0., cf/m, lf*cf/iz));
  curvature_input=factor.solve(dt*Eigen::Vector4d(0., -speed, 0., 0.));
  return a.allFinite() && b.allFinite() && curvature_input.allFinite();
}
}

extern "C" int dynamic_mpc_abi() { return 1; }

// Exported for independent model-equation and stability regression tests.
extern "C" int dynamic_discrete_model(double speed, double dt, const double* model,
    double* a_output, double* b_output, double* curvature_output) {
  if (!a_output || !b_output || !curvature_output) return -1;
  Eigen::Matrix4d a; Eigen::Vector4d b, c;
  if (!discrete(speed, dt, model, a, b, c)) return -1;
  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> output_a(a_output);
  Eigen::Map<Eigen::Vector4d> output_b(b_output), output_c(curvature_output);
  output_a=a; output_b=b; output_c=c;
  return 0;
}

extern "C" int dynamic_mpc_solve(const double* initial_state,
    const double* speeds, const double* curvatures, const double* model,
    const double* weights, const double* previous, double angle,
    double steering_limit, double first_step, double prediction_step,
    double dt, int maximum_iterations, double* control_output, double* maxima_output) {
  if (!finite(initial_state,D) || !finite(speeds,N) || !finite(curvatures,N) ||
      !finite(weights,10) || !finite(previous,N) || !control_output || !maxima_output)
    return -1;
  for (int i=0; i<10; ++i) if (weights[i] < 0.0) return -1;
  if (weights[8] <= 0.0) return -1;
  Eigen::Matrix<double,S,D> state_map=Eigen::Matrix<double,S,D>::Zero();
  Eigen::Matrix<double,S,N> input_map=Eigen::Matrix<double,S,N>::Zero();
  Eigen::Matrix<double,S,1> affine=Eigen::Matrix<double,S,1>::Zero();
  state_map.topRows<D>().setIdentity();
  Eigen::Matrix4d a; Eigen::Vector4d b, c;
  for (int step=0; step<N; ++step) {
    // Reuse the discretization within a constant-speed horizon. The model
    // still updates on every control tick and supports per-step speeds.
    if (step == 0 || speeds[step] != speeds[step-1])
      if (!discrete(speeds[step], dt, model, a, b, c)) return -2;
    int old=step*D, next=(step+1)*D;
    state_map.block<D,D>(next,0).noalias()=a*state_map.block<D,D>(old,0);
    input_map.block<D,N>(next,0).noalias()=a*input_map.block<D,N>(old,0);
    input_map.block<D,1>(next,step)+=b;
    affine.segment<D>(next)=a*affine.segment<D>(old)+c*curvatures[step];
  }
  Eigen::Matrix<double,S,1> q;
  for (int step=0; step<=N; ++step)
    for (int j=0; j<D; ++j) q[step*D+j]=weights[(step==N ? 4 : 0)+j];
  Eigen::Matrix<double,N,N> difference=Eigen::Matrix<double,N,N>::Identity();
  for (int i=1; i<N; ++i) difference(i,i-1)=-1.;
  Eigen::Matrix<double,N,1> anchor=Eigen::Matrix<double,N,1>::Zero(); anchor[0]=angle;
  const Eigen::Map<const Eigen::Vector4d> initial(initial_state);
  const Eigen::Matrix<double,N,S> weighted_input=input_map.transpose()*q.asDiagonal();
  const Eigen::Matrix<double,N,N,Eigen::RowMajor> hessian=
      weighted_input*input_map + weights[8]*Eigen::Matrix<double,N,N>::Identity()
      + weights[9]*difference.transpose()*difference;
  const Eigen::Matrix<double,N,1> linear=weighted_input*(state_map*initial+affine)
      - weights[9]*difference.transpose()*anchor;
  double lipschitz=2.*std::max(1e-6,hessian.cwiseAbs().rowwise().sum().maxCoeff());
  double warm[N];
  for (int i=0; i<N; ++i) warm[i]=previous[std::min(i+1,N-1)];
  int iterations=mpc_projected_iterations(N,maximum_iterations,hessian.data(),linear.data(),
      warm,angle,steering_limit,first_step,prediction_step,lipschitz,control_output);
  if (iterations < 0) return -3;
  const Eigen::Map<const Eigen::Matrix<double,N,1>> control(control_output);
  const Eigen::Matrix<double,S,1> predicted=state_map*initial+input_map*control+affine;
  if (!predicted.allFinite()) return -3;
  for (int j=0; j<D; ++j) {
    maxima_output[j]=0.;
    for (int step=0; step<=N; ++step)
      maxima_output[j]=std::max(maxima_output[j],std::abs(predicted[step*D+j]));
  }
  return iterations;
}
