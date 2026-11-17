#pragma once
#include "core.hpp"
namespace frenet {
// Small strictly-convex box QP. Feasible active-set steps preserve every bound;
// convergence is checked with the KKT conditions, never an unchecked iterate.
inline bool corridor_box_qp(const Eigen::MatrixXd& h,const Eigen::VectorXd& linear,
                            const Eigen::VectorXd& lo,const Eigen::VectorXd& hi,
                            Eigen::VectorXd& x,int& iterations) {
  const int n=lo.size();std::vector<int> active(n,0);x=Eigen::VectorXd::Zero(n);
  for(int i=0;i<n;++i) {
    if(!std::isfinite(lo[i]+hi[i]) || lo[i]>hi[i]+1e-9)return false;
    x[i]=clip(0.,lo[i],hi[i]);
    if(hi[i]-lo[i]<1e-9){active[i]=2;x[i]=lo[i];}
    else if(x[i]<=lo[i]+1e-9)active[i]=-1;
    else if(x[i]>=hi[i]-1e-9)active[i]=1;
  }
  for(iterations=0;iterations<8*n+40;++iterations) {
    std::vector<int> free;for(int i=0;i<n;++i)if(!active[i])free.push_back(i);
    Eigen::VectorXd gradient=h*x+linear,direction=Eigen::VectorXd::Zero(n);
    if(!free.empty()) {
      const int m=free.size();Eigen::MatrixXd sub(m,m);Eigen::VectorXd rhs(m);
      for(int i=0;i<m;++i){rhs[i]=-gradient[free[i]];for(int j=0;j<m;++j)sub(i,j)=h(free[i],free[j]);}
      Eigen::LDLT<Eigen::MatrixXd> factor(sub);
      if(factor.info()!=Eigen::Success)return false;
      Eigen::VectorXd delta=factor.solve(rhs);if(!delta.allFinite())return false;
      for(int i=0;i<m;++i)direction[free[i]]=delta[i];
    }
    if(direction.lpNorm<Eigen::Infinity>()<1e-8) {
      int release=-1;double violation=1e-6;
      for(int i=0;i<n;++i)if(active[i]==-1 || active[i]==1) {
        double bad=active[i]==-1?-gradient[i]:gradient[i];
        if(bad>violation){violation=bad;release=i;}
      }
      if(release<0)return true;
      active[release]=0;continue;
    }
    double alpha=1.;int hit=-1,side=0;
    for(int i:free) {
      double ratio=INF;int edge=0;
      if(direction[i]>1e-12){ratio=(hi[i]-x[i])/direction[i];edge=1;}
      else if(direction[i]<-1e-12){ratio=(lo[i]-x[i])/direction[i];edge=-1;}
      if(ratio<alpha){alpha=std::max(0.,ratio);hit=i;side=edge;}
    }
    x+=alpha*direction;
    if(hit>=0){x[hit]=side==1?hi[hit]:lo[hit];active[hit]=side;}
  }
  return false;
}
// Clamped cubic spline gives continuous heading and curvature. Constraints
// at the control stations are followed by dense full-body collision checks;
// spline overshoot is never considered free space just because knots fit.
struct CorridorSpline {
  std::vector<double> s;Eigen::VectorXd d,m;
  CorridorSpline(const std::vector<double>& stations,const Eigen::VectorXd& offsets,double slope):s(stations),d(offsets) {
    int n=s.size();Eigen::MatrixXd matrix=Eigen::MatrixXd::Zero(n,n);Eigen::VectorXd rhs=Eigen::VectorXd::Zero(n);
    double h=s[1]-s[0];matrix(0,0)=2*h;matrix(0,1)=h;rhs[0]=6*((d[1]-d[0])/h-slope);
    for(int i=1;i<n-1;++i) {
      double a=s[i]-s[i-1],b=s[i+1]-s[i];matrix(i,i-1)=a;matrix(i,i)=2*(a+b);matrix(i,i+1)=b;
      rhs[i]=6*((d[i+1]-d[i])/b-(d[i]-d[i-1])/a);
    }
    h=s[n-1]-s[n-2];matrix(n-1,n-2)=h;matrix(n-1,n-1)=2*h;rhs[n-1]=-6*(d[n-1]-d[n-2])/h;
    m=matrix.ldlt().solve(rhs);
  }
  std::array<double,3> at(double t) const {
    int i=std::max(0,std::min(int(s.size())-2,int(std::upper_bound(s.begin(),s.end(),t)-s.begin())-1));
    double h=s[i+1]-s[i],a=(s[i+1]-t)/h,b=(t-s[i])/h;
    return {{a*d[i]+b*d[i+1]+((a*a*a-a)*m[i]+(b*b*b-b)*m[i+1])*h*h/6,
             a*m[i]+b*m[i+1],(m[i+1]-m[i])/h}};
  }
};
} // namespace frenet
