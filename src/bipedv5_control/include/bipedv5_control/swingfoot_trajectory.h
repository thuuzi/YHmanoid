#include "Eigen/Core"
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template<typename T>
class FootSwingTrajectory {
public:


  FootSwingTrajectory() {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  void setInitialPosition(Vec3<T> p0) {
    _p0 = p0;
  }

  void setFinalPosition(Vec3<T> pf) {
    _pf = pf;
  }

  void setHeight(T h) {
    _height = h;
  }

  void computeSwingTrajectoryBezier(T phase, T swingTime);

  

  Vec3<T> getPositon() {
    return _p;
  }

  Vec3<T> getVelocity() {
    return _v;
  }

  Vec3<T> getAcceleration() {
    return _a;
  }

private:
  Vec3<T> _p0, _pf, _p, _v, _a;   //起始点、终点、足端位置、速度、加速度
  T _height;      //抬腿高度
};
