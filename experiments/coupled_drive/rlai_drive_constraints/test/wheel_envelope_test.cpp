// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include "rlai_drive_constraints/wheel_envelope.hpp"

void require(bool value) {if(!value) throw std::runtime_error("Wheel invariant failed");}
template<typename T> void check()
{
  rlai_drive_constraints::WheelEnvelope<T> g{T(.3101),T(.0625),T(.0625),T(5)};
  require(g.valid());T v=T(.3),w=T(1);require(g.project(v,w));
  require(std::abs(double(w/v)-1/.3)<1e-5);
  T pv=0,pw=T(1.9);require(g.project(pv,pw));
  T target_v=T(.3125),target_w=0;require(g.project(target_v,target_w));
  for(int i=0;i<130;++i) {
    const T lv=std::clamp(target_v,pv-T(.01),pv+T(.01));
    const T lw=std::clamp(target_w,pw-T(.032),pw+T(.032));
    T nv,nw;require(g.step(pv,pw,target_v,target_w,lv,lw,nv,nw));
    require(std::abs(double(nv-pv))<=.010001&&std::abs(double(nw-pw))<=.032001);
    require(std::max(std::abs((double(nv)-.3101*double(nw)/2)/.0625),std::abs((double(nv)+.3101*double(nw)/2)/.0625))<=5.);
    pv=nv;pw=nw;
  }
  require(std::abs(double(pv-target_v))<1e-6&&std::abs(double(pw))<1e-6);
  std::mt19937 random(17);std::uniform_real_distribution<double> vel(-.35,.5),turn(-1.9,1.9);
  for(int k=0;k<2500;++k) {
    target_v=T(vel(random));target_w=T(turn(random));require(g.project(target_v,target_w));
    for(int j=0;j<20;++j) {
      T nv,nw;require(g.step(pv,pw,target_v,target_w,std::clamp(target_v,pv-T(.01),pv+T(.01)),
        std::clamp(target_w,pw-T(.032),pw+T(.032)),nv,nw));
      require(std::abs(double(nv-pv))<=.010001&&std::abs(double(nw-pw))<=.032001);
      require(std::max(std::abs((double(nv)-.3101*double(nw)/2)/.0625),std::abs((double(nv)+.3101*double(nw)/2)/.0625))<=5.);
      pv=nv;pw=nw;
    }
  }
  T invalid=std::numeric_limits<T>::quiet_NaN(),zero=0;
  require(!g.project(invalid,zero));require(!rlai_drive_constraints::finite(invalid));
  require(!g.project(zero,invalid));g.maximum=T(0);require(!g.valid());
}
int main() {check<float>();check<double>();std::cout<<"float/double: 100260 coordinated steps, bounds, rates, progress and nonfinite checks passed\n";}
