// SPDX-License-Identifier: Apache-2.0
#ifndef RLAI_DRIVE_CONSTRAINTS__WHEEL_ENVELOPE_HPP_
#define RLAI_DRIVE_CONSTRAINTS__WHEEL_ENVELOPE_HPP_
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace rlai_drive_constraints
{
// MPPI uses -ffast-math: classify IEEE bits instead of an erasable isfinite.
template<typename T> inline bool finite(T value)
{
  static_assert(std::is_floating_point<T>::value && (sizeof(T)==4 || sizeof(T)==8));
  if constexpr(sizeof(T)==4) {
    std::uint32_t bits;std::memcpy(&bits,&value,4);
    return (bits & 0x7f800000u)!=0x7f800000u;
  } else {
    std::uint64_t bits;std::memcpy(&bits,&value,8);
    return (bits & 0x7ff0000000000000ULL)!=0x7ff0000000000000ULL;
  }
}

template<typename T> struct WheelEnvelope
{
  T separation{},left_radius{},right_radius{},maximum{};
  bool valid() const
  {
    return finite(separation)&&finite(left_radius)&&finite(right_radius)&&finite(maximum)&&
      separation>0&&left_radius>0&&right_radius>0&&maximum>0;
  }
  T peak(T v,T w) const
  {
    return std::max(std::abs((v-separation*w/T(2))/left_radius),
      std::abs((v+separation*w/T(2))/right_radius));
  }
  bool feasible(T v,T w) const
  {
    return valid()&&finite(v)&&finite(w)&&finite(peak(v,w))&&peak(v,w)<=maximum;
  }
  bool project(T & v,T & w) const
  {
    if(!valid()||!finite(v)||!finite(w)) return false;
    const T demand=peak(v,w);
    if(!finite(demand)) return false;
    // Inward precision margin, never an increased actuator limit. It covers
    // float geometry conversion and convex-interpolation rounding in MPPI.
    const T inner=maximum*(T(1)-T(8)*std::numeric_limits<T>::epsilon());
    if(demand>inner) {const T factor=inner/demand;v*=factor;w*=factor;}
    return feasible(v,w);
  }
  // Targets and previous emitted commands must be feasible. Native axis
  // limiters supply per-axis progress; share its minimum so the result stays
  // on the feasible segment and consumes no more than either axis allowance.
  bool step(T pv,T pw,T tv,T tw,T limited_v,T limited_w,T & out_v,T & out_w) const
  {
    out_v=T(0);out_w=T(0);
    if(!feasible(pv,pw)||!feasible(tv,tw)||!finite(limited_v)||!finite(limited_w)) return false;
    T progress=T(1);
    if(tv!=pv) progress=std::min(progress,std::clamp((limited_v-pv)/(tv-pv),T(0),T(1)));
    if(tw!=pw) progress=std::min(progress,std::clamp((limited_w-pw)/(tw-pw),T(0),T(1)));
    out_v=pv+progress*(tv-pv);out_w=pw+progress*(tw-pw);
    return feasible(out_v,out_w);
  }
};
}  // namespace rlai_drive_constraints
#endif
