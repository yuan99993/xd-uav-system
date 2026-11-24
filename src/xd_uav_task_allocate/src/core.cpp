#include <xd_uav_task_allocate/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace xd_uav_task_allocate {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

bool finite(const Vec3& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

double distance2d(const Vec3& a, const Vec3& b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

Vec3 geodeticToEcef(double latitude, double longitude, double altitude) {
  const double phi = latitude * kPi / 180.0;
  const double lam = longitude * kPi / 180.0;
  const double sp = std::sin(phi), cp = std::cos(phi);
  const double radius = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sp * sp);
  return {{(radius + altitude) * cp * std::cos(lam),
           (radius + altitude) * cp * std::sin(lam),
           (radius * (1.0 - kWgs84E2) + altitude) * sp}};
}

std::array<std::array<double, 3>, 3> rotationMatrix(Quaternion q) {
  double n = 0.0;
  for (const auto v : q) n += v * v;
  n = std::sqrt(n);
  if (!std::isfinite(n) || n <= 1e-9) {
    throw std::invalid_argument("quaternion norm is zero or invalid");
  }
  for (auto& v : q) v /= n;
  const auto x = q[0], y = q[1], z = q[2], w = q[3];
  return {{{1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)},
           {2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)},
           {2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)}}};
}

double horizontalVariance(const Covariance3& c) {
  const double xx = c[0], xy = 0.5 * (c[1] + c[3]), yy = c[4];
  if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy)) return 0.0;
  const double d = std::max(0.0, (xx - yy) * (xx - yy) + 4.0 * xy * xy);
  return std::max(0.0, 0.5 * (xx + yy + std::sqrt(d)));
}

}  // namespace

Vec3 rotateVector(const Quaternion& quaternion, const Vec3& vector) {
  if (!finite(vector)) throw std::invalid_argument("vector contains non-finite values");
  double qx = quaternion[0], qy = quaternion[1], qz = quaternion[2], qw = quaternion[3];
  double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    throw std::invalid_argument("quaternion norm is zero or invalid");
  }
  qx /= norm; qy /= norm; qz /= norm; qw /= norm;
  const double tx = 2.0 * (qy*vector[2] - qz*vector[1]);
  const double ty = 2.0 * (qz*vector[0] - qx*vector[2]);
  const double tz = 2.0 * (qx*vector[1] - qy*vector[0]);
  return {{vector[0] + qw*tx + qy*tz - qz*ty,
           vector[1] + qw*ty + qz*tx - qx*tz,
           vector[2] + qw*tz + qx*ty - qy*tx}};
}

Vec3 RigidTransform::applyVector(const Vec3& vector) const {
  return rotateVector(rotation, vector);
}

Vec3 RigidTransform::apply(const Vec3& point) const {
  const auto r = applyVector(point);
  return {{translation[0] + r[0], translation[1] + r[1], translation[2] + r[2]}};
}

Vec3 relativeFrdToShared(const Vec3& relative_frd, const PoseSample& pose,
                         const RigidTransform& world_from_reference) {
  const Vec3 flu{{relative_frd[0], -relative_frd[1], -relative_frd[2]}};
  const auto relative = rotateVector(pose.orientation, flu);
  return world_from_reference.apply(
      {{pose.position[0] + relative[0], pose.position[1] + relative[1],
        pose.position[2] + relative[2]}});
}

Covariance3 rotateCovarianceFrdToShared(const Covariance3& covariance,
                                       const Quaternion& body_orientation,
                                       const Quaternion& reference_to_world) {
  for (const auto value : covariance) {
    if (!std::isfinite(value)) return {};
  }
  auto body = rotationMatrix(body_orientation);
  auto shared = rotationMatrix(reference_to_world);
  const double frd_flu[3][3] = {{1,0,0},{0,-1,0},{0,0,-1}};
  double t[3][3]{}, tmp[3][3]{}, out[3][3]{};
  for (int r=0;r<3;++r) for (int c=0;c<3;++c)
    for (int k=0;k<3;++k) tmp[r][c] += body[r][k] * frd_flu[k][c];
  for (int r=0;r<3;++r) for (int c=0;c<3;++c)
    for (int k=0;k<3;++k) t[r][c] += shared[r][k] * tmp[k][c];
  for (int r=0;r<3;++r) for (int c=0;c<3;++c)
    for (int i=0;i<3;++i) for (int j=0;j<3;++j)
      out[r][c] += t[r][i] * covariance[3*i+j] * t[c][j];
  Covariance3 result{};
  for (int r=0;r<3;++r) for (int c=0;c<3;++c) result[3*r+c] = out[r][c];
  return result;
}

Covariance3 rotateEnuCovarianceToWorld(const Covariance3& covariance,double yaw) {
  for(const auto value:covariance)if(!std::isfinite(value))return {};
  if(!std::isfinite(yaw))return {};
  const double c=std::cos(yaw),s=std::sin(yaw);
  const double r[3][3]={{c,-s,0},{s,c,0},{0,0,1}};
  Covariance3 out{};
  for(int row=0;row<3;++row)for(int col=0;col<3;++col)
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)
      out[3*row+col]+=r[row][i]*covariance[3*i+j]*r[col][j];
  return out;
}

PoseHistory::PoseHistory(double maximum_age_sec, std::size_t maximum_size)
    : maximum_age_sec_(std::max(0.1, maximum_age_sec)),
      maximum_size_(std::max<std::size_t>(2, maximum_size)) {}

void PoseHistory::add(const PoseSample& sample) {
  if (!std::isfinite(sample.stamp) || sample.stamp <= 0.0 || !finite(sample.position)) return;
  samples_.push_back(sample);
  while (samples_.size() > maximum_size_) samples_.erase(samples_.begin());
  const double newest = samples_.back().stamp;
  while (!samples_.empty() && newest - samples_.front().stamp > maximum_age_sec_)
    samples_.erase(samples_.begin());
}

std::optional<PoseSample> PoseHistory::closest(double stamp, double tolerance) const {
  if (samples_.empty() || !std::isfinite(stamp) || stamp <= 0.0) return std::nullopt;
  auto it = std::min_element(samples_.begin(), samples_.end(), [stamp](const auto& a, const auto& b) {
    return std::abs(a.stamp - stamp) < std::abs(b.stamp - stamp);
  });
  if (std::abs(it->stamp - stamp) > std::max(0.0, tolerance)) return std::nullopt;
  return *it;
}

std::optional<PoseSample> PoseHistory::latest() const {
  if (samples_.empty()) return std::nullopt;
  return samples_.back();
}

Vec3 geodeticToWorld(double latitude, double longitude, double altitude,
                     const Vec3& datum_lla, double yaw, const Vec3& world_position) {
  for (const double value : {latitude, longitude, altitude, datum_lla[0], datum_lla[1],
                             datum_lla[2], yaw}) {
    if (!std::isfinite(value)) throw std::invalid_argument("geodetic input is non-finite");
  }
  if (std::abs(latitude) > 90.0 || std::abs(datum_lla[0]) > 90.0 ||
      std::abs(longitude) > 180.0 || std::abs(datum_lla[1]) > 180.0) {
    throw std::invalid_argument("latitude or longitude is out of range");
  }
  const auto p = geodeticToEcef(latitude, longitude, altitude);
  const auto d = geodeticToEcef(datum_lla[0], datum_lla[1], datum_lla[2]);
  const double lat0 = datum_lla[0] * kPi / 180.0, lon0 = datum_lla[1] * kPi / 180.0;
  const Vec3 delta{{p[0]-d[0], p[1]-d[1], p[2]-d[2]}};
  const Vec3 enu{{-std::sin(lon0)*delta[0] + std::cos(lon0)*delta[1],
                  -std::sin(lat0)*std::cos(lon0)*delta[0] - std::sin(lat0)*std::sin(lon0)*delta[1] + std::cos(lat0)*delta[2],
                   std::cos(lat0)*std::cos(lon0)*delta[0] + std::cos(lat0)*std::sin(lon0)*delta[1] + std::sin(lat0)*delta[2]}};
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {{world_position[0] + c*enu[0] - s*enu[1],
           world_position[1] + s*enu[0] + c*enu[1],
           world_position[2] + enu[2]}};
}

std::vector<Vec3> lawnmowerPath(const SearchAreaDefinition& area) {
  if (area.boundary.size() < 3) throw std::invalid_argument("area boundary needs at least three points");
  const double spacing = area.lane_spacing;
  if (!std::isfinite(spacing) || spacing <= 0.0)
    throw std::invalid_argument("lane_spacing must be positive");
  double min_x = area.boundary.front()[0], max_x = min_x;
  double min_y = area.boundary.front()[1], max_y = min_y;
  for (const auto& p : area.boundary) {
    min_x = std::min(min_x, p[0]); max_x = std::max(max_x, p[0]);
    min_y = std::min(min_y, p[1]); max_y = std::max(max_y, p[1]);
  }
  const bool horizontal = (max_x-min_x) >= (max_y-min_y);
  const double cross_min = horizontal ? min_y : min_x;
  const double cross_max = horizontal ? max_y : max_x;
  double scan = cross_min + std::min(0.5*spacing, 0.5*(cross_max-cross_min));
  auto intersections = [&](double value) {
    std::vector<double> values;
    for (std::size_t i=0;i<area.boundary.size();++i) {
      const auto& a = area.boundary[i];
      const auto& b = area.boundary[(i+1)%area.boundary.size()];
      const double ac = horizontal ? a[1] : a[0];
      const double bc = horizontal ? b[1] : b[0];
      const double aa = horizontal ? a[0] : a[1];
      const double ba = horizontal ? b[0] : b[1];
      if (!((ac <= value && value < bc) || (bc <= value && value < ac))) continue;
      const double ratio=(value-ac)/(bc-ac);
      values.push_back(aa+ratio*(ba-aa));
    }
    std::sort(values.begin(),values.end());
    return values;
  };
  std::vector<std::pair<Vec3,Vec3>> segments;
  while (scan < cross_max + 1e-9) {
    const auto values=intersections(scan);
    for (std::size_t i=0;i+1<values.size();i+=2) {
      if (values[i+1]-values[i]<=1e-6) continue;
      if(horizontal) segments.push_back({{{values[i],scan,area.altitude}},{{values[i+1],scan,area.altitude}}});
      else segments.push_back({{{scan,values[i],area.altitude}},{{scan,values[i+1],area.altitude}}});
    }
    scan += spacing;
  }
  std::vector<Vec3> path;bool reverse=false;
  for(const auto& segment:segments) {
    if(reverse){path.push_back(segment.second);path.push_back(segment.first);}
    else {path.push_back(segment.first);path.push_back(segment.second);}
    reverse=!reverse;
  }
  if (path.empty()) throw std::invalid_argument("search area is too small for the requested lane spacing");
  return path;
}

namespace {
double mod2pi(double angle) { return std::fmod(angle,2.0*kPi)+((std::fmod(angle,2.0*kPi)<0)?2.0*kPi:0.0); }

struct DubinsCandidate { double length; std::string modes; std::array<double,3> params; };

std::vector<DubinsCandidate> dubinsCandidates(double alpha,double beta,double d) {
  const double sa=std::sin(alpha),sb=std::sin(beta),ca=std::cos(alpha),cb=std::cos(beta),cd=std::cos(alpha-beta);
  std::vector<DubinsCandidate> out;
  auto add=[&](const std::string& modes,double first,double second2,double third){
    if(second2 < -1e-9) return;
    std::array<double,3> p{{mod2pi(first),std::sqrt(std::max(0.0,second2)),mod2pi(third)}};
    out.push_back({p[0]+p[1]+p[2],modes,p});};
  double q=2+d*d-2*cd+2*d*(sa-sb),tmp=std::atan2(cb-ca,d+sa-sb);
  add("LSL",-alpha+tmp,q,beta-tmp);
  q=2+d*d-2*cd+2*d*(sb-sa);tmp=std::atan2(ca-cb,d-sa+sb);
  add("RSR",alpha-tmp,q,-beta+tmp);
  q=-2+d*d+2*cd+2*d*(sa+sb);
  if(q>=-1e-9){const double second=std::sqrt(std::max(0.0,q));tmp=std::atan2(-ca-cb,d+sa+sb)-std::atan2(-2.0,second);
    std::array<double,3> p{{mod2pi(-alpha+tmp),second,mod2pi(-beta+tmp)}};out.push_back({p[0]+p[1]+p[2],"LSR",p});}
  q=d*d-2+2*cd-2*d*(sa+sb);
  if(q>=-1e-9){const double second=std::sqrt(std::max(0.0,q));tmp=std::atan2(ca+cb,d-sa-sb)-std::atan2(2.0,second);
    std::array<double,3> p{{mod2pi(alpha-tmp),second,mod2pi(beta-tmp)}};out.push_back({p[0]+p[1]+p[2],"RSL",p});}
  double value=(6-d*d+2*cd+2*d*(sa-sb))/8;
  if(std::abs(value)<=1+1e-9){const double second=mod2pi(2*kPi-std::acos(std::clamp(value,-1.0,1.0)));
    const double first=mod2pi(alpha-std::atan2(ca-cb,d-sa+sb)+0.5*second);
    std::array<double,3> p{{first,second,mod2pi(alpha-beta-first+second)}};out.push_back({p[0]+p[1]+p[2],"RLR",p});}
  value=(6-d*d+2*cd+2*d*(-sa+sb))/8;
  if(std::abs(value)<=1+1e-9){const double second=mod2pi(2*kPi-std::acos(std::clamp(value,-1.0,1.0)));
    const double first=mod2pi(-alpha-std::atan2(ca-cb,d+sa-sb)+0.5*second);
    std::array<double,3> p{{first,second,mod2pi(beta-alpha-first+second)}};out.push_back({p[0]+p[1]+p[2],"LRL",p});}
  return out;
}

std::array<double,3> advanceDubins(const std::array<double,3>& pose,char mode,double amount,double radius) {
  const auto x=pose[0],y=pose[1],h=pose[2];
  if(mode=='S'){const double d=amount*radius;return {{x+d*std::cos(h),y+d*std::sin(h),h}};}
  if(mode=='L'){const double cx=x-radius*std::sin(h),cy=y+radius*std::cos(h),nh=h+amount;return {{cx+radius*std::sin(nh),cy-radius*std::cos(nh),nh}};}
  const double cx=x+radius*std::sin(h),cy=y-radius*std::cos(h),nh=h-amount;return {{cx-radius*std::sin(nh),cy+radius*std::cos(nh),nh}};
}
} // namespace

std::vector<std::array<double,3>> dubinsPath(const std::array<double,3>& start,
    const std::array<double,3>& goal,double radius,double spacing) {
  if(!std::isfinite(radius)||radius<=0)throw std::invalid_argument("minimum_turn_radius must be positive");
  if(!std::isfinite(spacing)||spacing<=0)throw std::invalid_argument("waypoint_spacing must be positive");
  const double dx=goal[0]-start[0],dy=goal[1]-start[1],dxy=std::hypot(dx,dy);
  if(dxy<=1e-9&&std::abs(mod2pi(goal[2]-start[2]))<=1e-9)return {start,goal};
  const double ref=dxy>1e-9?std::atan2(dy,dx):start[2];
  const auto candidates=dubinsCandidates(mod2pi(start[2]-ref),mod2pi(goal[2]-ref),dxy/radius);
  if(candidates.empty())throw std::runtime_error("no feasible Dubins connection was found");
  const auto best=std::min_element(candidates.begin(),candidates.end(),[](const auto&a,const auto&b){return a.length<b.length;});
  std::vector<std::array<double,3>> result{start};auto pose=start;
  for(int i=0;i<3;++i){const char mode=best->modes[i];const double param=best->params[i],length=param*radius;if(length<=1e-9)continue;
    const int steps=std::max(1,static_cast<int>(std::ceil(length/spacing)));double previous=0;
    for(int step=1;step<=steps;++step){const double current=param*step/static_cast<double>(steps);pose=advanceDubins(pose,mode,current-previous,radius);result.push_back(pose);previous=current;}}
  const double pos_err=std::hypot(result.back()[0]-goal[0],result.back()[1]-goal[1]);
  const double heading_err=std::abs(mod2pi(result.back()[2]-goal[2]+kPi)-kPi);
  if(pos_err>std::max(0.05,1e-7*std::max(radius,dxy))||heading_err>1e-3)
    throw std::runtime_error("Dubins integration missed requested endpoint");
  result.back()=goal;return result;
}

std::vector<Vec3> fixedwingLawnmowerPath(const SearchAreaDefinition& area,double radius,
    double spacing,std::optional<double> lead_value) {
  const auto raw=lawnmowerPath(area);if(raw.size()<2)return raw;
  if(radius<=0)throw std::invalid_argument("minimum_turn_radius must be positive");
  const double lead=lead_value.value_or(radius);if(lead<0)throw std::invalid_argument("straight_lead_distance must be non-negative");
  double minx=area.boundary[0][0],maxx=minx,miny=area.boundary[0][1],maxy=miny;
  for(const auto&p:area.boundary){minx=std::min(minx,p[0]);maxx=std::max(maxx,p[0]);miny=std::min(miny,p[1]);maxy=std::max(maxy,p[1]);}
  const bool horizontal=maxx-minx>=maxy-miny;
  std::vector<std::pair<Vec3,Vec3>> segments;
  for(std::size_t i=0;i+1<raw.size();i+=2){auto a=raw[i],b=raw[i+1];const int axis=horizontal?0:1;if(a[axis]>b[axis])std::swap(a,b);segments.push_back({a,b});}
  const std::size_t half=(segments.size()+1)/2;std::vector<std::size_t> order;
  for(std::size_t i=0;i<half;++i){order.push_back(i);if(i+half<segments.size())order.push_back(i+half);}
  std::vector<Vec3> route;std::optional<Vec3> previous;double previous_heading=0;
  for(std::size_t vi=0;vi<order.size();++vi){const auto& seg=segments[order[vi]];Vec3 start=vi%2?seg.second:seg.first,end=vi%2?seg.first:seg.second;
    const double dx=end[0]-start[0],dy=end[1]-start[1],length=std::hypot(dx,dy),heading=std::atan2(dy,dx);
    if(length<=1e-9) continue;
    const double ux=dx/length,uy=dy/length;
    Vec3 approach{{start[0]-lead*ux,start[1]-lead*uy,area.altitude}},departure{{end[0]+lead*ux,end[1]+lead*uy,area.altitude}};
    if(!previous)route.push_back(approach);
    else {const auto connector=dubinsPath({{(*previous)[0],(*previous)[1],previous_heading}},{{approach[0],approach[1],heading}},radius,spacing);
      for(std::size_t i=1;i<connector.size();++i)route.push_back({{connector[i][0],connector[i][1],area.altitude}});}
    route.push_back(start);route.push_back(end);route.push_back(departure);previous=departure;previous_heading=heading;
  }
  return route;
}

std::vector<Vec3> connectFixedwingPaths(const std::vector<std::vector<Vec3>>& paths,
    double radius,double spacing) {
  std::vector<Vec3> combined;
  for(const auto& path:paths){if(path.empty())continue;if(combined.empty()){combined=path;continue;}
    if(combined.size()<2||path.size()<2){combined.insert(combined.end(),path.begin(),path.end());continue;}
    const double h0=std::atan2(combined.back()[1]-combined[combined.size()-2][1],combined.back()[0]-combined[combined.size()-2][0]);
    const double h1=std::atan2(path[1][1]-path[0][1],path[1][0]-path[0][0]);
    const auto connector=dubinsPath({{combined.back()[0],combined.back()[1],h0}},{{path[0][0],path[0][1],h1}},radius,spacing);
    const double z0=combined.back()[2],dz=path[0][2]-z0;const auto count=std::max<std::size_t>(1,connector.size()-1);
    for(std::size_t i=1;i<connector.size();++i)combined.push_back({{connector[i][0],connector[i][1],z0+static_cast<double>(i)/count*dz}});
    combined.insert(combined.end(),path.begin()+1,path.end());
  }return combined;
}

std::vector<Vec3> fixedwingEntryPath(const Vec3& current,double heading,
    const std::vector<Vec3>& route,double radius,double spacing) {
  if(route.size()<2)return route;
  const double h=std::atan2(route[1][1]-route[0][1],route[1][0]-route[0][0]);
  const auto connector=dubinsPath({{current[0],current[1],heading}},{{route[0][0],route[0][1],h}},radius,spacing);
  const double dz=route[0][2]-current[2];const auto count=std::max<std::size_t>(1,connector.size()-1);std::vector<Vec3> out;
  for(std::size_t i=1;i<connector.size();++i)out.push_back({{connector[i][0],connector[i][1],current[2]+static_cast<double>(i)/count*dz}});
  out.insert(out.end(),route.begin()+1,route.end());return out;
}

double pathLength(const std::vector<Vec3>& path) {
  double result = 0.0;
  for (std::size_t i=1;i<path.size();++i) {
    result += std::hypot(path[i][0]-path[i-1][0],path[i][1]-path[i-1][1]);
  }
  return result;
}

std::map<std::string, std::vector<PlannedArea>> assignAreas(
    const std::vector<PlannedArea>& areas,
    const std::map<std::string, Vec3>& positions,
    const std::map<std::string, double>& speeds,
    const std::map<std::pair<std::string, std::uint32_t>, double>& area_lengths) {
  std::map<std::string, std::vector<PlannedArea>> result;
  std::map<std::string, double> load;
  std::map<std::string, Vec2> current;
  std::map<std::string, double> speed;
  for (const auto& item : positions) {result[item.first]={};load[item.first]=0;current[item.first]={{item.second[0],item.second[1]}};speed[item.first]=std::max(0.1,speeds.count(item.first)?speeds.at(item.first):1.0);}
  std::vector<PlannedArea> ordered = areas;
  std::stable_sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) {
    if(a.area.priority!=b.area.priority)return a.area.priority>b.area.priority;
    if(a.length!=b.length)return a.length>b.length;
    return a.area.area_id<b.area.area_id;
  });
  for (const auto& area : ordered) {
    if(area.path.empty())continue;
    std::string best;double best_cost=std::numeric_limits<double>::infinity();
    for (const auto& entry : positions) {
      const auto& first=area.path.front();const double travel=std::hypot(first[0]-current[entry.first][0],first[1]-current[entry.first][1]);
      const double cost=load[entry.first]+travel/speed[entry.first];
      if(cost<best_cost||(cost==best_cost&&entry.first<best)){best_cost=cost;best=entry.first;}
    }
    if (!best.empty()) {
      result[best].push_back(area);
      const auto key=std::make_pair(best,area.area.area_id);const double length=area_lengths.count(key)?area_lengths.at(key):area.length;
      const auto& first=area.path.front();const double travel=std::hypot(first[0]-current[best][0],first[1]-current[best][1]);
      load[best]+=(travel+length)/speed[best];
      current[best]={{area.path.back()[0],area.path.back()[1]}};
    }
  }
  return result;
}

GlobalTargetRegistry::GlobalTargetRegistry(
    double radius, int hits, double window, double stale, int distinct,
    double minimum_span, const std::set<std::string>& profiles,
    double covariance_sigma, double maximum_radius, double cross_class_radius,
    double track_jump, int class_minimum, double class_ratio,
    double maximum_spread, std::size_t history_size, double bbox_iou)
    : association_radius_m_(std::max(0.1, radius)),
      confirmation_hits_(std::max(1, hits)),
      confirmation_window_sec_(std::max(0.1, window)),
      stale_timeout_sec_(std::max(0.1, stale)),
      confirmation_distinct_uavs_(std::max(1, distinct)),
      confirmation_minimum_span_sec_(std::max(0.0, minimum_span)),
      confirmation_mobility_profiles_(profiles),
      association_covariance_sigma_(std::max(0.0, covariance_sigma)),
      maximum_association_radius_m_(std::max(association_radius_m_, maximum_radius)),
      cross_class_association_radius_m_(std::max(0.0, cross_class_radius)),
      stable_track_maximum_jump_m_(std::max(association_radius_m_, track_jump)),
      class_confirmation_minimum_observations_(std::max(1, class_minimum)),
      class_confirmation_minimum_ratio_(std::clamp(class_ratio, 0.0, 1.0)),
      maximum_confirmation_position_spread_m_(std::max(0.0, maximum_spread)),
      position_history_size_(std::max<std::size_t>(3, history_size)),
      duplicate_bbox_iou_threshold_(std::clamp(bbox_iou, 0.0, 1.0)) {
  if (confirmation_mobility_profiles_.empty())
    throw std::invalid_argument("confirmation_mobility_profiles must not be empty");
}

namespace {
using SourceTrackKey = std::tuple<std::string, std::string, int>;

std::optional<SourceTrackKey> sourceTrackKey(const TargetObservation& observation) {
  if (!observation.track_id_is_stable || observation.track_id < 0) return std::nullopt;
  return SourceTrackKey{observation.source_uav, observation.sensor_id, observation.track_id};
}

double bboxIoU(const std::optional<std::array<double, 4>>& first,
               const std::optional<std::array<double, 4>>& second) {
  if (!first || !second) return 0.0;
  const auto& a = *first;
  const auto& b = *second;
  if (std::min({a[2], a[3], b[2], b[3]}) <= 0.0) return 0.0;
  const double left = std::max(a[0] - 0.5*a[2], b[0] - 0.5*b[2]);
  const double top = std::max(a[1] - 0.5*a[3], b[1] - 0.5*b[3]);
  const double right = std::min(a[0] + 0.5*a[2], b[0] + 0.5*b[2]);
  const double bottom = std::min(a[1] + 0.5*a[3], b[1] + 0.5*b[3]);
  const double intersection = std::max(0.0, right-left) * std::max(0.0, bottom-top);
  const double union_area = a[2]*a[3] + b[2]*b[3] - intersection;
  return union_area > 1e-12 ? intersection/union_area : 0.0;
}

struct ClassWinner {
  std::optional<int> id;
  double ratio{0.0};
  int count{0};
  std::map<int,double> scores;
};

ClassWinner classWinner(const std::vector<TargetObservation>& observations) {
  ClassWinner result;
  std::map<int,int> counts;
  for (const auto& observation : observations) {
    const double confidence = std::clamp(observation.confidence, 0.05, 1.0);
    result.scores[observation.class_id] += confidence;
    ++counts[observation.class_id];
  }
  if (result.scores.empty()) return result;
  auto winner = result.scores.begin();
  double total = 0.0;
  for (auto it=result.scores.begin(); it!=result.scores.end(); ++it) {
    total += it->second;
    if (it->second > winner->second) winner = it;
  }
  result.id = winner->first;
  result.count = counts[winner->first];
  result.ratio = total > 1e-9 ? winner->second/total : 0.0;
  return result;
}

double medianValue(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const auto mid = values.size()/2;
  return values.size()%2 ? values[mid] : 0.5*(values[mid-1]+values[mid]);
}
}  // namespace

double GlobalTargetRegistry::associationRadius(
    const GlobalTargetRecord& target, const TargetObservation& observation) const {
  const double sigma = std::sqrt(horizontalVariance(target.covariance) +
                                 horizontalVariance(observation.covariance));
  return std::min(maximum_association_radius_m_,
                  std::max(association_radius_m_, association_covariance_sigma_*sigma));
}

bool GlobalTargetRegistry::sameSourceFrame(
    const GlobalTargetRecord& target, const TargetObservation& observation) const {
  return std::any_of(target.recent_measurements.begin(), target.recent_measurements.end(),
      [&](const auto& item) {
        return item.source_uav == observation.source_uav &&
               std::abs(item.stamp-observation.stamp) <= 1e-6;
      });
}

bool GlobalTargetRegistry::bboxDuplicate(
    const TargetObservation& first, const TargetObservation& second) const {
  return duplicate_bbox_iou_threshold_ > 0.0 &&
         first.class_id == second.class_id &&
         bboxIoU(first.normalized_bbox, second.normalized_bbox) >= duplicate_bbox_iou_threshold_;
}

bool GlobalTargetRegistry::sameFrameDistinctTrack(
    const GlobalTargetRecord& target, const TargetObservation& observation) const {
  const auto key = sourceTrackKey(observation);
  if (!key) return false;
  for (const auto& item : target.recent_measurements) {
    if (item.source_uav != observation.source_uav ||
        std::abs(item.stamp-observation.stamp) > 1e-6) continue;
    const auto other = sourceTrackKey(item);
    if (other && *other != *key && !bboxDuplicate(item, observation)) return true;
  }
  return false;
}

bool GlobalTargetRegistry::sameFrameDuplicateTrack(
    const GlobalTargetRecord& target, const TargetObservation& observation) const {
  const auto key = sourceTrackKey(observation);
  if (!key) return false;
  return std::any_of(target.recent_measurements.begin(), target.recent_measurements.end(),
      [&](const auto& item) {
        const auto other = sourceTrackKey(item);
        return item.source_uav == observation.source_uav &&
               std::abs(item.stamp-observation.stamp) <= 1e-6 &&
               other && *other != *key && bboxDuplicate(item, observation);
      });
}

bool GlobalTargetRegistry::compatible(
    const GlobalTargetRecord& target, const TargetObservation& observation) const {
  if (target.status == TARGET_STALE) return false;
  const double distance = distance2d(target.position, observation.position);
  if (sameFrameDistinctTrack(target, observation)) return false;
  if (sameFrameDuplicateTrack(target, observation))
    return distance <= stable_track_maximum_jump_m_;
  const bool class_conflict = target.class_id >= 0 && observation.class_id >= 0 &&
                              target.class_id != observation.class_id;
  if (!class_conflict) return distance <= associationRadius(target, observation);
  if (sameSourceFrame(target, observation)) return false;
  return distance <= std::min(cross_class_association_radius_m_,
                              associationRadius(target, observation));
}

GlobalTargetRecord* GlobalTargetRegistry::findTarget(const TargetObservation& observation) {
  const auto key = sourceTrackKey(observation);
  if (key) {
    const auto mapped = source_track_to_target_.find(*key);
    if (mapped != source_track_to_target_.end()) {
      auto target = targets_.find(mapped->second);
      if (target != targets_.end() && target->second.status != TARGET_STALE &&
          distance2d(target->second.position, observation.position) <=
              stable_track_maximum_jump_m_) return &target->second;
    }
  }
  GlobalTargetRecord* best = nullptr;
  double best_distance = std::numeric_limits<double>::infinity();
  for (auto& entry : targets_) {
    auto& target = entry.second;
    if (!compatible(target, observation)) continue;
    const double distance = distance2d(target.position, observation.position);
    if (distance < best_distance ||
        (distance == best_distance && best && target.target_id < best->target_id)) {
      best = &target;
      best_distance = distance;
    }
  }
  return best;
}

bool GlobalTargetRegistry::positionStable(
    const std::vector<TargetObservation>& observations) const {
  if (observations.size() <
      static_cast<std::size_t>(class_confirmation_minimum_observations_)) return false;
  std::vector<double> xs, ys;
  for (const auto& observation : observations) {
    xs.push_back(observation.position[0]);
    ys.push_back(observation.position[1]);
  }
  const double center_x = medianValue(xs);
  const double center_y = medianValue(ys);
  double spread = 0.0;
  for (const auto& observation : observations)
    spread = std::max(spread, std::hypot(observation.position[0]-center_x,
                                         observation.position[1]-center_y));
  return spread <= maximum_confirmation_position_spread_m_;
}

void GlobalTargetRegistry::refreshClassIdentity(GlobalTargetRecord& target) const {
  const auto winner = classWinner(target.recent_measurements);
  target.class_scores = winner.scores;
  if (winner.id && (target.status == TARGET_CANDIDATE ||
                    target.status == TARGET_VERIFYING)) target.class_id = *winner.id;
}

RegistryUpdate GlobalTargetRegistry::observe(const TargetObservation& input) {
  TargetObservation observation = input;
  observation.mobility_profile =
      observation.mobility_profile == "fixedwing" ? "fixedwing" : "hover";
  if (!std::isfinite(observation.stamp) || observation.stamp <= 0.0 ||
      !finite(observation.position))
    throw std::invalid_argument("invalid observation timestamp or position");

  GlobalTargetRecord* target = findTarget(observation);
  const bool created = target == nullptr;
  if (!target) {
    GlobalTargetRecord record;
    record.target_id = next_target_id_;
    record.class_id = observation.class_id;
    record.position = observation.position;
    record.covariance = observation.covariance;
    record.confidence = std::clamp(observation.confidence, 0.05, 1.0);
    record.first_seen = observation.stamp;
    record.last_seen = observation.stamp;
    record.observation_count = 1;
    record.observer_uavs.insert(observation.source_uav);
    record.recent_observations.emplace_back(observation.stamp, observation.source_uav);
    const double variance = horizontalVariance(observation.covariance);
    record.accumulated_weight = variance <= 1e-9
        ? record.confidence
        : std::max(1e-3, record.confidence/std::max(1.0, variance));
    record.observer_mobility_profiles[observation.source_uav] = observation.mobility_profile;
    record.recent_measurements.push_back(observation);
    const auto key = sourceTrackKey(observation);
    if (key) {
      record.source_tracks.insert(*key);
      source_track_to_target_[*key] = record.target_id;
    }
    record.class_scores[observation.class_id] = record.confidence;
    for (auto& other : targets_) {
      if (sameFrameDistinctTrack(other.second, observation)) {
        record.known_distinct_target_ids.insert(other.first);
        other.second.known_distinct_target_ids.insert(record.target_id);
      }
    }
    const auto inserted = targets_.emplace(record.target_id, std::move(record));
    target = &inserted.first->second;
    ++next_target_id_;
  } else {
    const bool same_frame = sameSourceFrame(*target, observation);
    const double confidence = std::clamp(observation.confidence, 0.05, 1.0);
    const double variance = horizontalVariance(observation.covariance);
    const double weight = variance <= 1e-9
        ? confidence : std::max(1e-3, confidence/std::max(1.0, variance));
    if (!same_frame) {
      const double old_weight = std::min(100.0, target->accumulated_weight);
      const double total_weight = old_weight + weight;
      for (int axis=0; axis<3; ++axis)
        target->position[axis] = (old_weight*target->position[axis] +
                                  weight*observation.position[axis])/total_weight;
      for (int index=0; index<9; ++index)
        target->covariance[index] = (old_weight*target->covariance[index] +
                                    weight*observation.covariance[index])/total_weight;
      target->accumulated_weight = total_weight;
      target->confidence = std::min(1.0, 0.75*confidence + 0.25*target->confidence);
      target->last_seen = std::max(target->last_seen, observation.stamp);
      ++target->observation_count;
      target->observer_uavs.insert(observation.source_uav);
      target->observer_mobility_profiles[observation.source_uav] = observation.mobility_profile;
      target->recent_observations.emplace_back(observation.stamp, observation.source_uav);
      target->recent_measurements.push_back(observation);
    }
    const auto key = sourceTrackKey(observation);
    if (key) {
      target->source_tracks.insert(*key);
      source_track_to_target_[*key] = target->target_id;
    }
  }

  const double cutoff = observation.stamp-confirmation_window_sec_;
  auto& recent = target->recent_observations;
  recent.erase(std::remove_if(recent.begin(), recent.end(),
      [cutoff](const auto& item){return item.first < cutoff;}), recent.end());
  std::sort(recent.begin(), recent.end());
  auto& measurements = target->recent_measurements;
  measurements.erase(std::remove_if(measurements.begin(), measurements.end(),
      [cutoff](const auto& item){return item.stamp < cutoff;}), measurements.end());
  std::sort(measurements.begin(), measurements.end(), [](const auto& a,const auto& b) {
    return std::tie(a.stamp,a.source_uav) < std::tie(b.stamp,b.source_uav);
  });
  if (measurements.size() > position_history_size_)
    measurements.erase(measurements.begin(), measurements.end()-position_history_size_);
  refreshClassIdentity(*target);

  std::set<std::string> recent_sources;
  for (const auto& item : recent) recent_sources.insert(item.second);
  const double observation_span = recent.size() >= 2
      ? recent.back().first-recent.front().first : 0.0;
  const bool evidence_ready =
      (recent.size() >= static_cast<std::size_t>(confirmation_hits_) &&
       observation_span >= confirmation_minimum_span_sec_) ||
      recent_sources.size() >= static_cast<std::size_t>(confirmation_distinct_uavs_);
  const bool newly_evidence_ready = evidence_ready && !target->coarse_evidence_ready;
  if (newly_evidence_ready) target->coarse_evidence_ready = true;

  std::vector<std::pair<double,std::string>> confirmable_observations;
  std::set<std::string> confirmable_sources;
  for (const auto& item : recent) {
    auto profile = target->observer_mobility_profiles.find(item.second);
    const auto value = profile == target->observer_mobility_profiles.end()
        ? std::string("hover") : profile->second;
    if (confirmation_mobility_profiles_.count(value)) {
      confirmable_observations.push_back(item);
      confirmable_sources.insert(item.second);
    }
  }
  std::vector<TargetObservation> confirmable_measurements;
  for (const auto& item : measurements)
    if (confirmation_mobility_profiles_.count(item.mobility_profile))
      confirmable_measurements.push_back(item);
  const auto winner = classWinner(confirmable_measurements);
  const bool class_stable = winner.id &&
      winner.count >= class_confirmation_minimum_observations_ &&
      winner.ratio >= class_confirmation_minimum_ratio_;
  const bool position_stable = positionStable(confirmable_measurements);
  const double confirmable_span = confirmable_observations.size() >= 2
      ? confirmable_observations.back().first-confirmable_observations.front().first : 0.0;
  const bool already_confirmed = target->status != TARGET_CANDIDATE &&
                                 target->status != TARGET_VERIFYING;
  bool newly_confirmed = false;
  if (!already_confirmed && class_stable && position_stable &&
      ((confirmable_observations.size() >= static_cast<std::size_t>(confirmation_hits_) &&
        confirmable_span >= confirmation_minimum_span_sec_) ||
       confirmable_sources.size() >= static_cast<std::size_t>(confirmation_distinct_uavs_))) {
    target->class_id = *winner.id;
    target->status = TARGET_CONFIRMED;
    newly_confirmed = true;
  }
  return {*target, created, newly_confirmed, newly_evidence_ready};
}

bool GlobalTargetRegistry::setStatus(std::uint32_t id, TargetStatus status) {
  auto it=targets_.find(id);
  if(it==targets_.end())return false;
  it->second.status=status;
  return true;
}

bool GlobalTargetRegistry::remove(std::uint32_t id) {
  auto it=targets_.find(id);
  if(it==targets_.end())return false;
  const auto tracks=it->second.source_tracks;
  targets_.erase(it);
  for(const auto& track:tracks) {
    auto mapped=source_track_to_target_.find(track);
    if(mapped!=source_track_to_target_.end()&&mapped->second==id)
      source_track_to_target_.erase(mapped);
  }
  for(auto& target:targets_)target.second.known_distinct_target_ids.erase(id);
  return true;
}

void GlobalTargetRegistry::clear(bool reset_ids) {
  targets_.clear();
  source_track_to_target_.clear();
  if(reset_ids)next_target_id_=1;
}

std::vector<std::uint32_t> GlobalTargetRegistry::expire(double now) {
  std::vector<std::uint32_t> expired;
  for(auto& item:targets_) {
    if(item.second.status==TARGET_CANDIDATE&&now-item.second.last_seen>stale_timeout_sec_) {
      item.second.status=TARGET_STALE;
      expired.push_back(item.first);
    }
  }
  return expired;
}

void RescueTaskAllocator::registerWorker(const std::string& name) {
  WorkerRecord record;
  record.name=name;
  workers_.emplace(name,std::move(record));
}

void RescueTaskAllocator::updateWorker(const std::string& name, const Vec3& position,
                                       double stamp, bool online) {
  registerWorker(name); auto& w = workers_.at(name);
  w.position = position; w.last_update = stamp; w.online = online;
}

RescueTaskRecord& RescueTaskAllocator::ensureTask(const GlobalTargetRecord& target,
                                                  int priority, double duplicate_radius) {
  auto existing = target_to_task_.find(target.target_id);
  if (existing != target_to_task_.end()) {
    auto& task=tasks_.at(existing->second);
    if(task.target_id!=target.target_id)return task;
    if(task.status==TASK_PENDING||task.status==TASK_ASSIGNED){
      task.target_position=target.position;
      if(task.status==TASK_PENDING)task.goal=target.position;
    }
    return task;
  }
  if (target.status != TARGET_CONFIRMED) throw std::invalid_argument("task requires a confirmed target");
  if (duplicate_radius > 0) {
    for (auto& item : tasks_) {
      auto& t = item.second;
      if (t.class_id == target.class_id &&
          !target.known_distinct_target_ids.count(t.target_id) &&
          distance2d(t.target_position, target.position) <= duplicate_radius) {
        target_to_task_[target.target_id] = t.task_id; return t;
      }
    }
  }
  RescueTaskRecord task;
  task.task_id=next_task_id_++; task.target_id=target.target_id; task.class_id=target.class_id;
  task.target_position=target.position; task.goal=target.position; task.priority=priority;
  auto inserted = tasks_.emplace(task.task_id, std::move(task));
  target_to_task_[target.target_id] = inserted.first->first;
  return inserted.first->second;
}

std::vector<Assignment> RescueTaskAllocator::assignPending() {
  std::vector<RescueTaskRecord*> pending;
  for (auto& item : tasks_) if (item.second.status == TASK_PENDING) pending.push_back(&item.second);
  std::stable_sort(pending.begin(), pending.end(), [](auto a, auto b) {
    if(a->priority!=b->priority)return a->priority>b->priority;
    return a->task_id<b->task_id;
  });
  std::vector<Assignment> out;
  for (auto* task : pending) {
    WorkerRecord* best=nullptr; double dmin=std::numeric_limits<double>::infinity();
    for (auto& entry : workers_) {
      auto& w=entry.second; if (!w.online || w.assigned_task) continue;
      const auto d=distance2d(w.position,task->target_position);
      if (d<dmin || (d==dmin && best && w.name<best->name)) {best=&w;dmin=d;}
    }
    if (!best) continue;
    task->assigned_worker=best->name; task->status=TASK_ASSIGNED;
    task->detail="assigned; waiting for planner execution"; best->assigned_task=task->task_id;
    out.push_back({*task,*best});
  }
  return out;
}

bool RescueTaskAllocator::markExecuting(std::uint32_t id) {
  auto it=tasks_.find(id); if (it==tasks_.end() || it->second.status!=TASK_ASSIGNED) return false;
  it->second.status=TASK_EXECUTING; it->second.detail="planner trajectory active"; return true;
}

std::optional<RescueTaskRecord> RescueTaskAllocator::complete(std::uint32_t id) {
  auto it=tasks_.find(id); if (it==tasks_.end() || (it->second.status!=TASK_ASSIGNED && it->second.status!=TASK_EXECUTING)) return std::nullopt;
  auto& t=it->second; t.status=TASK_COMPLETED; t.detail="worker reached the rescue task goal";
  auto w=workers_.find(t.assigned_worker); if (w!=workers_.end() && w->second.assigned_task==id) w->second.assigned_task.reset();
  return t;
}

std::optional<RescueTaskRecord> RescueTaskAllocator::releaseWorker(const std::string& name,
                                                                   const std::string& detail) {
  auto w=workers_.find(name); if (w==workers_.end() || !w->second.assigned_task) return std::nullopt;
  auto t=tasks_.find(*w->second.assigned_task); w->second.assigned_task.reset();
  if (t==tasks_.end() || t->second.status==TASK_COMPLETED) return std::nullopt;
  t->second.status=TASK_PENDING; t->second.assigned_worker.clear(); t->second.goal=t->second.target_position; t->second.detail=detail; return t->second;
}

std::vector<RescueTaskRecord> RescueTaskAllocator::markOfflineWorkers(double now, double timeout) {
  std::vector<RescueTaskRecord> released;
  for (auto& item : workers_) if (item.second.assigned_task && now-item.second.last_update > timeout) {
    auto task=releaseWorker(item.first,"worker state timed out; task returned to queue");
    if (task) released.push_back(*task);
  }
  return released;
}

std::optional<RescueTaskRecord> RescueTaskAllocator::cancelTask(std::uint32_t id,
                                                                const std::string& detail) {
  auto it=tasks_.find(id); if (it==tasks_.end() || it->second.status==TASK_COMPLETED) return std::nullopt;
  auto& t=it->second; auto w=workers_.find(t.assigned_worker);
  if (w!=workers_.end() && w->second.assigned_task==id) w->second.assigned_task.reset();
  t.assigned_worker.clear();t.status=TASK_FAILED; t.detail=detail; return t;
}

std::optional<RescueTaskRecord> RescueTaskAllocator::retryTask(std::uint32_t id,
                                                               const std::string& detail) {
  auto it=tasks_.find(id); if (it==tasks_.end() || it->second.status==TASK_COMPLETED) return std::nullopt;
  auto& t=it->second; auto w=workers_.find(t.assigned_worker);
  if (w!=workers_.end() && w->second.assigned_task==id) w->second.assigned_task.reset();
  t.status=TASK_PENDING; t.assigned_worker.clear(); t.goal=t.target_position; t.detail=detail; return t;
}

std::optional<RescueTaskRecord> RescueTaskAllocator::removeTargetTask(std::uint32_t target_id) {
  auto alias=target_to_task_.find(target_id); if (alias==target_to_task_.end()) return std::nullopt;
  const auto id=alias->second; target_to_task_.erase(alias);
  auto task=tasks_.find(id); if (task==tasks_.end()) return std::nullopt;
  if(task->second.target_id!=target_id)return std::nullopt;
  auto w=workers_.find(task->second.assigned_worker);
  if (w!=workers_.end() && w->second.assigned_task==id) w->second.assigned_task.reset();
  auto result=task->second; tasks_.erase(task); return result;
}

void RescueTaskAllocator::clearTasks(bool reset_ids) {
  tasks_.clear(); target_to_task_.clear();
  for (auto& item : workers_) item.second.assigned_task.reset();
  if (reset_ids) next_task_id_=1;
}

}  // namespace xd_uav_task_allocate
