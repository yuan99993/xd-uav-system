// Read-only stream verification; no image publication or tracking input.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <sar_yolo_detector/SmartTrackerState.h>
#include <fstream>
#include <set>
#include <cmath>
#include <algorithm>
#include <vector>
int main(int argc,char** argv) {
  ros::init(argc,argv,"sar_camera_stream_probe"); ros::NodeHandle n,p("~");
  std::string ns,instance,path; double seconds;
  p.param<std::string>("namespace",ns,"ue_test");p.param<std::string>("instance",instance,"dfine_v3");
  p.param<std::string>("output",path,"/tmp/sar_camera_stream_probe.json");p.param("seconds",seconds,15.);
  if (!std::isfinite(seconds)||seconds<1||seconds>120) return 2;
  if (ns.empty()||ns.front()!='/') ns="/"+ns;
  std::size_t images=0,infos=0,states=0,current=0,feedback=0;
  std::set<uint64_t> processed;std::vector<double> ages;
  const auto start=ros::WallTime::now();
  auto image=n.subscribe<sensor_msgs::Image>(ns+"/fixed_camera/image_raw",1,
    [&](const sensor_msgs::Image::ConstPtr& m){if (!m->data.empty()) ++images;},ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto info=n.subscribe<sensor_msgs::CameraInfo>(ns+"/fixed_camera/camera_info",1,
    [&](const sensor_msgs::CameraInfo::ConstPtr&){++infos;},ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto state=n.subscribe<sar_yolo_detector::SmartTrackerState>(ns+"/sar_yolo_detector/"+instance+"/state",1,
    [&](const sar_yolo_detector::SmartTrackerState::ConstPtr& m){
      ++states;processed.insert(m->header.stamp.toNSec());
      if(m->has_selection&&m->control_measurement_ready&&!m->prediction_only&&!m->data_is_stale&&!m->tentative){
        ++current;ages.push_back((ros::Time::now()-m->header.stamp).toSec()*1000.);
      }
    },ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto fb=n.subscribe<std_msgs::String>(ns+"/gimbal/state",1,
    [&](const std_msgs::String::ConstPtr&){++feedback;},ros::VoidConstPtr(),ros::TransportHints().tcpNoDelay());
  auto timer=n.createWallTimer(ros::WallDuration(seconds),[&](const ros::WallTimerEvent&){ros::shutdown();},true);
  ros::spin();const double duration=(ros::WallTime::now()-start).toSec();
  std::sort(ages.begin(),ages.end());std::ofstream out(path);
  if(!out) return 3;
  out<<"{\"duration_sec\":"<<duration<<",\"image_fps\":"<<images/duration
     <<",\"camera_info_fps\":"<<infos/duration<<",\"state_fps\":"<<states/duration
     <<",\"unique_processed_capture_fps\":"<<processed.size()/duration
     <<",\"current_visual_fps\":"<<current/duration<<",\"feedback_fps\":"<<feedback/duration
     <<",\"capture_to_visual_p95_ms\":";
  if(ages.empty())out<<"null";else out<<ages[static_cast<size_t>((ages.size()-1)*.95)];
  out<<",\"max_capture_to_visual_ms\":";
  if(ages.empty())out<<"null";else out<<ages.back();out<<"}\n";
  return 0;
}
