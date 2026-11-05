"""Coordinate and route invariants used by the packaged PX4/UE adapter."""
import math
import unittest
from xd_uav_track.mission import Mission
from xd_uav_track.px4_pose import enu_to_map, map_to_enu, ue_rotation_deg


class AirgroundMissionTest(unittest.TestCase):
    def test_ned_enu_round_trip_with_nonzero_origin(self):
        origin=[12.,-3.,7.];home=[-55.,-35.,-.7]
        for i in range(30):
            point=[i*.3,i*-.7,-i*.8]
            returned=enu_to_map(map_to_enu(point,origin,home),origin,home)
            for a,b in zip(point,returned):self.assertAlmostEqual(a,b)

    def test_px4_heading_maps_to_ue_heading(self):
        angles=ue_rotation_deg([0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
        self.assertAlmostEqual(angles['yaw'],0.)
        self.assertAlmostEqual(angles['roll'],0.)
        self.assertAlmostEqual(angles['pitch'],0.)

    def test_route_returns_home_and_has_finite_continuous_poses(self):
        mission=Mission(dict(home_ned_m=[0,0,-.7],cruise_altitude_m=10,
            vertical_speed_mps=2,speed_mps=4,hover_sec=3,waypoints_ned_m=[[8,0,-10]]))
        previous=mission.sample(0)[0]
        for i in range(1,int(mission.duration*100)+1):
            point,yaw,_=mission.sample(i*.01)
            self.assertTrue(all(math.isfinite(v) for v in point+[yaw]))
            self.assertLessEqual(math.dist(previous,point),.040001)
            previous=point
        self.assertEqual(mission.sample(mission.duration+1)[0],[0,0,-.7])


if __name__=='__main__':unittest.main()
