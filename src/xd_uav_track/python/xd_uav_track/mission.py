"""Deterministic NED waypoint mission; kinematic simulation, no flight controller."""
import math


class Mission:
    def __init__(self, config):
        self.config = config
        if any(not math.isfinite(config[k]) or config[k] <= 0 for k in
               ('cruise_altitude_m','vertical_speed_mps','speed_mps','hover_sec')):
            raise ValueError('Mission altitude, speeds and hover duration must be positive')
        points=[config['home_ned_m']]+config['waypoints_ned_m']
        if not config['waypoints_ned_m'] or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('Mission needs finite 3D home and waypoints')
        if any(p[2]>=0 for p in points):
            raise ValueError('NED vehicle centres must remain above ground (negative Z)')
        self.home = list(config['home_ned_m'])
        hover_by_waypoint = config.get('hover_seconds_by_waypoint')
        if hover_by_waypoint is None:
            hover_by_waypoint = [config['hover_sec']] * len(config['waypoints_ned_m'])
        if (not isinstance(hover_by_waypoint, list)
                or len(hover_by_waypoint) != len(config['waypoints_ned_m'])
                or any(not math.isfinite(value) or value < 0 for value in hover_by_waypoint)):
            raise ValueError('hover_seconds_by_waypoint must have one non-negative duration per waypoint')
        self.segments = []
        position = self.home[:]
        elapsed = 0.
        yaw = 0.
        def add(stage, target, speed, hold=0.):
            nonlocal position, elapsed, yaw
            distance = math.dist(position, target)
            duration = distance / speed if distance else hold
            delta = [b-a for a,b in zip(position,target)]
            if math.hypot(delta[0],delta[1]) > 1e-6:
                yaw = math.atan2(delta[1],delta[0])
            self.segments.append((elapsed,elapsed+duration,stage,position[:],list(target),yaw))
            elapsed += duration
            position = list(target)
        altitude = config['cruise_altitude_m']
        add('takeoff', [position[0],position[1],-altitude], config['vertical_speed_mps'])
        for index, waypoint in enumerate(config['waypoints_ned_m']):
            add('transit_%02d' % (index+1), waypoint, config['speed_mps'])
            add('inspect_%02d' % (index+1), waypoint, config['speed_mps'], hover_by_waypoint[index])
        add('return', [self.home[0],self.home[1],position[2]], config['speed_mps'])
        add('approach_land', [self.home[0],self.home[1],-3.0], config['vertical_speed_mps'])
        add('land', self.home, config['vertical_speed_mps'])
        self.duration = elapsed

    def sample(self, seconds):
        t = max(0.,seconds)
        for start,end,stage,a,b,yaw in self.segments:
            if t < end:
                fraction = (t-start)/(end-start)
                return [x+(y-x)*fraction for x,y in zip(a,b)],yaw,{
                    'stage':stage,'elapsed_sec':t,'duration_sec':self.duration,
                    'progress':min(t/self.duration,1.),'complete':False}
        return self.home[:],self.segments[-1][-1],{
            'stage':'complete','elapsed_sec':t,'duration_sec':self.duration,'progress':1.,'complete':True}
