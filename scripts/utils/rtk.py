# Lazy import to avoid version conflicts between asammdf and canmatrix
# The MDF import is deferred until read_rtk_data_from_mf4() is called
from datetime import datetime, timezone, timedelta
from zoneinfo import ZoneInfo
import math
from bisect import bisect_left
import numpy as np
import pandas as pd

GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
SECONDS_PER_WEEK = 604800

GPS_UTC_LEAP_SECONDS = 18  # 截至2024年，GPS时间领先UTC时间18秒
def gps_to_utc(gps_week: int, gps_seconds: float) -> datetime:
    """将GPS周和秒转换为UTC时间"""
    gps_time = GPS_EPOCH + timedelta(weeks=gps_week, seconds=gps_seconds)
    utc_time = gps_time - timedelta(seconds=GPS_UTC_LEAP_SECONDS)
    return utc_time  #输出UTC时间
def utc_to_local(utc_dt: datetime, local_tz: str) -> datetime:
    """将UTC时间转换为本地时区时间"""
    local_zone = ZoneInfo(local_tz)
    local_dt = utc_dt.astimezone(local_zone)
    return local_dt
def gps_to_local(gps_week: int, gps_seconds: float, local_tz: str="Asia/Shanghai") -> datetime:
    """将GPS周和秒转换为本地时区时间"""
    utc_dt = gps_to_utc(gps_week, gps_seconds)
    local_dt = utc_to_local(utc_dt, local_tz)
    return local_dt
class RTKData:
    
    def __init__(self, gps_week:int=None,gps_seconds:float=None, longitudinal:float=0.0, latitudinal:float=0.0, altitude:float=0.0,heading:float=0.0,
                 system_state:int=None, satellite_status:int=None, gps_num_used:int=0, gps_age:float=None,timestamp:float=None):
                """Create an RTKData record.

                Parameters
                - gps_week, gps_seconds: GPS week and seconds (used to compute UTC timestamp).
                - longitudinal: longitudinal in degrees 
                - latitudinal: latitude in degrees
                - altitude: ellipsoidal height in meters
                - heading: RTK heading in degrees, range [0, 360), defined as
                        0 = geographic north, increasing clockwise (same convention as device output)
                - system_state: integer RTK system state code (per DBC/设备定义)
                - satellite_status: integer satellite status or quality code
                - gps_num_used: number of satellites used
                - gps_age: age of differential correction (seconds) or similar metric

                Notes:
                - All geographic inputs are expected in WGS84.
                - Heading uses 0..360 convention; conversion to vehicle/frame angles
                    should be handled by caller or helper methods (e.g. `to_body`).
                """
                # UTC timestamp derived from GPS week/seconds if provided, otherwise None
                if gps_week is None or gps_seconds is None:
                    self.gps_time = None
                else:
                    self.gps_time = gps_to_utc(gps_week, gps_seconds)

                # Geographic coordinates (degrees, meters)
                # attribute name `longitudinal` kept for compatibility with existing code
                self.longitudinal = longitudinal
                self.latitudinal = latitudinal
                self.altitude = altitude

                # Heading (degrees, 0..360, north = 0, clockwise positive)
                self.heading = heading

                # RTK status and metadata
                self.system_state = system_state
                self.satellite_status = satellite_status
                self.gps_num_used = gps_num_used
                self.gps_age = gps_age
                self.timestamp = timestamp  # optional raw timestamp (e.g., from CAN message)
    
    def get_quality(self, max_age=1.0):
        score = 0
        if self.satellite_status == 4:
            score  = score +3 
        if self.gps_age < max_age:
            score = score +2
        if self.gps_num_used >= 10:
            score = score +2
        return score
    @staticmethod
    def _geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float):
        """Convert geodetic coordinates (deg,deg,m) to ECEF (m).

        Uses WGS84 ellipsoid.
        Returns (x,y,z) in meters.
        """
        # WGS84 parameters
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = f * (2 - f)

        lat = np.deg2rad(lat_deg)
        lon = np.deg2rad(lon_deg)

        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        cos_lon = np.cos(lon)
        sin_lon = np.sin(lon)

        N = a / np.sqrt(1 - e2 * sin_lat * sin_lat)

        x = (N + alt) * cos_lat * cos_lon
        y = (N + alt) * cos_lat * sin_lon
        z = (N * (1 - e2) + alt) * sin_lat
        return x, y, z

    @staticmethod
    def _ecef_to_enu(x: float, y: float, z: float, lat0_deg: float, lon0_deg: float, alt0: float):
        """Convert ECEF coordinates to local ENU coordinates relative to origin (lat0, lon0, alt0).

        Returns (e, n, u) in meters.
        """
        x0, y0, z0 = RTKData._geodetic_to_ecef(lat0_deg, lon0_deg, alt0)

        dx = x - x0
        dy = y - y0
        dz = z - z0

        lat0 = np.deg2rad(lat0_deg)
        lon0 = np.deg2rad(lon0_deg)

        sin_lat0 = np.sin(lat0)
        cos_lat0 = np.cos(lat0)
        sin_lon0 = np.sin(lon0)
        cos_lon0 = np.cos(lon0)

        # East
        e = -sin_lon0 * dx + cos_lon0 * dy
        # North
        n = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
        # Up
        u = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
        return e, n, u

    def to_enu(self, origin_lat: float, origin_lon: float, origin_alt: float = 0.0):
        """Convert this RTKData's geodetic coordinates to local ENU using given origin.

        Parameters:
            origin_lat, origin_lon: origin reference in degrees
            origin_alt: origin altitude in meters

        Returns:
            (east, north, up) in meters and also sets `self.enu_e`, `self.enu_n`, `self.enu_u`.
        """
        # Note: attribute names in this class: lonitudinal (longitude), latitudinal (latitude)
        lon = float(self.longitudinal)
        lat = float(self.latitudinal)
        alt = float(self.altitude)

        x, y, z = RTKData._geodetic_to_ecef(lat, lon, alt)
        e, n, u = RTKData._ecef_to_enu(x, y, z, origin_lat, origin_lon, origin_alt)

        self.enu_e = float(e)
        self.enu_n = float(n)
        self.enu_u = float(u)
        return self.enu_e, self.enu_n, self.enu_u

    @staticmethod
    def antenna_to_vehicle_ref(E_ant: float, N_ant: float, U_ant: float,
                               heading_deg: float, dx: float, dy: float, dz: float):
        """Convert antenna ENU to vehicle reference ENU using body offset.

        Args:
            E_ant, N_ant, U_ant: antenna position in ENU (meters)
            heading_deg: heading (0-360) where 0 = north, positive clockwise
            dx, dy, dz: antenna offset in vehicle body frame (meters):
                dx = forward (x forward), dy = right (y right), dz = up

        Returns:
            (E_ref, N_ref, U_ref): vehicle reference point in ENU

        Note: rotation uses heading as angle from north clockwise.
        """
        psi = math.radians(heading_deg)
        # body->ENU: forward (dx) projects to ENU as (sinψ, cosψ)
        bE = dx * math.sin(psi) + dy * math.cos(psi)
        bN = dx * math.cos(psi) - dy * math.sin(psi)
        bU = dz
        E_ref = float(E_ant) - float(bE)
        N_ref = float(N_ant) - float(bN)
        U_ref = float(U_ant) - float(bU)
        return E_ref, N_ref, U_ref

    @staticmethod
    def enu_to_body(E: float, N: float, U: float,
                    E_ref: float, N_ref: float, U_ref: float,heading0_deg:float,
                    heading_deg: float, body_z_up: bool = True):
        """Project ENU coordinates into vehicle body frame (ISO 8855-like).

        Args:
            E,N,U: point in ENU (meters)
            E_ref,N_ref,U_ref: vehicle reference point in ENU (meters)
            heading_deg: heading (0-360) where 0 = north, positive clockwise
            body_z_up: True if body z is up (returns up positive), False to invert

        Returns:
            x (forward), y (right positive), z (up or down based on body_z_up)

        Formula assumptions: heading is angle from north clockwise. Using psi = rad(heading),
        forward unit in ENU = [sinψ, cosψ], right unit = [cosψ, -sinψ].
        """
        dE = float(E) - float(E_ref)
        dN = float(N) - float(N_ref)
        dU = float(U) - float(U_ref)
        psi0 = RTKData.huace_heading_def_math_rad(heading0_deg)
        cos0 = np.cos(psi0)
        sin0 = np.sin(psi0)
        x = dE * cos0 +dN*sin0
        y = -dE * sin0 + dN * cos0
        z = dU if body_z_up else -dU
        heading_math = RTKData.huace_heading_def_math_rad(heading_deg)
        heading_vehicle = RTKData.wrap_to_pi(heading_math - psi0)
        return float(x), float(y), float(z), float(heading_vehicle)
    @staticmethod
    def wrap_to_pi(rad:float):
        return (rad+np.pi) % (2*np.pi) - np.pi
    @staticmethod
    def huace_heading_def_math_rad(heading_deg:float):
        """Convert heading from RTK (geographic) to ISO 8855 (vehicle body frame).

        RTK heading: 0°=north, 90°=east, clockwise positive, range [0, 360)
        ISO 8855 heading: 0=forward, π/2=left, counterclockwise positive, range [-π, π]
        Formula: heading_8855_rad = -heading_rtk_rad (normalized to [-π, π])
        """
        heading_rad = np.deg2rad(heading_deg)
        heading_math = np.pi/2 - heading_rad
        return RTKData.wrap_to_pi(heading_math)
    def to_body(self, origin_lat: float, origin_lon: float, origin_alt: float,
                origin_heading: float, antenna_offset: tuple = None, body_z_up: bool = True):
        """Convert this RTKData to vehicle body coordinates.

        Steps:
        1. compute antenna ENU relative to provided origin (calls `to_enu`)
        2. if `antenna_offset` provided (dx,dy,dz in body frame), compute vehicle reference ENU
           by rotating offset into ENU and subtracting from antenna ENU
        3. project ENU -> body using heading
        4. convert heading from RTK (geographic, 0-360° clockwise from north) to ISO 8855 (-π to π, counterclockwise from forward)

        Returns:
            (x_body, y_body, z_body, heading_8855) in meters and radians; sets instance attributes
            `body_x`, `body_y`, `body_z`, `body_heading`.
            
        Heading Conversion:
            - RTK heading: 0°=north, 90°=east, clockwise positive, range [0, 360)
            - ISO 8855 heading: 0=forward, π/2=left, counterclockwise positive, range [-π, π]
            - Formula: heading_8855_rad = -heading_rtk_rad (normalized to [-π, π])
        """
        # antenna ENU relative to origin
        E_ant, N_ant, U_ant = self.to_enu(origin_lat, origin_lon, origin_alt)
        if antenna_offset is not None:
            dx, dy, dz = antenna_offset
            E_ref, N_ref, U_ref = RTKData.antenna_to_vehicle_ref(E_ant, N_ant, U_ant, self.heading, dx, dy, dz)
        else:
            E_ref, N_ref, U_ref = 0.0, 0.0, 0.0   
        x_8855, y_8855, z_8855,heading_8855 = RTKData.enu_to_body(E_ant, N_ant, U_ant, E_ref, N_ref, U_ref, origin_heading,self.heading, body_z_up)
        
        # Convert heading from RTK (geographic) to ISO 8855 (vehicle body frame)
        # RTK: 0°=north, 90°=east, clockwise positive
        # ISO 8855: 0=forward, π/2=left, counterclockwise positive
        # heading_rtk_rad = math.radians(heading_deg)
        # heading_8855 = -heading_rtk_rad
        
        # Normalize heading to [-π, π]
        # while heading_8855 > math.pi:
        #     heading_8855 -= 2 * math.pi
        # while heading_8855 < -math.pi:
        #     heading_8855 += 2 * math.pi

        self.body_x = float(x_8855)
        self.body_y = float(y_8855)
        self.body_z = float(z_8855)
        self.body_heading = float(heading_8855)
        
        return self.body_x, self.body_y, self.body_z, self.body_heading
        
class RTKUtils:
    def __init__(self, mf4_file:str=None):
        if mf4_file is not None:
            self.__mf4_file__ = mf4_file
            self.read_rtk_data_from_mf4(mf4_file)
            self.rtk_list = self.filter_rtk_by_quality(self.rtk_list)
            # Build read-time statistics for callers / debugging
            read_stats = {}
            read_stats['assembled_count'] = len(self.rtk_list)
            read_stats['total_attempts'] = int(self.last_rtk_stats.get('total_attempts', 0)) if getattr(self, 'last_rtk_stats', None) is not None else None
            read_stats['dropped_count'] = int(self.last_rtk_stats.get('dropped', 0)) if getattr(self, 'last_rtk_stats', None) is not None else None
            read_stats['missing_gps_time'] = sum(1 for r in self.rtk_list if getattr(r, 'gps_time', None) is None)
            read_stats['missing_system_state'] = sum(1 for r in self.rtk_list if getattr(r, 'system_state', None) is None)
            read_stats['missing_satellite_status'] = sum(1 for r in self.rtk_list if getattr(r, 'satellite_status', None) is None)
            read_stats['missing_gps_age'] = sum(1 for r in self.rtk_list if getattr(r, 'gps_age', None) is None)
            read_stats['missing_altitude'] = sum(1 for r in self.rtk_list if getattr(r, 'altitude', None) is None)
            timestamps = [float(getattr(r, 'timestamp')) for r in self.rtk_list if getattr(r, 'timestamp', None) is not None]
            if timestamps:
                read_stats['timestamp_min'] = float(min(timestamps))
                read_stats['timestamp_max'] = float(max(timestamps))
            else:
                read_stats['timestamp_min'] = None
                read_stats['timestamp_max'] = None
            gps_times = [r.gps_time for r in self.rtk_list if getattr(r, 'gps_time', None) is not None]
            if gps_times:
                read_stats['gps_time_min'] = min(gps_times).isoformat()
                read_stats['gps_time_max'] = max(gps_times).isoformat()
            else:
                read_stats['gps_time_min'] = None
                read_stats['gps_time_max'] = None
            self.read_stats = read_stats
            pop_data_num = 10
            for ii in range(pop_data_num):
                if len(self.rtk_list) > 0:
                    self.rtk_list.pop(0)
            # Try to find a static start and use that RTK point as conversion origin.
            start_info = self.find_static_start(self.rtk_list, origin_lat=0.0, origin_lon=0.0, origin_alt=0.0, antenna_offset=None, window_seconds=2.0, movement_threshold=0.1)
            if start_info is not None and 'start_index' in start_info:
                print('find static start at index:', start_info['start_index'])
                si = int(start_info['start_index'])
                if 0 <= si < len(self.rtk_list):
                    origin_lat = getattr(self.rtk_list[si], 'latitudinal', 0.0)
                    origin_lon = getattr(self.rtk_list[si], 'longitudinal', 0.0)
                    origin_alt = getattr(self.rtk_list[si], 'altitude', 0.0)
                    origin_heading = getattr(self.rtk_list[si], 'heading', 0.0) 
                else:
                    origin_lat = self.rtk_list[0].latitudinal if self.rtk_list else 0.0
                    origin_lon = self.rtk_list[0].longitudinal if self.rtk_list else 0.0
                    origin_alt = self.rtk_list[0].altitude if self.rtk_list else 0.0
                    origin_heading = self.rtk_list[0].heading if self.rtk_list else 0.0
            else:
                # fallback to first sample (or zeros if empty)
                origin_lat = self.rtk_list[0].latitudinal if self.rtk_list else 0.0
                origin_lon = self.rtk_list[0].longitudinal if self.rtk_list else 0.0
                origin_alt = self.rtk_list[0].altitude if self.rtk_list else 0.0
                origin_heading = self.rtk_list[0].heading if self.rtk_list else 0.0

            # store chosen origin for callers
            self.origin = (origin_lat, origin_lon, origin_alt,origin_heading)
            # convert all RTK points to body coords using selected origin
            self.body_coords = self.convert_all_to_body(self.rtk_list, origin_lat=origin_lat, origin_lon=origin_lon, origin_alt=origin_alt, origin_heading=origin_heading, antenna_offset=None, body_z_up=True)
            # Print concise read statistics and chosen origin for quick feedback
            try:
                print('RTK read summary:')
                print(f"  assembled_count={self.read_stats.get('assembled_count')}, total_attempts={self.read_stats.get('total_attempts')}, dropped={self.read_stats.get('dropped_count')}")
                print(f"  missing: gps_time={self.read_stats.get('missing_gps_time')}, system_state={self.read_stats.get('missing_system_state')}, satellite_status={self.read_stats.get('missing_satellite_status')}, gps_age={self.read_stats.get('missing_gps_age')}, altitude={self.read_stats.get('missing_altitude')}")
                print(f"  timestamps: min={self.read_stats.get('timestamp_min')}, max={self.read_stats.get('timestamp_max')}")
                print(f"  chosen_origin: lat={origin_lat}, lon={origin_lon}, alt={origin_alt}")
                bc_count = len(self.body_coords) if getattr(self, 'body_coords', None) is not None else 0
                print(f"  body_coords_count={bc_count}")
            except Exception:
                pass
    def convert_all_to_body(self, rtk_list, origin_lat: float, origin_lon: float, origin_alt: float=0.0, origin_heading:float=0.0, antenna_offset: tuple=None, body_z_up: bool=True):
        """Convert a list of RTKData to vehicle body coordinates (ISO8855-like).

        Returns list of (x,y,z,heading) tuples and sets `body_x`,`body_y`,`body_z`,`body_heading` on each RTKData.
        """
        out = []
        for r in rtk_list:
            # heading may be None, but RTKData.heading should exist; if missing skip conversion
            #heading_deg = r.heading if hasattr(r, 'heading') else 0.0
            #try:
            x, y, z, heading_8855 = r.to_body(origin_lat, origin_lon, origin_alt, origin_heading, antenna_offset, body_z_up)
            out.append(r)
            #except Exception:
                # x, y, z, heading_8855 = None, None, None, None
            #    print("Error converting RTKData to body coordinates for:", r)
            #    continue
        return out

    def find_static_start(self, rtk_list, origin_lat: float, origin_lon: float, origin_alt: float=0.0, antenna_offset: tuple=None, window_seconds: float=2.0, movement_threshold: float=0.1):
        """Find earliest index where vehicle is approximately stationary in body frame.

        Algorithm:
        - Convert RTK points to body coords via `to_body`.
        - For each start index i, gather samples with timestamp <= ts[i] + window_seconds.
        - Compute maximum pairwise planar distance in that window; if <= movement_threshold, return info.
        Returns dict with keys: start_index, start_time, reference (mean x,y,z), count. None if not found.
        """
        if not rtk_list:
            return None

        # choose a provisional ENU origin (use first valid RTK point)
        origin = None

        for r in rtk_list:
            if getattr(r, 'latitudinal', None) is not None and getattr(r, 'longitudinal', None) is not None and getattr(r, 'altitude', None) is not None:
                origin = (r.latitudinal, r.longitudinal, r.altitude)
                break
        if origin is None:
            return None
        origin_lat0, origin_lon0, origin_alt0 = origin

        # compute ENU for all RTK points relative to provisional origin (do not apply antenna offset)
        enu_list = []
        for r in rtk_list:
            try:
                x, y, z = RTKData._geodetic_to_ecef(r.latitudinal, r.longitudinal, r.altitude)
                e, n, u = RTKData._ecef_to_enu(x, y, z, origin_lat0, origin_lon0, origin_alt0)
                enu_list.append((e, n, u))
            except Exception:
                enu_list.append((None, None, None))

        # prepare timestamps (use r.timestamp if available)
        times = [getattr(r, 'timestamp', None) for r in rtk_list]
        times = [t if t is not None else float(i) for i,t in enumerate(times)]
        n = len(rtk_list)

        for i in range(n):
            t0 = times[i]
            window_idx = [j for j in range(i, n) if times[j] - t0 <= window_seconds]
            if len(window_idx) < 2:
                continue
            coords = []
            for j in window_idx:
                e,n_e,u = enu_list[j]
                if e is None or n_e is None:
                    coords = None
                    break
                # project ENU -> body ignoring antenna offset for stationary detection
                psi = math.radians(rtk_list[j].heading if getattr(rtk_list[j], 'heading', None) is not None else 0.0)
                bx = e * math.sin(psi) + n_e * math.cos(psi)
                by = e * math.cos(psi) - n_e * math.sin(psi)
                bz = u
                coords.append((bx, by, bz))
            if coords is None or not coords:
                continue
            # compute max pairwise planar distance
            maxd = 0.0
            L = len(coords)
            for a in range(L):
                xa, ya = coords[a][0], coords[a][1]
                for b in range(a+1, L):
                    xb, yb = coords[b][0], coords[b][1]
                    d = math.hypot(xa - xb, ya - yb)
                    if d > maxd:
                        maxd = d
                        if maxd > movement_threshold:
                            break
                if maxd > movement_threshold:
                    break
            if maxd <= movement_threshold:
                mean_x = sum(c[0] for c in coords) / len(coords)
                mean_y = sum(c[1] for c in coords) / len(coords)
                mean_z = sum(c[2] for c in coords) / len(coords)
                return {'start_index': i, 'start_time': t0, 'reference': (mean_x, mean_y, mean_z), 'count': len(coords)}
        return None
    def read_rtk_data_from_mf4(self, mf4_file:str=None):
        """
        从MF4文件中读取RTK数据，返回包含时间戳和坐标的DataFrame。
        
        Args:
            mf4_file: MF4文件路径
        
        Returns:
            pd.DataFrame: 包含'timestamp', 'x', 'y'列的DataFrame
        """
        if mf4_file is None:
            return None
        
        # Lazy import of asammdf to avoid version conflicts
        from asammdf import MDF
        
        mdf = MDF(mf4_file)
        signals = ["PosLat2","PosLon2","AngleHeading","GpsWeek","GpsTime","system_state","satellite_status","GpsAge","PosAlt"]
        signal_samples = mdf.select(signals)
        gps_lateral = signal_samples[0].samples.tolist()
        gps_lateral_timestmap = signal_samples[0].timestamps.tolist()
        gps_longitudinal = signal_samples[1].samples.tolist()
        gps_longitudinal_timestmap = signal_samples[1].timestamps.tolist()
        heading = signal_samples[2].samples.tolist()
        heading_timestmap = signal_samples[2].timestamps.tolist()
        gps_week = signal_samples[3].samples.tolist()
        gps_week_timestmap = signal_samples[3].timestamps.tolist()
        gps_seconds= signal_samples[4].samples.tolist()
        gps_seconds_timestmap = signal_samples[4].timestamps.tolist()
        system_state = signal_samples[5].samples.tolist()
        system_state_timestmap = signal_samples[5].timestamps.tolist()
        satellite_status = signal_samples[6].samples.tolist()
        satellite_status_timestmap = signal_samples[6].timestamps.tolist()
        gps_age = signal_samples[7].samples.tolist()
        gps_age_timestmap = signal_samples[7].timestamps.tolist()
        gps_altitude = signal_samples[8].samples.tolist()
        gps_altitude_timestmap = signal_samples[8].timestamps.tolist()
        delta_short = 0.005 # heading <-> lat/lon/alt tolerance (5 ms)
        delta_long = 1.0    # low-frequency signals tolerance (1 s)
        # Helper: find nearest index in a sorted timestamp list within tolerance
        def find_nearest_index(ts_list, target, tol):
            if not ts_list:
                return None
            i = bisect_left(ts_list, target)
            candidates = []
            if i < len(ts_list):
                candidates.append(i)
            if i > 0:
                candidates.append(i-1)
            best_idx = None
            best_dt = None
            for ci in candidates:
                dt = abs(ts_list[ci] - target)
                if dt <= tol and (best_dt is None or dt < best_dt):
                    best_dt = dt
                    best_idx = ci
            return best_idx

        rtk_list = []
        dropped_count = 0
        # (no ENU conversion here; leave conversion to caller)
        # Use heading signal as the basis for grouping
        for jj in range(0, len(heading)):
            base_ts = heading_timestmap[jj]

            # heading matches tightly to lat/lon (alt treated as low-frequency optional)
            idx_head = jj
            idx_lat = find_nearest_index(gps_lateral_timestmap, base_ts, delta_short)
            idx_lon = find_nearest_index(gps_longitudinal_timestmap, base_ts, delta_short)
            # altitude timestamps/samples already extracted above as gps_altitude/gps_altitude_timestmap
            # treat altitude as low-frequency optional: match with long tolerance
            idx_alt = find_nearest_index(gps_altitude_timestmap, base_ts, delta_long) if gps_altitude_timestmap else None

            # require lat/lon to be present within short tolerance
            if idx_lat is None or idx_lon is None:
                dropped_count += 1
                continue

            # low-frequency signals match with long tolerance; if not found, set None
            idx_week = find_nearest_index(gps_week_timestmap, base_ts, delta_long)
            idx_sec = find_nearest_index(gps_seconds_timestmap, base_ts, delta_long)
            idx_sys = find_nearest_index(system_state_timestmap, base_ts, delta_long)
            idx_sat = find_nearest_index(satellite_status_timestmap, base_ts, delta_long)
            idx_age = find_nearest_index(gps_age_timestmap, base_ts, delta_long)

            # Build matched timestamp list from available matches
            matched_ts = [heading_timestmap[idx_head]]
            '''
            if idx_week is not None:
                matched_ts.append(gps_week_timestmap[idx_week])
            if idx_sec is not None:
                matched_ts.append(gps_seconds_timestmap[idx_sec])
            if idx_sys is not None:
                matched_ts.append(system_state_timestmap[idx_sys])
            if idx_sat is not None:
                matched_ts.append(satellite_status_timestmap[idx_sat])
            if idx_age is not None:
                matched_ts.append(gps_age_timestmap[idx_age])
            '''
            avg_ts = float(sum(matched_ts) / len(matched_ts))


            # Extract values (low-freq may be None)
            lat_val = gps_lateral[idx_lat]
            lon_val = gps_longitudinal[idx_lon]
            heading_val = heading[idx_head]
            alt_val = gps_altitude[idx_alt]
            week_val = gps_week[idx_week] if idx_week is not None else None
            seconds_val = gps_seconds[idx_sec] if idx_sec is not None else None
            system_state_val = system_state[idx_sys] if idx_sys is not None else None
            satellite_status_val = satellite_status[idx_sat] if idx_sat is not None else None
            gps_age_val = gps_age[idx_age] if idx_age is not None else None

            # Build RTKData (gps_num_used unknown -> set 0); allow None for low-frequency fields
            try:
                rtk = RTKData(gps_week=week_val, gps_seconds=seconds_val, longitudinal=float(lon_val), latitudinal=float(lat_val), altitude=float(alt_val), heading=float(heading_val),
                              system_state=(int(system_state_val) if system_state_val is not None else None), satellite_status=(int(satellite_status_val) if satellite_status_val is not None else None), gps_num_used=0, gps_age=(float(gps_age_val) if gps_age_val is not None else None), timestamp=avg_ts)
                # no ENU conversion here; caller may convert later via RTKData.to_enu
                rtk_list.append(rtk)
            except Exception:
                dropped_count += 1
                continue

        # 保存统计信息到实例，保持向后兼容（仍返回 rtk_list）
        total = len(heading)
        valid = len(rtk_list)
        self.rtk_list = rtk_list
        self.last_rtk_stats = {
            'total_attempts': int(total),
            'valid': int(valid),
            'dropped': int(dropped_count)
        }
        # print(f"RTK组包完成：{valid} 条有效，{dropped_count} 条丢弃（时间同步失败），总尝试 {total}")
        return self.rtk_list
    def filter_rtk_by_quality(self, rtk_list, quality:int=5, max_age: float=1.0):
        """Filter RTKData list by quality criteria.

        Args:
            rtk_list: list of RTKData
            quality: minimum quality score required
            max_age: maximum GPS age (seconds) allowed

        Returns:
            filtered list of RTKData
        """
        filtered = []
        for r in rtk_list:
            score = 0
            if r.satellite_status == 4:
                score  = score +3 
            if r.gps_age < max_age:
                score = score +2
            if r.gps_num_used >= 10:
                score = score +2
            if score >= quality:
                filtered.append(r)
        return filtered
    @staticmethod
    def unwrap_angle(angle):
        return np.unwrap(angle)
    @staticmethod
    def compute_yaw_rate(time,yaw):
        """
        compute yaw rate from yaw angle and time
        Args:
            time: time array
            yaw: yaw angle array
        Returns:
            yaw rate array
        """
        yaw_u = RTKUtils.unwrap_angle(yaw)
        dt = np.diff(time)
        dyaw = np.diff(yaw_u)
        valid = dt > 1e-4
        yaw_rate = np.zeros_like(yaw)
        yaw_rate[1:][valid] = dyaw[valid] / dt[valid]
        yaw_rate[0] = yaw_rate[1]  # set first value to second to avoid zero
        return yaw_rate
    @staticmethod
    def interp_signal(t_src,v_src,t_dst):
        """
        插值信号到目标时间点
        Args:
            t_src: 源时间数组
            v_src: 源信号数组
            t_dst: 目标时间数组
        Returns:
            插值后的信号数组
        """
        v_dst = np.interp(t_dst, t_src, v_src,left=np.nan,right=np.nan)
        return v_dst
    @staticmethod
    def yaw_rate_offset_search(t_loc,yaw_loc,t_rtk,yaw_rtk,search_angle=(-0.5,0.5),search_step=0.01,min_valid_ratio=0.6):
        """
         Estimate time offset between RTK and localization using yaw_rate correlation

    Returns:
        best_offset, offsets, scores
        """
        yaw_rate_loc = RTKUtils.compute_yaw_rate(t_loc,yaw_loc)
        yaw_rate_rtk = RTKUtils.compute_yaw_rate(t_rtk,yaw_rtk)

        offsets = np.arange(search_angle[0],search_angle[1],search_step)
        scores = np.zeros_like(offsets)
        for i,dt in enumerate(offsets):
            # RTK time shifted
            t_rtk_shift = t_rtk + dt

            # inertpolate RTK yaw_rate to loc timeline
            yaw_rate_rtk_i = RTKUtils.interp_signal(t_rtk_shift,yaw_rate_rtk,t_loc)

            valid = ~np.isnan(yaw_rate_rtk_i)
            valid_ratio = np.sum(valid) / len(t_loc)
            if valid_ratio < min_valid_ratio:
                scores[i] = np.nan
                continue
            # correlation
            corr = np.corrcoef(yaw_rate_loc[valid],yaw_rate_rtk_i[valid])[0,1]
            scores[i] = corr
        # best offset
        best_idx = np.nanargmax(scores)
        best_offset = offsets[best_idx]
        return best_offset, offsets, scores        
    def find_best_time_offset(self,localization_csv:str=None,is_show_plot:bool=False):
        if localization_csv is None:
            print("No localization CSV file provided for time offset search.")
            return None
        if self.body_coords is None:
            print("No body coordinates available for RTK data.")
            return None
        # read localization data
        loc_data = pd.read_csv(localization_csv,sep=',',skipinitialspace=True)
        loc_time = loc_data['timestamp'].to_numpy()
        loc_yaw = loc_data['heading'].to_numpy()
        rtk_time = []
        for rtk in self.body_coords:
            rtk_time.append(rtk.timestamp)
        rtk_time = np.array(rtk_time)
        rtk_yaw = [] 
        for rtk in self.body_coords:
            rtk_yaw.append(rtk.body_heading)
        rtk_yaw = np.array(rtk_yaw) 
        best_offset, offsets, scores = RTKUtils.yaw_rate_offset_search(loc_time,loc_yaw,rtk_time,rtk_yaw,
                                                                       search_angle=(-0.3,0.3),search_step=0.001)
        if is_show_plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.plot(offsets,scores)
            plt.axvline(best_offset, color='r')
            plt.xlabel("dt_offset [s]")
            plt.ylabel("yaw_rate correlation")
            plt.title("RTK ↔ Localization Time Offset Search")
            plt.grid(True)
            plt.show()
        return best_offset,offsets,scores
    def save_csv(self,rtk_file:str='rtk_data.csv',rtk_body_file:str='rtk_converted.csv'):
        # save rtk_data.csv file
        print(f"Saving RTK data to '{rtk_file}' and body coordinates to '{rtk_body_file}'")
        with open(rtk_file,'w') as f:
            f.write("RTK_x,RTK_y,RTK_z,RTK_heading,RTK_qualit,timestamp\r\n")
            for rtk in self.rtk_list:
                f.write(f"{rtk.latitudinal},{rtk.longitudinal},{rtk.altitude},{rtk.heading},{rtk.get_quality()},{rtk.timestamp}\r\n")
        print("RTK data saved. total points:", len(self.rtk_list))
        # save rtk_converted.csv file with body coordinates including heading in ISO 8855
        with open(rtk_body_file,'w') as f:
            f.write("x,y,z,heading_rad,heading_deg,quality,timestamp\r\n")
            for i, rtk in enumerate(self.rtk_list):
                if i < len(self.body_coords):
                    f.write(f"{rtk.body_x:3g},{rtk.body_y:3g},{rtk.body_z:3g},{rtk.body_heading:4g},{np.rad2deg(rtk.body_heading):4g},{rtk.get_quality()},{rtk.timestamp}\r\n")
        print("RTK body coordinates saved. total points:", len(self.body_coords))
if __name__ == "__main__":
    rtk_utils = RTKUtils(mf4_file="/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stype_forward_5kph_001.mf4")
    rtk_utils.save_csv(rtk_file='/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stype_forward_5kph_001/rtk_data.csv', 
                       rtk_body_file='/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stype_forward_5kph_001/rtk_converted.csv')
    best_offset,offsets,scores= rtk_utils.find_best_time_offset(localization_csv="/Users/alex/workspace/code_review/vehicle_localization/data/Lev01_R19_Stype_forward_5kph_001/vehicle_localization.csv"
                                                                ,is_show_plot=True)
    print("Best time offset (s):", best_offset)
