import math
import sys


def deg2rad(x):
    return x / 180 * math.pi


def rad2deg(x):
    return x / math.pi * 180


def ft2m(x):
    return x * 0.3048

def m2ft(x):
    return x / 0.3048

def nm2m(x):
    return x*1852

def kts2mps(x):
    return x * 0.5144444444

def mps2kts(x):
    return x / 0.5144444444


def hex_string_to_byte_values(hex_string):
    """
    Converts a string which contains hex strings e.g. "0xff" to a string of byte values
    :param hex_string:
    :return:
    """
    hex_string = hex_string.replace("0x", "")
    hex_string = hex_string.replace("\\x", "")

    return bytearray.fromhex(hex_string)


class ECEF_t:
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.z = 0.


class LLH_t:
    def __init__(self, lat=0., lon=0., height=0.):
        self.latitude = float(lat)
        self.longitude = float(lon)
        self.height = float(height)


class NED_t:
    def __init__(self, n=0, e=0, d=0):
        self.north = float(n)
        self.east = float(e)
        self.down = float(d)

    def distance(self, other):
        dn = self.north - other.north
        de = self.east - other.east
        dd = self.down - other.down

        return math.sqrt(dn * dn + de * de + dd * dd)


class HomeReference:
    def __init__(self, lat_ref, lon_ref):
        self.SemiMajorAxis = 6378137.0
        self.SemiMinorAxis = 6356752.315
        self.Excentricity = 0.0818191908426
        self.Excentricity2 = 0.0818191908426 * 0.0818191908426
        self.LatitudeReference = lat_ref
        self.LongitudeReference = lon_ref
        self.AltitudeReference = 0

        sinlat = math.sin(lat_ref)
        coslat = math.cos(lat_ref)
        sinlon = math.sin(lon_ref)
        coslon = math.cos(lon_ref)

        self.sinlat_ref = sinlat
        self.coslat_ref = coslat
        self.sinlon_ref = sinlon
        self.coslon_ref = coslon

        ROfCurvature = self.RadiusOfCurvature(lat_ref)
        RofC_AltRef = ROfCurvature + self.AltitudeReference

        self.e0 = ECEF_t()
        self.e0.x = RofC_AltRef * coslat * coslon
        self.e0.y = RofC_AltRef * coslat * sinlon
        self.e0.z = ((ROfCurvature * (1 - self.Excentricity2)) + self.AltitudeReference) * sinlat

        self.TransformationMatrix = [0. for i in range(0, 9)]
        self.TransformationMatrix[0] = -sinlat * coslon
        self.TransformationMatrix[1] = -sinlon
        self.TransformationMatrix[2] = -coslat * coslon
        self.TransformationMatrix[3] = -sinlat * sinlon
        self.TransformationMatrix[4] = coslon
        self.TransformationMatrix[5] = -coslat * sinlon
        self.TransformationMatrix[6] = coslat
        self.TransformationMatrix[7] = 0
        self.TransformationMatrix[8] = -sinlat

    '''
     convert GPS coordinates to X/Y/Z coordinates
	v = a / sqrt(1 -(e * sin(lat))^2)

	xe = (v+h) cos(lat)cos(lon)
	ye = (v+h) cos(lat)sin(lon)
	ze = ((1-e^2)v+h) sin(lat)

	xg   |-sin(lat0)cos(lon0) -sin(lat0)sin(lon0)  cos(lat0)| |xe-xe0|
	yg = |    -sin(lon0)          -cos(lon0)           0    | |ye-ye0|
	zg   |-cos(lat0)cos(lon0) -cos(lat0)sin(lon0) -sin(lat0)| |ze-ze0|

    '''

    def llh2ned(self, llh):
        v_ref = self.SemiMajorAxis / math.sqrt(1 - (self.Excentricity * self.sinlat_ref) ** 2)

        x_ref = (v_ref + self.AltitudeReference) * self.coslat_ref * self.coslon_ref
        y_ref = (v_ref + self.AltitudeReference) * self.coslat_ref * self.sinlon_ref
        z_ref = ((1 - self.Excentricity2) * v_ref + self.AltitudeReference) * self.sinlat_ref

        sinlat = math.sin(llh.latitude)
        coslat = math.cos(llh.latitude)
        sinlon = math.sin(llh.longitude)
        coslon = math.cos(llh.longitude)

        v = self.SemiMajorAxis / math.sqrt(1 - (self.Excentricity * sinlat) ** 2)

        x_gps = (v + llh.height) * coslat * coslon - x_ref
        y_gps = (v + llh.height) * coslat * sinlon - y_ref
        z_gps = (llh.height + (1 - self.Excentricity2) * v) * sinlat - z_ref

        x = (-self.sinlat_ref * self.coslon_ref) * x_gps + (
                    -self.sinlat_ref * self.sinlon_ref) * y_gps + self.coslat_ref * z_gps
        y = (-self.sinlon_ref) * x_gps + self.coslon_ref * y_gps
        z = (-self.coslat_ref * self.coslon_ref) * x_gps + (-self.coslat_ref * self.sinlon_ref) * y_gps + (
            -self.sinlat_ref) * z_gps

        return NED_t(x, y, z)

    '''
     METHOD 1------------------------------------------------------------------------
   convert XgYgZg coordinates to gps coordinates

   xg   |-sin(lat0)cos(lon0) -sin(lat0)sin(lon0)  cos(lat0)| |xe-xe0|
   yg = |    -sin(lon0)          -cos(lon0)           0    | |ye-ye0|
   zg   |-cos(lat0)cos(lon0) -cos(lat0)sin(lon0) -sin(lat0)| |ze-ze0|

   xe = (v+h) cos(lat)cos(lon)
   ye = (v+h) cos(lat)sin(lon)
   ze = ((1-e^2)v+h) sin(lat)

   v  = a / sqrt (1-(e*sin(lat))^2)


   xe   |-sin(lat0)cos(lon0) -sin(lon0) -cos(lat0)cos(lon0)| |xg|  |xe0|
   ye = |-sin(lat0)sin(lon0) -cos(lon0) -cos(lat0)sin(lon0)| |yg| +|ye0|
   ze   |     cos(lat0)         0           -sin(lat0)     | |zg|  |ze0|

   while h(i)-h(i-1) < eps && lat(i)-lat(i-1) < eps
   gps_lon = atan2 ( xe   ,  ye )
   gps_lat = atan2 ( ze   , sqrt(xe^2+ye^2)*(1-e^2*v/(v+h))
         h = sqrt(xe^2+ye^2) / cos(gps_lat)    - v

   end

METHOD 2------------------------------------------------------------------------
   convert XgYgZg coordinates to gps coordinates
   Zg is equivalent to h and not to "standard zg"


   xg   |-sin(lat0)cos(lon0) -sin(lat0)sin(lon0)  cos(lat0)| |xe-xe0|
   yg = |    -sin(lon0)          -cos(lon0)           0    | |ye-ye0|
   zg   |-cos(lat0)cos(lon0) -cos(lat0)sin(lon0) -sin(lat0)| |ze-ze0|

   xe = (v+h) cos(lat)cos(lon)
   ye = (v+h) cos(lat)sin(lon)
   ze = ((1-e^2)v+h) sin(lat)

   v  = a / sqrt (1-(e*sin(lat))^2)

   from gps to xg yg zg use h=0 ang zg = h!!!!

   from xg yg zg=h to gps

   gps_h = zg


   zg  = 0 start value
   h   = 0
  while h < eps
   xe   |-sin(lat0)cos(lon0) -sin(lon0) -cos(lat0)cos(lon0)| |xg|  |xe0|
   ye = |-sin(lat0)sin(lon0) -cos(lon0) -cos(lat0)sin(lon0)| |yg| +|ye0|
   ze   |     cos(lat0)         0           -sin(lat0)     | |zg|  |ze0|

   gps_lon = atan2 ( xe   ,  ye )
   gps_lat = atan2 ( ze   , sqrt(xe^2+ye^2)*(1-e^2*v/(v+h))
         h = sqrt(xe^2+ye^2) / cos(gps_lat)    - v

   end

  xe0 ye0 ze0 calc with h=0 !!!!!!!
*/
    '''

    def ned2llh(self, ned: NED_t):
        # Transform Reference Point to CartesianSystem
        v_ref = self.SemiMajorAxis / math.sqrt(1 - ((self.Excentricity * self.sinlat_ref) ** 2))

        x_ref = (v_ref + self.AltitudeReference) * self.coslat_ref * self.coslon_ref
        y_ref = (v_ref + self.AltitudeReference) * self.coslat_ref * self.sinlon_ref
        z_ref = ((1 - (self.Excentricity ** 2)) * v_ref + self.AltitudeReference) * self.sinlat_ref

        xi = ned.north
        yi = ned.east
        zi = ned.down

        # Transformation from the Earth -Fixed Geodectic System to the Geocentric
        # Cartesian System to the, see Coordinate Systems 4.5 .2
        x = (-self.sinlat_ref * self.coslon_ref) * xi + (-self.sinlon_ref) * yi + (
                    -self.coslat_ref * self.coslon_ref) * zi + x_ref
        y = (-self.sinlat_ref * self.sinlon_ref) * xi + (self.coslon_ref) * yi + (
                    -self.coslat_ref * self.sinlon_ref) * zi + y_ref
        z = self.coslat_ref * xi + (-self.sinlat_ref) * zi + z_ref

        # Transform from the Gecentric Cartesian to the Geocentric Polar System(GPS),
        # see Coordinate Systems 4.3 .3 .1         Approximate Transformation is used
        # Requirement: p > 42698 m, i.e.minimum distance from south or north pole
        p = math.sqrt(x * x + y * y)
        beta = math.atan2(z, math.sqrt(1 - (self.Excentricity ** 2)) * p)

        lat = math.atan2((z + (self.Excentricity ** 2) * self.SemiMajorAxis / math.sqrt(1 - self.Excentricity ** 2) * (
                    math.sin(beta) ** 3)),
                         p - (self.Excentricity ** 2) * self.SemiMajorAxis * (math.cos(beta) ** 3))
        hae = p * math.cos(lat) + z * math.sin(lat) - self.SemiMajorAxis * math.sqrt(
            1 - ((self.Excentricity * math.sin(lat)) ** 2))

        lon = math.atan2(y, x)

        return LLH_t(lat, lon, hae)

    @staticmethod
    def Radius(x, y):
        return math.sqrt((x * x) + (y * y))

    def RadiusOfCurvature(self, phi):
        sin_phi = math.sin(phi)
        return self.div(self.SemiMajorAxis, math.sqrt(1 - self.Excentricity2 * (sin_phi * sin_phi)))

    @staticmethod
    def div(dividend, divisor):
        eps = sys.float_info.epsilon
        if divisor < eps:
            if divisor < 0:
                divisor = -eps
            else:
                divisor = eps

        return (dividend / divisor)


def get_data_from_wireshark_dump(file_name):
    data = ""

    with open(file_name) as wireshark_file:
        for line in wireshark_file:
            split_line = line.split()
            for word in split_line:
                is_hex_value = len(word) == 2
                if is_hex_value:
                    data += "0x" + word

    return data
