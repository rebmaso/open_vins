import numpy as np
from pyproj import Transformer
from scipy.spatial.transform import Rotation as R
import argparse

# Utility script to get ECEF pos & rot coordinates of ENU frame centered at LLA
# Use as arguments for static transform in ros2, for visualization purposes

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Convert LLA to ECEF and ENU rotation for static_transform_publisher.")
parser.add_argument("latitude", type=float, help="Latitude in decimal degrees")
parser.add_argument("longitude", type=float, help="Longitude in decimal degrees")
parser.add_argument("altitude", type=float, help="Altitude in meters")
args = parser.parse_args()

lat = args.latitude
lon = args.longitude
alt = args.altitude

# 1. Convert LLA to ECEF
transformer = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)  # WGS84 3D to ECEF
ecef_x, ecef_y, ecef_z = transformer.transform(lon, lat, alt)

# 2. Compute rotation from ECEF to ENU
lat_rad = np.radians(lat)
lon_rad = np.radians(lon)

# Define the rotation matrix from ECEF to ENU
rot_matrix = np.array([
    [-np.sin(lon_rad),              np.cos(lon_rad),               0],
    [-np.sin(lat_rad)*np.cos(lon_rad), -np.sin(lat_rad)*np.sin(lon_rad), np.cos(lat_rad)],
    [np.cos(lat_rad)*np.cos(lon_rad),  np.cos(lat_rad)*np.sin(lon_rad),  np.sin(lat_rad)]
])

# 3. Convert rotation matrix to quaternion
r = R.from_matrix(np.transpose(rot_matrix))
quat = r.as_quat()  # returns [x, y, z, w]

# Output in the format expected by static_transform_publisher
print("Paste this into static transform arguments:")
print(f' "{ecef_x:.6f}" , "{ecef_y:.6f}" , "{ecef_z:.6f}" , "{quat[0]:.6f} " , "{quat[1]:.6f}" ,  "{quat[2]:.6f}"  , "{quat[3]:.6f}" ')
