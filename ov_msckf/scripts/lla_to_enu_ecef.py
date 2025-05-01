import numpy as np
from pyproj import Transformer
from scipy.spatial.transform import Rotation as R

# Utility script to get ECEF pos & rot coordinates of ENU frame centered at LLA
# Use as arguments for static transform in ros2, for visualization purposes

# Input LLA (latitude, longitude, altitude)
lat = 40.867752314626976
lon = 14.122253633809104
alt = 100

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
print("paste this into static transform arguments:")
print(f' "{ecef_x:.6f}" , "{ecef_y:.6f}" , "{ecef_z:.6f}" , "{quat[0]:.6f} " , "{quat[1]:.6f}" ,  "{quat[2]:.6f}"  , "{quat[3]:.6f}" ')

