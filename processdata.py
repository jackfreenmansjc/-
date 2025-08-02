# I'll integrate the UTM coordinate conversion functionality into the existing Python code.

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
from pyproj import Proj, transform

# Define WGS84 and UTM coordinate systems
wgs84 = Proj(init='epsg:4326')  # WGS84 geographic coordinate system
utm = Proj(proj='utm', zone=51, ellps='WGS84')  # UTM zone 51 (China eastern region)

# Function to convert lat/lon to UTM coordinates
def convert_to_utm(lat, lon):
    return transform(wgs84, utm, lon, lat)  # lon, lat is the required order for pyproj

# Read the input file path from arguments
input_file = sys.argv[1]

# Load data from process.txt
data = pd.read_csv(input_file, header=None, names=["timestamp", "lat", "lon", "heading_angle", "expected_heading_angle"])

# Convert lat/lon to UTM coordinates
utm_coords = data.apply(lambda row: convert_to_utm(row['lat'], row['lon']), axis=1)
data['x'] = [coord[0] for coord in utm_coords]
data['y'] = [coord[1] for coord in utm_coords]

# Convert timestamp to datetime if needed
data['timestamp'] = pd.to_datetime(data['timestamp'])

# Plot 1: GPS Coordinates (x, y) in UTM
plt.figure(figsize=(10, 6))
plt.subplot(211)
plt.plot(data['x'], data['y'], marker='o', linestyle='-', color='b', label='Boat Trajectory')
for i, row in data.iterrows():
    if i % 5 == 0:  # Add boat icon every 5 seconds
        plt.text(row['x'], row['y'], 'A', fontsize=12, color='red')
plt.xlabel("X Coordinate (UTM)")
plt.ylabel("Y Coordinate (UTM)")
plt.title("Boat Trajectory (UTM Coordinates)")
plt.grid(True)
plt.legend()

# Plot 2: Heading Angle over Time
plt.subplot(212)
plt.plot(data['timestamp'], data['heading_angle'], label='Heading Angle', color='g')
plt.plot(data['timestamp'], data['expected_heading_angle'], label='Expected Heading Angle', color='r', linestyle='--')
plt.xlabel("Time")
plt.ylabel("Angle (degrees)")
plt.title("Heading Angle vs Expected Heading Angle")
plt.legend()
plt.grid(True)

# Show the plots
plt.tight_layout()
plt.show()
