import setup_path
import furosim
import ast
import cv2
import json
import math
import numpy as np
from pathlib import Path


# connect to the AirSim simulator
client = furosim.VehicleClient()
client.confirmConnection()

vehicle_name = "AuvSimple"
sonar_name = "GSonar"
save_to_disk = False
save_dir = "sonar_logs"
range_px = 600

settings_text = client.getSettingsString()
if not settings_text:
    raise ValueError("AirSim settings are empty.")

settings = json.loads(settings_text)
sensor_settings = settings["Vehicles"][vehicle_name]["Sensors"][sonar_name]
n_samples = int(sensor_settings["NumberOfSamples"])
n_beams = int(sensor_settings["NumberOfBeams"])
sonar_image_shape = (n_samples, n_beams)
h_fov_rad = math.radians(float(sensor_settings["HorizontalFOV"]))
min_range = float(sensor_settings["MinRange"])
max_range = float(sensor_settings["MaxRange"])

color_map = {}
with open(Path(__file__).resolve().parents[2] / "docs" / "seg_rgbs.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        idx, rgb = line.split("\t")
        color_map[int(idx)] = ast.literal_eval(rgb)

lut = np.zeros((max(color_map) + 1, 3), dtype=np.uint8)
for idx, rgb in color_map.items():
    lut[idx] = rgb

# build polar -> fan remap once
half_fov = h_fov_rad / 2.0
yrmax = max_range * math.sin(half_fov)
out_w = range_px
out_h = max(1, int(round(range_px * (2.0 * yrmax) / (max_range - min_range))))

x_lin = np.linspace(min_range, max_range, out_w)
y_lin = np.linspace(-yrmax, yrmax, out_h)
X, Y = np.meshgrid(x_lin, y_lin)
R = np.hypot(X, Y)
Th = np.arctan2(Y, X)
in_fan = (R >= min_range) & (R <= max_range) & (Th >= -half_fov) & (Th <= half_fov)
map_y = ((R - min_range) / (max_range - min_range) * (n_samples - 1)).astype(np.float32)
map_x = ((Th + half_fov) / h_fov_rad * (n_beams - 1)).astype(np.float32)
map_x[~in_fan] = -1.0
map_y[~in_fan] = -1.0

if save_to_disk:
    import os
    os.makedirs(save_dir, exist_ok=True)


while True:
    sonar_data = client.getGpuSonarData(sonar_name, vehicle_name)
    if np.asarray(sonar_data.point_cloud).size < 3:
        print("\tNo points received from Sonar data")
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        continue

    seg_img = np.array(sonar_data.segmentation_sonar, dtype=np.float32).reshape(sonar_image_shape).astype(np.uint8)

    # INTER_NEAREST so label IDs are not interpolated/averaged
    seg_fan = cv2.remap(
        seg_img, map_x, map_y,
        interpolation=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )
    seg_fan = np.rot90(seg_fan, 1)
    seg_fan = np.fliplr(seg_fan)
    seg_fan_color = lut[seg_fan]

    cv2.imshow("GSonar Fan LabelIds", seg_fan)
    cv2.imshow("GSonar Fan Segmentation", seg_fan_color)

    if save_to_disk:
        label_prefix = f"{save_dir}/{sonar_data.time_stamp}_fan_labelIds.png"
        color_prefix = f"{save_dir}/{sonar_data.time_stamp}_fan_color.png"
        cv2.imwrite(label_prefix, seg_fan)
        cv2.imwrite(color_prefix, seg_fan_color)
        print(f"\t{label_prefix} has been saved")

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break


cv2.destroyAllWindows()
