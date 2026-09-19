import setup_path
import furosim
import cv2
import json
import math
import numpy as np


# connect to the AirSim simulator
client = furosim.VehicleClient()
client.confirmConnection()

vehicle_name = "AuvSimple"
sonar_name = "GSonar"
save_to_disk = False
save_dir = "sonar_logs"
range_px = 600  # vertical pixels in the final fan image (range axis)

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

# build polar -> fan remap once (Cartesian grid -> polar sample/beam lookup)
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
    if np.asarray(sonar_data.sonar_raw_data).size < 1:
        print("\tNo data received from Sonar")
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        continue

    sonar_raw_data = np.array(sonar_data.sonar_raw_data, dtype=np.float32).reshape(sonar_image_shape)
    img = (255 * sonar_raw_data).astype(np.uint8)

    fan = cv2.remap(
        img, map_x, map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )
    # sensor at bottom, range increasing upward (matches raw2fans_didson.py)
    fan = np.rot90(fan, 1)
    fan = np.fliplr(fan)

    cv2.imshow("GSonar Fan", fan)

    if save_to_disk:
        prefix = f"{save_dir}/{sonar_data.time_stamp}_fan.png"
        cv2.imwrite(prefix, fan)
        print(f"\t{prefix} has been saved")

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break


cv2.destroyAllWindows()
