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
camera_name = sonar_name
detection_image_type = furosim.ImageType.DepthPerspective
mesh_name = "bricks*"
use_shadow = True
shadow_margin_px = 2
range_px = 600
arc_samples = 32  # samples per arc/ray edge when building the fan-image AABB

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

# final fan image shape after rot90(k=1) + fliplr is (out_w, out_h)
fan_h, fan_w = out_w, out_h


def polar_bbox_to_fan_aabb(range_min_m, range_max_m, az_min, az_max):
    """Sample the polar-bbox boundary (two arcs + two rays), project to the
    fan image, and return an axis-aligned bbox (x_min, y_min, x_max, y_max)."""
    az_lin = np.linspace(az_min, az_max, arc_samples)
    r_lin = np.linspace(range_min_m, range_max_m, arc_samples)
    xs = np.concatenate([
        range_min_m * np.cos(az_lin),
        range_max_m * np.cos(az_lin),
        r_lin * np.cos(az_min),
        r_lin * np.cos(az_max),
    ])
    ys = np.concatenate([
        range_min_m * np.sin(az_lin),
        range_max_m * np.sin(az_lin),
        r_lin * np.sin(az_min),
        r_lin * np.sin(az_max),
    ])
    col_pre = (xs - min_range) / (max_range - min_range) * (out_w - 1)
    row_pre = (ys + yrmax) / (2.0 * yrmax) * (out_h - 1)
    # rot90(k=1) then fliplr: (r, c) -> ((out_w - 1) - c, (out_h - 1) - r)
    fan_y = (out_w - 1) - col_pre
    fan_x = (out_h - 1) - row_pre
    fan_x = np.clip(fan_x, 0, fan_w - 1)
    fan_y = np.clip(fan_y, 0, fan_h - 1)
    return (
        int(np.floor(fan_x.min())),
        int(np.floor(fan_y.min())),
        int(np.ceil(fan_x.max())),
        int(np.ceil(fan_y.max())),
    )


# set detection radius in [cm]
client.simSetDetectionFilterRadius(camera_name, detection_image_type, 200 * 100, vehicle_name=vehicle_name)
# add desired object name to detect in wild card/regex format
client.simAddDetectionFilterMeshName(camera_name, detection_image_type, mesh_name, vehicle_name=vehicle_name)


while True:
    sonar_data = client.getGpuSonarData(sonar_name, vehicle_name)
    if np.asarray(sonar_data.point_cloud).size < 3:
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
    fan = np.rot90(fan, 1)
    fan = np.fliplr(fan)
    fan = cv2.cvtColor(fan, cv2.COLOR_GRAY2BGR)

    response = client.simGetImages(
        [furosim.ImageRequest(camera_name, detection_image_type, True, False)],
        vehicle_name=vehicle_name,
    )[0]
    depth_image = np.array(response.image_data_float, dtype=np.float32).reshape(response.height, response.width)

    detections = client.simGetDetections(camera_name, detection_image_type, vehicle_name=vehicle_name)
    if detections:
        for detection in detections:
            x_min = max(int(detection.box2D.min.x_val), 0)
            y_min = max(int(detection.box2D.min.y_val), 0)
            x_max = min(int(detection.box2D.max.x_val), response.width - 1)
            y_max = min(int(detection.box2D.max.y_val), response.height - 1)

            box_min = detection.box3D.min
            box_max = detection.box3D.max
            corners = np.array([
                [box_min.x_val, box_min.y_val, box_min.z_val],
                [box_max.x_val, box_min.y_val, box_min.z_val],
                [box_max.x_val, box_max.y_val, box_min.z_val],
                [box_min.x_val, box_max.y_val, box_min.z_val],
                [box_min.x_val, box_min.y_val, box_max.z_val],
                [box_max.x_val, box_min.y_val, box_max.z_val],
                [box_max.x_val, box_max.y_val, box_max.z_val],
                [box_min.x_val, box_max.y_val, box_max.z_val],
            ], dtype=np.float32)
            ranges = np.linalg.norm(corners, axis=1)
            azimuths = np.arctan2(corners[:, 1], corners[:, 0])
            valid = np.isfinite(ranges) & np.isfinite(azimuths)
            valid &= ranges > 0
            valid &= corners[:, 0] > 0
            if not np.any(valid):
                continue

            box_ranges = np.clip(ranges[valid], min_range, max_range)
            box_azimuths = np.clip(azimuths[valid], -half_fov, half_fov)
            box_range_min_m = float(box_ranges.min())
            box_range_max_m = float(box_ranges.max())
            azimuth_min = float(box_azimuths.min())
            azimuth_max = float(box_azimuths.max())

            range_min_m = box_range_min_m
            range_max_m = box_range_max_m

            if x_max > x_min and y_max > y_min:
                depth_y_min = max(y_min - shadow_margin_px, 0) if use_shadow else y_min
                depth_roi = depth_image[depth_y_min:y_max + 1, x_min:x_max + 1]
                valid_depth = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]
                if valid_depth.size:
                    valid_depth = np.clip(valid_depth, min_range, max_range)

                    if not use_shadow:
                        box_range_margin = max(0.1, 0.1 * (box_range_max_m - box_range_min_m))
                        depth_mask = valid_depth <= min(max_range, box_range_max_m + box_range_margin)
                        if np.any(depth_mask):
                            valid_depth = valid_depth[depth_mask]

                    range_min_m = float(np.percentile(valid_depth, 0.2))
                    range_max_m = float(np.percentile(valid_depth, 99))

            fx_min, fy_min, fx_max, fy_max = polar_bbox_to_fan_aabb(
                range_min_m, range_max_m, azimuth_min, azimuth_max
            )

            cv2.rectangle(fan, (fx_min, fy_min), (fx_max, fy_max), (255, 0, 0), 2)
            cv2.putText(
                fan,
                detection.name,
                (fx_min, max(fy_min - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (36, 255, 12),
                1,
            )

    cv2.imshow(sonar_name + " Fan 2D BBox", fan)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        client.simClearDetectionMeshNames(camera_name, detection_image_type, vehicle_name=vehicle_name)
    elif key == ord('a'):
        client.simAddDetectionFilterMeshName(camera_name, detection_image_type, mesh_name, vehicle_name=vehicle_name)


cv2.destroyAllWindows()
