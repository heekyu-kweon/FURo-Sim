import setup_path
import furosim
import cv2
import json
import math
import numpy as np


# connect to the AirSim simulator
client = furosim.VehicleClient()
client.confirmConnection()

# set sonar name and image type to request images and detections
vehicle_name = "AuvSimple"
sonar_name = "GSonar"
camera_name = sonar_name
detection_image_type = furosim.ImageType.DepthPerspective
mesh_name = "bricks*"
use_shadow = True
shadow_margin_px = 2
window_scale = 2

settings_text = client.getSettingsString()
if not settings_text:
    raise ValueError("AirSim settings are empty.")

settings = json.loads(settings_text)
sensor_settings = settings["Vehicles"][vehicle_name]["Sensors"][sonar_name]
sonar_image_shape = (
    int(sensor_settings["NumberOfSamples"]),
    int(sensor_settings["NumberOfBeams"]),
)
h_fov_rad = math.radians(float(sensor_settings["HorizontalFOV"]))
min_range = float(sensor_settings["MinRange"])
max_range = float(sensor_settings["MaxRange"])

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
    image = (255 * sonar_raw_data).astype(np.uint8)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    image = cv2.resize(
        image,
        (image.shape[1] * window_scale, image.shape[0] * window_scale),
        interpolation=cv2.INTER_NEAREST,
    )

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
            box_azimuths = np.clip(azimuths[valid], -0.5 * h_fov_rad, 0.5 * h_fov_rad)
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

            sonar_y0 = int(np.round((range_min_m - min_range) * (sonar_image_shape[0] - 1) / max(max_range - min_range, 1e-6)))
            sonar_y1 = int(np.round((range_max_m - min_range) * (sonar_image_shape[0] - 1) / max(max_range - min_range, 1e-6)))
            sonar_x0 = int(np.round((azimuth_min / h_fov_rad + 0.5) * (sonar_image_shape[1] - 1)))
            sonar_x1 = int(np.round((azimuth_max / h_fov_rad + 0.5) * (sonar_image_shape[1] - 1)))

            sonar_x_min = int(np.clip(min(sonar_x0, sonar_x1), 0, sonar_image_shape[1] - 1))
            sonar_x_max = int(np.clip(max(sonar_x0, sonar_x1), 0, sonar_image_shape[1] - 1))
            sonar_y_min = int(np.clip(min(sonar_y0, sonar_y1), 0, sonar_image_shape[0] - 1))
            sonar_y_max = int(np.clip(max(sonar_y0, sonar_y1), 0, sonar_image_shape[0] - 1))
            sonar_y_min = max(sonar_y_min - 1, 0)

            cv2.rectangle(
                image,
                (sonar_x_min * window_scale, sonar_y_min * window_scale),
                (sonar_x_max * window_scale, sonar_y_max * window_scale),
                (255, 0, 0),
                2,
            )
            cv2.putText(
                image,
                detection.name,
                (sonar_x_min * window_scale, max(sonar_y_min * window_scale - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (36, 255, 12),
                1,
            )

    cv2.imshow(sonar_name + " 2D BBox", image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        client.simClearDetectionMeshNames(camera_name, detection_image_type, vehicle_name=vehicle_name)
    elif key == ord('a'):
        client.simAddDetectionFilterMeshName(camera_name, detection_image_type, mesh_name, vehicle_name=vehicle_name)


cv2.destroyAllWindows()
