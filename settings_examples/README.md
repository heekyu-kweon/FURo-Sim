# Settings Examples

This directory contains `settings.json` example files for FURo-Sim.

## Files

| File | SimMode | Description |
|------|---------|-------------|
| `auv_basic.json` | Auv | AUV with default sensors (IMU, Pressure, DVL) |
| `auv_multi_agent.json` | Auv | 2x AUV with default sensors (IMU, Pressure, DVL) |
| `auv_with_didson.json` | Auv | AUV + DIDSON (Forward-Looking Sonar) |
| `auv_with_oculus.json` | Auv | AUV + Oculus (Forward-Looking Sonar) |
| `auv_with_sss.json` | Auv | AUV + Side-Scan Sonar (SSS) |

## Sensor Type IDs

| SensorType | ID |
|------------|----|
| IMU | 2 |
| GPS | 3 |
| LiDAR | 6 |
| FLS (GpuSonar) | 7 |
| SSS (GpuSidescanSonar) | 8 |
| DVL | 9 |
| Pressure | 10 |

## Usage

Copy the desired example file and rename it to `settings.json`:

**Windows**
```
copy settings_examples\auv_basic.json %USERPROFILE%\Documents\FURo-Sim\settings.json
```

**Linux**
```bash
cp settings_examples/auv_basic.json ~/Documents/FURo-Sim/settings.json
```
