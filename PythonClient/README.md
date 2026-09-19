# FURo-Sim Python Client

Python API for [FURo-Sim](https://github.com/heekyu-kweon/FURo-Sim).

## Layout
- `furosim/`: package source (`client.py`, `types.py`, `utils.py`, `pfm.py`)
- `auv/`: AUV examples (`hello_auv.py`, `waypoint_demo.py`, `ocean_current.py`)
- `fls/`: Forward-Looking Sonar examples (`gpu_sonar.py`, annotators, segmentation, pointcloud)
- `sss/`: Side-Scan Sonar examples (`gpu_sidescan.py`, segmentation)
- `computer_vision/`: segmentation / depth / IR capture utilities
- `detection/`: object detection helper
- `environment/`: weather, lighting, texture, marker control

## Dependencies
```
pip install -r requirements.txt
```

`msgpack-rpc-python` may require administrator/sudo on first install.

## Usage
```python
import furosim
client = furosim.AuvClient()
client.confirmConnection()
```

See `auv/hello_auv.py` for a runnable example.
