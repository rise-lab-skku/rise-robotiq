# Robotiq without ROS

## Setup

Add permissions to USB port.

```sh
sudo usermod -a -G dialout $USER
sudo chmod 777 /dev/ttyUSB0
```

Install dependencies.

```sh
pip install -U pymodbus==2.5.3
```

## Usage

### Robotiq 3F

```sh
python NoRosRtuNode.py /dev/ttyUSB0
```
