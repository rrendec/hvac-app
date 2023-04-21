hvac-app
========

This is the source code component of an embedded HVAC controller. The hardware
design component is [hvac-eda](https://github.com/rrendec/hvac-eda).

The system is designed to run on a Raspberry Pi, and I am currently using it to
control the HVAC in my home. Some aspects of the system are hard-coded into the
source code, such as the external sensors and their Modbus addresses/registers.
Using different parts (or a different setup) would require improving the code to
allow further customization through the configuration file or modifying the code
directly.

## Features

Currently, the system implements the functions of a basic thermostat, an ERV
controller, and a humidifier controller, with minimal correlation between the
three of them. The goal is to create a smarter controller by adding correlation
to optimize the energy consumption and overall system efficiency.

The following features/functions are already implemented:
* Reading of external temperature/humidity sensors over Modbus.
* Basic thermostat control with dual heat/cool mode and distinct setpoints for
  cooling and heating.
* Basic HRV/ERV control with fixed recirculation/intermittent/low/high settings.
* Basic humidifier control with a single humidity setpoint and water flow
  control though water valve pulse length modulation.
* Embedded JSON based REST API over HTTP for controlling the run mode (heating,
  cooling, etc.), adjusting the setpoints, and reading status and sensor data.
* Embedded HTML/JavaScript web UI for controlling the system.
* Data telemetry to an external time-series database for graphic visualization,
  logging and analysis purposes. Currently [graphite](https://graphiteapp.org/)
  is supported.
* Built-in emulator (of the sensors and control peripherals) that allows the
  code to be run and tested on any standard system.

## Motivation

* What happens in my home stays in my home. Most (all?) commercially available
  solutions share anything and everything they can with the cloud service of
  their manufacturer. And yes, privacy *does* matter.
* The ability to control the HVAC in my home even if my internet connection
  stops working.
* The ability to integrate all HVAC subsystems (furnace/ERV/humidifier) into a
  single centralized controller to optimize energy consumption and overall
  system efficiency.
* The ability to fiddle with the control algorithms, experiment and improve.
* To have fun. I am a professional software designer, but also an electronics
  hobbyist and enthusiast.

# Prerequisites

## Install required packages (Debian/Ubuntu/Raspbian)

```
sudo apt install cmake libmodbus-dev libgpiod-dev libsystemd-dev civetweb libcivetweb-dev libcjson-dev
```

## Install required packages (Fedora)

```
sudo dnf install cmake libmodbus-devel libgpiod-devel systemd-devel cjson-devel
```

### civetweb

Get the source code:
```
git clone https://github.com/civetweb/civetweb.git
cd civetweb
git checkout v1.13
```

Build:
```
cmake -B output/build -DCMAKE_BUILD_TYPE=None -DCIVETWEB_BUILD_TESTING=OFF -DBUILD_SHARED_LIBS=ON
make -C output/build
cmake -DCMAKE_INSTALL_PREFIX=output/install -P output/build/cmake_install.cmake
```

Install:
```
sudo cp -df output/install/bin/civetweb /usr/local/bin/
sudo cp -df output/install/include/civetweb.h /usr/local/include/
sudo cp -df output/install/lib64/libcivetweb.* /usr/local/lib64/
```

Add `/usr/local/lib64` to the linker path (temporarily):
```
sudo ldconfig /usr/local/lib64
```

Add `/usr/local/lib64` to the linker path (permanently):
```
echo /usr/local/lib64 | sudo tee /etc/ld.so.conf.d/local.conf
sudo ldconfig
```

See [RHBZ #144967](https://bugzilla.redhat.com/show_bug.cgi?id=144967)

# Build instructions

## Debug build

```
cmake -B build -DCMAKE_BUILD_TYPE=Debug
make -C build
```

## Release build

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build
```

## Debug build with emulator enabled

```
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DWITH_EMULATOR=ON
make -C build
```

## CMake configuration using the Text User Interface (TUI)

```
ccmake -B build
```

If configuring for the first time, use the `c` option to create an initial
configuration. Otherwise, the `-B` command line parameter can be omitted.

# Install instructions

```
sudo cp build/hvac /usr/local/bin/
sudo mkdir -p /usr/local/etc/hvac
sudo cp config.json.sample /usr/local/etc/hvac/config.json
sudo mkdir -p /usr/local/share/hvac
sudo cp -rf web /usr/local/share/hvac
sudo mkdir -p /var/local/hvac
sudo chown pi: /var/local/hvac
sudo mkdir -p /usr/local/lib/systemd/system
sudo cp systemd/hvac.service /usr/local/lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable --now hvac
sudo civetweb -A /usr/local/etc/hvac/htpasswd api user 1234
```

# Web API examples

```
curl -i -u user:1234 --digest http://localhost:8080/api/sensordata
curl -i -u user:1234 --digest http://localhost:8080/api/ctrldata
curl -i -u user:1234 --digest http://localhost:8080/api/rundata
curl -i -u user:1234 --digest -X POST -d '{"furnace_mode": "heat"}' -H "Content-Type: application/json" http://localhost:8080/api/rundata
```
