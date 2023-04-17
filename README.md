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
