# satellite-sim

A simulated satellite constellation data generation and testbed system for [Modality](https://docs.auxon.io/modality/).

Features:
* Physics and environmental simulation provided by [42](https://github.com/ericstoneking/42)
* Constellation of 14 satellites
* Satellites have configurable IR event detection cameras: a fixed camera for wide-angle scanning and a moveable focus camera for tracking
* Satellites communicate to relay ground stations at fixed locations on the Earth
* Relay ground stations relay incoming communications to a central ground station
* IR event fusion and simulated operator happen in the central ground station
* Fault models built into all the subsystems and components with configurable point failure

## Getting Started

This assumes Modality is installed and configured (see the [Modality docs](https://docs.auxon.io/modality/installation/) for more information).

1. Update submodules and install dependencies
  ```bash
  git submodule init
  git submodule update
  git lfs pull
  sudo apt install freeglut3-dev -y
  ```
2. Install [rust](https://www.rust-lang.org/tools/install)
  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  ```
3. Run the system. This script will build `42` and the `fsw` binaries if needed. See it's `--help` message for more options.
   The `fsw` binary can be found in `target/release/fsw`.
  ```bash
  ./go.sh
  ```

## Configuration

The system has a default configuration for the satellites and their subsystems that represents nominal execution behavior.
A `toml` configuration file can be provided to override the defaults and enable the
various point failures.

* See [scenarios/example.toml](scenarios/example.toml) for an example scenario configuration file
* The default scenario configuration is defined in [fsw/src/scenario/nominal.rs](fsw/src/scenario/nominal.rs)
* The available configurations can be found in [fsw/src/scenario/config.rs](fsw/src/scenario/config.rs)

```bash
fsw --scenario scenarios/example.toml
```

## Using 42 Data Files

By default the system connects to `42`'s IPC server, and is in-loop with the simulation step function.
The system can also read from a pre-generated data file produced by `42`.

1. Modify the `42` IPC config file [config/Inp_IPC.txt](config/Inp_IPC.txt) to produce a data file instead of being a server
  ```diff
   <<<<<<<<<<<<<<< 42: InterProcess Comm Configuration File >>>>>>>>>>>>>>>>
   1                                       ! Number of Sockets
   **********************************  IPC 0   *****************************
  -TX                                      ! IPC Mode (OFF,TX,RX,TXRX,ACS,WRITEFILE,READFILE)
  +WRITEFILE                               ! IPC Mode (OFF,TX,RX,TXRX,ACS,WRITEFILE,READFILE)
   0                                       ! AC.ID for ACS mode
  "sat_log.42"                             ! File name for WRITE or READ
   SERVER                                  ! Socket Role (SERVER,CLIENT,GMSEC_CLIENT)
   localhost     10001                     ! Server Host Name, Port
   TRUE                                    ! Allow Blocking (i.e. wait on RX)
  ```
2. Run a `42` simulation. This will produce the data file `config/sat_log.42`
  ```bash
  cd 42/
  ./42 ../config
  ```
3. Run the system against the data file
  ```bash
  fsw config/sat_log.42
  ```

## Dev GUI

The system has a lightweight developer GUI to help get a sense of what the system is doing during runtime.

To enable the GUI, run with `--dev-gui` (see the `--help` message for more options and the key-map information printed to stdout).

![dev_gui.png](images/dev_gui.png)

## OpenC3 COSMOS UI

The directory [cosmos](./cosmos) contains the [OpenC3 COSMOS](https://openc3.com/) targets and UI plugin for the satellite simulation.

![cosmos_ui.png](images/cosmos_ui.png)

1. Setup the OpenC3 COSMOS environment per the [README](cosmos/README)
2. Run the system with mission-control and tle options
```bash
fsw --mission-control 127.0.0.1:9999 --tle config/SAT_TLE.txt
```

## License

See [LICENSE](./LICENSE) for more details.

Copyright 2023 [Auxon Corporation](https://auxon.io)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
