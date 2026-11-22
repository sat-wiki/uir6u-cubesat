# UIR6U CubeSat

This is a SDR-based amateur satellite project based on STM32H743VIT6 microcontroller. Feel free to explore and contribute!

## Setup

**Prerequisites:**

- STM32CubeIDE for Visual Studio Code
- STM32CubeMX (optional, for peripheral configuration)

STM32CubeIDE is not required for this project.

```bash
$ git clone https://github.com/saurlax/uir6u-cubesat.git
```

Then open the project folder in VS Code, the extension should automatically detect the project and set up the environment.

## Build the firmware

Run these commands from the repository root in a terminal with CMake 3.22 or later, Ninja, and the STM32 `starm-clang` toolchain on `PATH`.

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

The presets produce `build/Debug/cubesat.elf` and `build/Release/cubesat.elf`. Build Debug before running the simulator so it can inspect FreeRTOS and application symbols.

## Run the host tests

Use a native C compiler such as GCC, plus CMake, Ninja, and [uv](https://docs.astral.sh/uv/getting-started/installation/). Both the ground tool and CTest use the repository's uv project with Python 3.10. To encode a sample frame from the repository root, run `uv run --locked tools/bench_frame.py encode 1 4660 010203`.

The three host tests exercise the BPSK and packet logic, SPI NAND recovery after simulated power loss, and I/Q exchange with the ground tool.

```sh
cmake -S tests -B build/host-tests -G Ninja -DTTC_TEST_RENODE=OFF
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

## Run the FreeRTOS simulation

Renode boots the Debug ELF on its STM32H743 model. The suite checks scheduler startup and watchdog GPIO timing, health reports and fault inputs, and time transfer on both CAN buses. The simulated board has no RF chips or NAND, so the transmit test checks that the power amplifier stays off when those devices are unavailable.

On Windows, extract the [Renode 1.17.0 portable archive](https://github.com/renode/renode/releases/tag/v1.17.0) under `build/renode-tools/`. Run the following commands in PowerShell to configure CTest. CTest starts the [Renode test runner](https://renode.readthedocs.io/en/latest/introduction/testing.html) through uv with the locked `renode` dependency group.

```powershell
$renodeDir = (Resolve-Path build/renode-tools/renode_1.17.0-portable).Path
cmake -S tests -B build/renode-tests -G Ninja -DTTC_TEST_RENODE=ON `
    "-DRENODE_TEST=$renodeDir/renode-test.bat"
cmake --build build/renode-tests
```

Run the simulator suite alone, or run it together with all three host tests.

```sh
ctest --test-dir build/renode-tests -R stm32h743_freertos --output-on-failure
ctest --test-dir build/renode-tests --output-on-failure
```

The simulator writes its per-case results to `build/renode-tests/renode-results/report.html`. After changing firmware code, rebuild the Debug ELF before rerunning CTest.
