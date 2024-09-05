# Examples

## Prerequisites

To build and run the examples you will need to have the Pico SDK set up.

## Build

1. Enter the example directory
2. Create a build folder within the examplen directory. Here all the build files will be generated.
3. Enter the build directory.
4. define PICO_SDK_PATH
5. Run `cmake ..`
6. Run `make`

To sum it up:

```bash
cd examples/<example>
mkdir build
cd build
export PICO_SDK_PATH=/path/to/pico-sdk
cmake .. -DPICO_PLATFORM=rp2350 
make
```

## Run

After the build there will be a `<example>.uf2` file in the build folder. Copy this file to your perpetuo board. The sensei board should restart and run the code :party:.
