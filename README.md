# elmo-solo-guitar-driver-can

Because there's no one that open sourced the driver, so i made this driver for linux using CANopen.
In other way, i've built for RS-232 too, but i'm too lazy to create a lib xd.

# Compile

```
mkdir build && cd build
cmake ..
make
sudo make install
```

# Usage

Just link your program to that lib, i've put the example on example/make.sh
