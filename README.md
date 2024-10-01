# libcanard

## How to build

```
mkdir build && cd build
```

```
cmake ..
```

```
make
```

## How to setup can network

```
cd can_set_up
```

```
sudo chmod +x setup_can.bash
```


## How to execute the esc_node

CAN Network setup
```
./setup_can.bash
```

Check the network
```
ifconfig can0
```

Navigate to the build folder and then execute the node

```
./esc_node can0
```
