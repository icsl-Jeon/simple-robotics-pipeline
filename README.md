# Starter code for frontend code for robotic system

The documentation of this repo can be found [here](https://motion-boseong.vercel.app/design-pattern/frontend).

![img](/img/pipeline.png)

## Features

This simple project is designed as a low to middle layer of frontend.
Observing adaptor pattern, I assumed you can use whatever higher level of frontend such as Qt or ROS.

It demonstrates how we can write a clean frontend code for robots considering:

- Event driven flow management (state transition)
- Strategy pattern for motion policy
- Clearcut frontend and backend structure

## Installation

This project is a pure cmake project written C++.

```sh
mkdir build && cd build
cmake ..
make
```

## TODO

- I will check this prject can be seamless used from other CMake project.
