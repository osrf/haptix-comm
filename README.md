# Haptix-comm

** Haptix-comm classes and functions for communication.**

Haptix-comm is a component in the Haptix project.

  [http://haptix.org](http://haptix.org)

## Continuous integration

Please refer to the [drone.io
job](https://drone.io/bitbucket.org/osrf/haptix_comm).

[![Build Status](https://drone.io/bitbucket.org/osrf/haptix_comm/status.png)](https://drone.io/bitbucket.org/osrf/haptix_comm/latest)


## Dependencies

The following dependencies are required to compile ignition-transport from
source:

 - ignition-transport
 - cmake
 - mercurial
 - C compiler.

**Note:** *if you are using an Ubuntu platform previous to Saucy, you will need to install zeromq from source, since there is no libzmq3-dev*

    sudo apt-get install build-essential cmake mercurial

    hg clone https://bitbucket.org/ignitionrobotics/ign-transport
    cd ign-transport
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    sudo make install
    cd ..

## Installation

Standard installation can be performed in UNIX systems using the following
steps:

 - mkdir build/
 - cd build/
 - cmake ..
 - sudo make install

## Uninstallation

To uninstall the software installed with the previous steps:

 - cd build/
 - sudo make uninstall
