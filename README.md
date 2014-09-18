# Haptix-comm

** Haptix-comm classes and functions for communication.**

Haptix-comm is a component in the Haptix project.

  [http://haptix.org](http://haptix.org)

## Dependencies

The following dependencies are required to compile haptix-comm from
source:

 - ignition-transport
 - cmake
 - mercurial
 - C compiler.

Installation commands:

 - sudo apt-get install build-essential cmake mercurial
 - hg clone https://bitbucket.org/ignitionrobotics/ign-transport
 - cd ign-transport
 - mkdir build
 - cd build
 - cmake .. -DCMAKE_INSTALL_PREFIX=/usr
 - sudo make install
 - cd ..

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
