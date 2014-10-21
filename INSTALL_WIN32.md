# Installation on Windows

## Supported compilers

At this moment, compilation has been testewd on Windows 7 and is supported 
when using Visual Studio 2013. Patches for other versions are welcome.

## Note for 64bits installations

Currently, FindProtobuf.cmake is not support 64bits compilations since the
binaries are placed under vsprojects\x64\Release and that PATH is not being
used by the cmake module.

## Installation

Totally experimental, using pre-compiled binaries in a local workspace.

1. Follow the [ign-transport Windows installation
instructions](https://bitbucket.org/ignitionrobotics/ign-transport/src/default/INSTALL_WIN32.md?at=win_support)
to build and install `ign-transport`.

1. Add `haptix_comm` to that workspace:

        cd ign-ws
	hg clone https://bitbucket.org/osrf/haptix_comm
        cd haptix_comm

1. Configure and build:

	mkdir build
	cd build
        ..\configure
	nmake
	nmake install

You should now have an installation of `haptix_comm` in `ign-ws/haptix_comm/build/install`.
