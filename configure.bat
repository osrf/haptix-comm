@rem Run CMake, pointing to sibling directories containing dependencies.
@rem the script is prepared to look for MATLAB in 64bits machines
@rem if you are running a 32bits copy of windows please modify the
@rem MATLAB PATH accordingly

@set build_type=Release
@if not "%1"=="" set build_type=%1

@set build_bitness=32
@if not "%2"=="" set build_bitness=%2

@set MATLAB_PATH=C:\Program Files\MATLAB\R2014b
@rem remove the line below if you run a windows 32bits copy
@if "%build_bitness%"=="32" set MATLAB_PATH=C:\Program Files (x86)\MATLAB\R2014b

@echo Configuring for build type %build_type%
cmake -DCMAKE_PREFIX_PATH="..\ign-transport\build\install\%build_type%" -DPROTOBUF_SRC_ROOT_FOLDER="..\..\protobuf-2.6.0-win%build_bitness%-vc12" -DCPPZMQ_HEADER_PATH="..\cppzmq" -DZeroMQ_ROOT_DIR="..\ZeroMQ 3.2.4" -DMATLAB_ROOT:STRING="%MATLAB_PATH%" -DCMAKE_INSTALL_PREFIX="install/%build_type%" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE="%build_type%" ..
@if %errorlevel% neq 0 exit /b %errorlevel%
@echo Configuration complete.  To build, run `nmake`
