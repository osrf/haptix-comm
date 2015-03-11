@rem Run CMake, pointing to sibling directories containing dependencies.
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%
cmake -DCMAKE_PREFIX_PATH="..\..\ign-transport\build\install\%build_type%\lib\cmake\ignition-transport;..\build\install\%build_type%\lib\cmake\haptix-comm" -DPROTOBUF_SRC_ROOT_FOLDER="..\..\..\protobuf-2.6.0-win%build_bitness%-vc12" -DCPPZMQ_HEADER_PATH="..\..\cppzmq" -DZeroMQ_ROOT_DIR="..\..\ZeroMQ 3.2.4" -DCMAKE_INSTALL_PREFIX="install" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE="%build_type%" ..
@if %errorlevel% neq 0 exit /b %errorlevel%
@echo Configuration complete.  To build, run `nmake`
