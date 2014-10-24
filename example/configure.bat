@rem Run CMake, pointing to sibling directories containing dependencies.
@rem cmake -DCMAKE_PREFIX_PATH="..\..\ign-transport\build\install;..\build\install" -DPROTOBUF_SRC_ROOT_FOLDER="..\..\..\protobuf-2.6.0-win32-vc12" -DCPPZMQ_HEADER_PATH="..\..\cppzmq" -DZeroMQ_ROOT_DIR="..\..\ZeroMQ 3.2.4" -DCMAKE_INSTALL_PREFIX="install" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release .. 
cmake -DCMAKE_PREFIX_PATH="..\..\ign-transport\build\install;..\build\install" -DPROTOBUF_SRC_ROOT_FOLDER="..\..\..\protobuf-2.6.0-win32-vc12" -DCPPZMQ_HEADER_PATH="..\..\cppzmq" -DZeroMQ_ROOT_DIR="..\..\ZeroMQ 3.2.4" -DCMAKE_INSTALL_PREFIX="install" -G "Visual Studio 12" -DCMAKE_BUILD_TYPE=Release .. 
@if %errorlevel% neq 0 exit /b %errorlevel%
@echo Configuration complete.  To build, run `nmake`

