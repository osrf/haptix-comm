@rem Run CMake, pointing to sibling directories containing dependencies.

@set build_type=Release
@if not "%1"=="" set build_type=%1

@set build_bitness=32
@if not "%2"=="" set build_bitness=%2

@set BOOST_PATH=%cd%\..\..\boost_1_56_0
@set BOOST_LIBRARY_DIR=%BOOST_PATH%\lib64-msvc-12.0

@echo Configuring for build type %build_type%
cmake -DCMAKE_PREFIX_PATH="..\ign-transport\build\install\%build_type%" -DPROTOBUF_SRC_ROOT_FOLDER="..\..\protobuf-2.6.0-win%build_bitness%-vc12" -DCPPZMQ_HEADER_PATH="..\cppzmq" -DZeroMQ_ROOT_DIR="..\ZeroMQ 3.2.4" -DBOOST_ROOT:STRING="%BOOST_PATH%" -DBOOST_LIBRARYDIR:STRING="%BOOST_LIBRARY_DIR%" -DCMAKE_INSTALL_PREFIX="install/%build_type%" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE="%build_type%" .. 
@if %errorlevel% neq 0 exit /b %errorlevel%
@echo Configuration complete.  To build, run `nmake`
