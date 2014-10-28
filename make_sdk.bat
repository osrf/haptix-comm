@rem Setup directories
set cwd=%cd%
set tmpdir=%cwd%\hx_gz_sdk_tmp
rmdir "%tmpdir%" /S /Q
mkdir "%tmpdir%"
cd "%tmpdir%"

@rem Download stuff.  Note that bitsadmin requires an absolute path.
bitsadmin /transfer "Download ZeroMQ" http://packages.osrfoundation.org/win32/deps/zeromq-3.2.4-x86.zip "%tmpdir%\zeromq-3.2.4-x86.zip"
bitsadmin /transfer "Download cppzmq" http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip "%tmpdir%\cppzmq-noarch.zip"
bitsadmin /transfer "Download Protobuf" http://packages.osrfoundation.org/win32/deps/protobuf-2.6.0-win32-vc12.zip "%tmpdir%\protobuf-2.6.0-win32-vc12.zip"
bitsadmin /transfer "Download unzip" http://stahlworks.com/dev/unzip.exe "%tmpdir%\unzip.exe"
bitsadmin /transfer "Download zip" http://stahlworks.com/dev/zip.exe "%tmpdir%\zip.exe"

@rem Unzip stuff
unzip zeromq-3.2.4-x86.zip
unzip cppzmq-noarch.zip
unzip protobuf-2.6.0-win32-vc12.zip

@rem Clone stuff
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport
hg up windows_linux_fixes
cd ..
@rem This repository requires a username and password.  Don't know 
@rem whether SSH keys work in Windows.
hg clone https://bitbucket.org/osrf/haptix_comm
cd haptix_comm
hg up more_windows
cd ..

@rem Build ign-transport in Debug
cd ign-transport
mkdir build
cd build
call ..\configure Debug
nmake VERBOSE=1 > build_log
nmake install
cd ..\..

@rem Build haptix_comm in Debug
cd haptix_comm
mkdir build
cd build
call ..\configure Debug
nmake VERBOSE=1 > build_log
nmake install
cd ..\..

@rem Build ign-transport in Release
cd ign-transport
cd build
del CMakeCache.txt
call ..\configure Release
nmake VERBOSE=1 > build_log
nmake install
cd ..\..

@rem Build haptix_comm in Release
cd haptix_comm
cd build
del CMakeCache.txt
call ..\configure Release
nmake VERBOSE=1 > build_log
nmake install
cd ..\..

@rem Package it all up
@rem Our goal here is to create an "install" layout for all the stuff
@rem needed to use haptix_comm.  That layout can be then be zipped and
@rem distributed.  Lots of assumptions are being made here.

set installdir=%cwd%\hx_gz_sdk
rmdir "%installdir%" /S /Q
mkdir "%installdir%"
mkdir "%installdir%\deps"
xcopy "protobuf-2.6.0-win32-vc12" "%installdir%\deps\protobuf-2.6.0-win32-vc12" /s /e /i
xcopy "ZeroMQ 3.2.4" "%installdir%\deps\ZeroMQ 3.2.4" /s /e /i
mkdir "%installdir%\deps\ign-transport"
xcopy "ign-transport\build\install\Release" "%installdir%\deps\ign-transport\Release" /s /e /i
xcopy "ign-transport\build\install\Debug" "%installdir%\deps\ign-transport\Debug" /s /e /i
mkdir "%installdir%\haptix_comm"
xcopy "haptix_comm\build\install\Release" "%installdir%\haptix_comm\Release" /s /e /i
xcopy "haptix_comm\build\install\Debug" "%installdir%\haptix_comm\Debug" /s /e /i
xcopy "haptix_comm\haptix_comm.props" "%installdir%"
cd ..
"%tmpdir%\zip" -r hx_gz_sdk-0.0.0.zip hx_gz_sdk
