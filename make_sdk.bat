:: SDK Creation script
:: arg1 bitness [ x86 | amd64 ]


@set PLATFORM_TO_BUILD=x86
@if not "%1"=="" set PLATFORM_TO_BUILD=%1

IF %PLATFORM_TO_BUILD% == x86 (
  set BITNESS=32
) ELSE (
  REM Visual studio is accepting many keywords to compile for 64bits
  REM We need to set x86_amd64 to make express version to be able to
  REM Cross compile from x86 -> amd64
  echo "Using 64bits VS configuration"
  set BITNESS=64
  set MSVC_KEYWORD=x86_amd64
  set PLATFORM_TO_BUILD=amd64
)

echo
echo "======================="
echo "%bitness%bits SDK Generation  "
echo "======================="
echo

echo " - Configure the VC++ compilation"
set MSVC_ON_WIN64=c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat
set MSVC_ON_WIN32=c:\Program Files\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

IF exist "%MSVC_ON_WIN64%" ( 
   call "%MSVC_ON_WIN64%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE IF exist "%MSVC_ON_WIN32%" (
   call "%MSVC_ON_WIN32%" %MSVC_KEYWORD% || goto %win_lib% :error
) ELSE (
   echo "Could not find the vcvarsall.bat file"
   exit -1
)

@rem Setup directories
set cwd=%cd%
set tmpdir=%cwd%\hx_gz_sdk_tmp
rmdir "%tmpdir%" /S /Q
mkdir "%tmpdir%"
cd "%tmpdir%"

set zeromq_zip_name=zeromq-3.2.4-%PLATFORM_TO_BUILD%.zip
set protobuf_zip_name=protobuf-2.6.0-win%build_bitness%-vc12.zip

@rem Download stuff.  Note that bitsadmin requires an absolute path.
bitsadmin /transfer "Download ZeroMQ" http://packages.osrfoundation.org/win32/deps/%zeromq_zip_name% "%tmpdir%\"%zeromq_zip_name%"
bitsadmin /transfer "Download cppzmq" http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip "%tmpdir%\cppzmq-noarch.zip"
bitsadmin /transfer "Download Protobuf" http://packages.osrfoundation.org/win32/deps/%protobuf_zip_name% "%tmpdir%\"%protobuf_zip_name%"
bitsadmin /transfer "Download unzip" http://stahlworks.com/dev/unzip.exe "%tmpdir%\unzip.exe"
bitsadmin /transfer "Download zip" http://stahlworks.com/dev/zip.exe "%tmpdir%\zip.exe"

@rem Unzip stuff
unzip %zeromq_zip_name%
unzip cppzmq-noarch.zip
unzip %protobuf_zip_name%

@rem Clone stuff
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport
cd ..
hg clone https://bitbucket.org/osrf/haptix-comm haptix-comm
cd haptix-comm
cd ..

@rem Build ign-transport in Debug
cd ign-transport
mkdir build
cd build
call ..\configure Debug
nmake VERBOSE=1
nmake install
cd ..\..

@rem Build haptix-comm in Debug
cd haptix-comm
mkdir build
cd build
call ..\configure Debug
nmake VERBOSE=1
nmake install
cd ..\..

@rem Build ign-transport in Release
cd ign-transport
cd build
del CMakeCache.txt
call ..\configure Release
nmake VERBOSE=1
nmake install
cd ..\..

@rem Build haptix-comm in Release
cd haptix-comm
cd build
del CMakeCache.txt
call ..\configure Release
nmake VERBOSE=1
nmake install
cd ..\..

@rem Package it all up
@rem Our goal here is to create an "install" layout for all the stuff
@rem needed to use haptix-comm.  That layout can be then be zipped and
@rem distributed.  Lots of assumptions are being made here.

set installdir=%cwd%\hx_gz_sdk
rmdir "%installdir%" /S /Q
mkdir "%installdir%"
mkdir "%installdir%\deps"
xcopy "protobuf-2.6.0-win%BITNESS%-vc12" "%installdir%\deps\protobuf-2.6.0-win%BITNESS%-vc12" /s /e /i
xcopy "ZeroMQ 3.2.4" "%installdir%\deps\ZeroMQ 3.2.4" /s /e /i
mkdir "%installdir%\deps\ign-transport"
xcopy "ign-transport\build\install\Release" "%installdir%\deps\ign-transport\Release" /s /e /i
xcopy "ign-transport\build\install\Debug" "%installdir%\deps\ign-transport\Debug" /s /e /i
mkdir "%installdir%\haptix-comm"
xcopy "haptix-comm\build\install\Release" "%installdir%\haptix-comm\Release" /s /e /i
xcopy "haptix-comm\build\install\Debug" "%installdir%\haptix-comm\Debug" /s /e /i
xcopy "haptix-comm\haptix-comm.props" "%installdir%"
cd ..
"%tmpdir%\zip" -r hx_gz_sdk-0.0.0.zip hx_gz_sdk
