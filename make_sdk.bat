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

@echo ""
@echo "======================="
@echo "%bitness%bits SDK Generation  "
@echo "======================="
@echo ""

@echo " - Configure the VC++ compilation"

set MSVC_ON_WIN64=c:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\vcvarsall.bat
set MSVC_ON_WIN32=c:\Program Files\Microsoft Visual Studio 12.0\VC\vcvarsall.bat

IF exist "%MSVC_ON_WIN64%" ( 
   call "%MSVC_ON_WIN64%" %MSVC_KEYWORD% || goto :error
) ELSE IF exist "%MSVC_ON_WIN32%" (
   call "%MSVC_ON_WIN32%" %MSVC_KEYWORD% || goto :error
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
set protobuf_zip_name=protobuf-2.6.0-win%BITNESS%-vc12.zip

@rem Download stuff.  Note that bitsadmin requires an absolute path.
bitsadmin /transfer "Download ZeroMQ" http://packages.osrfoundation.org/win32/deps/%zeromq_zip_name% "%tmpdir%\%zeromq_zip_name%" || goto :error
bitsadmin /transfer "Download cppzmq" http://packages.osrfoundation.org/win32/deps/cppzmq-noarch.zip "%tmpdir%\cppzmq-noarch.zip"  || goto :error
bitsadmin /transfer "Download Protobuf" http://packages.osrfoundation.org/win32/deps/%protobuf_zip_name% "%tmpdir%\%protobuf_zip_name%"  || goto :error
bitsadmin /transfer "Download 7zip" http://packages.osrfoundation.org/win32/deps/7za.exe "%tmpdir%\7za.exe"

@rem Unzip stuff
7za x %zeromq_zip_name%
7za x cppzmq-noarch.zip
7za x %protobuf_zip_name%

@rem Clone stuff
hg clone https://bitbucket.org/ignitionrobotics/ign-transport
cd ign-transport
hg tip > ignition-transport.info
cd ..

hg clone https://bitbucket.org/osrf/haptix-comm haptix-comm -b parametrize_configure
cd haptix-comm
REM set haptix_hash variable. Yes, we need need to do this for structure
for /f "delims=" %%a in ('hg id -i') do @set haptix_hash=%%a
hg tip > haptix-comm.info
cd ..

setLocal Enabledelayedexpansion
for %%b in (Debug, Release) do (

    echo "Build ign-transport in %%b"
    cd ign-transport
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 || goto :error
    nmake install
    cd ..\..

    echo "Build haptix-comm in %%b"
    cd haptix-comm
    mkdir build
    cd build
    del CMakeCache.txt
    call ..\configure %%b %BITNESS%
    nmake VERBOSE=1 || goto :error
    nmake install
    cd ..\..

    :: Package it all up
    :: Our goal here is to create an "install" layout for all the stuff
    :: needed to use haptix-comm.  That layout can be then be zipped and
    :: distributed.  Lots of assumptions are being made here.

    :: BIG HACK to concatenate the loop index
    set "build_type=%%b"
    echo.!build_type!

    set "installdir=%cwd%\hx_gz_sdk_!build_type!"
    @echo "Installation directory is: !installdir!"
      
    rmdir !installdir! /S /Q
    mkdir !installdir! || goto :error

    mkdir "!%installdir!\deps\protobuf-2.6.0-win%build_bitness%-vc12\vsprojects\%build_type%" || goto :error
    :: Protobuf
    xcopy "protobuf-2.6.0-win%build_bitness%-vc12\vsprojects\%build_type%\*.lib" "!installdir!\deps\protobuf-2.6.0-win%build_bitness%-vc12\vsprojects\%build_type%" /s /e /i || goto :error
    xcopy "protobuf-2.6.0-win%build_bitness%-vc12\vsprojects\google" "!installdir!\deps\protobuf-2.6.0-win%build_bitness%-vc12\vsprojects\google" /s /e /i
    :: ZeroMQ
    xcopy "ZeroMQ 3.2.4\COPYING*" "!installdir!\deps\ZeroMQ 3.2.4" /s /e /i
    xcopy "ZeroMQ 3.2.4\bin\libzmq-v120-mt-3*" "!installdir!\deps\ZeroMQ 3.2.4\bin" /s /e /i
    ::xcopy "ZeroMQ 3.2.4\bin\msvc*" "!installdir!\deps\ZeroMQ 3.2.4\bin" /s /e /i
    xcopy "ZeroMQ 3.2.4\include" "!installdir!\deps\ZeroMQ 3.2.4\include" /s /e /i
    xcopy "ZeroMQ 3.2.4\lib\libzmq-v120-mt-3*" "!installdir!\deps\ZeroMQ 3.2.4\lib" /s /e /i
    :: Ignition transport
    mkdir "!installdir!\deps\ign-transport"
    xcopy "ign-transport\build\install\%build_type%\include" "!installdir!\deps\ign-transport\%build_type%\include" /s /e /i
    xcopy "ign-transport\build\install\%build_type%\lib" "!installdir!\deps\ign-transport\%build_type%\lib" /s /e /i
    xcopy "ign-transport\ignition-transport.info" "!installdir!"
    :: haptix-comm
    mkdir "!installdir!\haptix-comm"
    xcopy "haptix-comm\build\install\%build_type%\include" "!installdir!\haptix-comm\%build_type%\include" /s /e /i
    xcopy "haptix-comm\build\install\%build_type%\lib" "!installdir!\haptix-comm\%build_type%\lib" /s /e /i
    xcopy "haptix-comm\haptix-comm.props" "!installdir!"

    set sdk_zip_file=hx_gz_sdk-%build_type%-%haptix_hash%-win%BITNESS%.zip

    echo "Generating SDK zip file: %sdk_zip_file%"
    "%tmpdir%\7za.exe" a -tzip ../%sdk_zip_file% "hx_gz_sdk_%build_type%\"
)
setlocal disabledelayedexpansion

goto :EOF

:error
echo "The program is stopping with errors! Check the log" 
