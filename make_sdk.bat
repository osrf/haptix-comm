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

@rem Unzip stuff
call :create_unzip_script
call :unzip %zeromq_zip_name%
call :unzip cppzmq-noarch.zip
call :unzip %protobuf_zip_name%

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

call :creation_zip_script
call :zip %cd%hx_gz_sdk %cd%hx_gz_sdk-0.0.0.zip

:: ##################################
:create_unzip_script - Create the unzip script to run unzip command
::
REM Unzip script from http://jagaroth.livejournal.com/69147.html 

REM This script upzip's files...
> j_unzip.vbs ECHO '
>> j_unzip.vbs ECHO ' Dim ArgObj, var1, var2
>> j_unzip.vbs ECHO Set ArgObj = WScript.Arguments
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO If (Wscript.Arguments.Count ^> 0) Then
>> j_unzip.vbs ECHO. var1 = ArgObj(0)
>> j_unzip.vbs ECHO Else
>> j_unzip.vbs ECHO. var1 = ""
>> j_unzip.vbs ECHO End if
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO If var1 = "" then
>> j_unzip.vbs ECHO. strFileZIP = "example.zip"
>> j_unzip.vbs ECHO Else
>> j_unzip.vbs ECHO. strFileZIP = var1
>> j_unzip.vbs ECHO End if
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO 'The location of the zip file.
>> j_unzip.vbs ECHO REM Set WshShell = CreateObject("Wscript.Shell")
>> j_unzip.vbs ECHO REM CurDir = WshShell.ExpandEnvironmentStrings("%%cd%%")
>> j_unzip.vbs ECHO Dim sCurPath
>> j_unzip.vbs ECHO sCurPath = CreateObject("Scripting.FileSystemObject").GetAbsolutePathName(".")
>> j_unzip.vbs ECHO strZipFile = sCurPath ^& "\" ^& strFileZIP
>> j_unzip.vbs ECHO 'The folder the contents should be extracted to.
>> j_unzip.vbs ECHO outFolder = sCurPath ^& "\"
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO. WScript.Echo ( "Extracting file " ^& strFileZIP)
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO Set objShell = CreateObject( "Shell.Application" )
>> j_unzip.vbs ECHO Set objSource = objShell.NameSpace(strZipFile).Items()
>> j_unzip.vbs ECHO Set objTarget = objShell.NameSpace(outFolder)
>> j_unzip.vbs ECHO intOptions = 256
>> j_unzip.vbs ECHO objTarget.CopyHere objSource, intOptions
>> j_unzip.vbs ECHO.
>> j_unzip.vbs ECHO. WScrip.Echo ( "Extracted." )
>> j_unzip.vbs ECHO.
goto :EOF

::
:creation_zip_script - Create the zip
:: arg1 input folder (ABSOLUTE PATH)
:: arg2 output zip file (ABSOLUTE PATH)

echo Set objArgs = WScript.Arguments > _zipIt.vbs
echo InputFolder = objArgs(0) >> _zipIt.vbs
echo ZipFile = objArgs(1) >> _zipIt.vbs
echo CreateObject("Scripting.FileSystemObject").CreateTextFile(ZipFile, True).Write "PK" ^& Chr(5) ^& Chr(6) ^& String(18, vbNullChar) >> _zipIt.vbs
echo Set objShell = CreateObject("Shell.Application") >> _zipIt.vbs
echo Set source = objShell.NameSpace(InputFolder).Items >> _zipIt.vbs
echo objShell.NameSpace(ZipFile).CopyHere(source) >> _zipIt.vbs
echo wScript.Sleep 2000 >> _zipIt.vbs
goto :EOF

:: ##################################
:zip - Compress a file
:: arg1 directory to compress 
:: arg2 file to compress

@ echo "Zipping %~2 (compressing %~1)"
cscript _zipIt.vbs %~1 %~2
goto :EOF

:: ##################################
:unzip - Unizp a file
::
:: arg1 path to the zip file to uncompress

@echo "Unzip %~1 ..."
cscript //B j_unzip.vbs %~1 || goto:error
goto :EOF

:error
echo "The program is stopping with errors! Check the log" 
