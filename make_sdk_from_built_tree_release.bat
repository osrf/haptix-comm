set cwd=%cd%
set installdir=%cwd%\hx_gz_sdk
rmdir "%installdir%" /S /Q
mkdir "%installdir%"
mkdir "%installdir%\deps\protobuf-2.6.0-win32-vc12\vsprojects\Release"
:: Protobuf
xcopy "protobuf-2.6.0-win32-vc12\vsprojects\Release\*.lib" "%installdir%\deps\protobuf-2.6.0-win32-vc12\vsprojects\Release" /s /e /i
xcopy "protobuf-2.6.0-win32-vc12\vsprojects\google" "%installdir%\deps\protobuf-2.6.0-win32-vc12\vsprojects\google" /s /e /i
:: ZeroMQ
xcopy "ZeroMQ 3.2.4\COPYING*" "%installdir%\deps\ZeroMQ 3.2.4" /s /e /i
xcopy "ZeroMQ 3.2.4\bin\libzmq-v120-mt-3*" "%installdir%\deps\ZeroMQ 3.2.4\bin" /s /e /i
::xcopy "ZeroMQ 3.2.4\bin\msvc*" "%installdir%\deps\ZeroMQ 3.2.4\bin" /s /e /i
xcopy "ZeroMQ 3.2.4\include" "%installdir%\deps\ZeroMQ 3.2.4\include" /s /e /i
xcopy "ZeroMQ 3.2.4\lib\libzmq-v120-mt-3*" "%installdir%\deps\ZeroMQ 3.2.4\lib" /s /e /i
:: Ignition transport
mkdir "%installdir%\deps\ign-transport"
xcopy "ign-transport\build\install\Release\include" "%installdir%\deps\ign-transport\Release\include" /s /e /i
xcopy "ign-transport\build\install\Release\lib" "%installdir%\deps\ign-transport\Release\lib" /s /e /i
:: haptix-comm
mkdir "%installdir%\haptix-comm"
xcopy "haptix-comm\build\install\Release\include" "%installdir%\haptix-comm\Release\include" /s /e /i
xcopy "haptix-comm\build\install\Release\lib" "%installdir%\haptix-comm\Release\lib" /s /e /i
xcopy "haptix-comm\haptix-comm.props" "%installdir%"
@rem "%tmpdir%\zip" -r hx_gz_sdk-0.0.0.zip hx_gz_sdk

