set cwd=%cd%
set installdir=%cwd%\hx_gz_sdk
rmdir "%installdir%" /S /Q
mkdir "%installdir%"
mkdir "%installdir%\deps"
xcopy "protobuf-2.6.0-win32-vc12" "%installdir%\deps\protobuf-2.6.0-win32-vc12" /s /e /i
xcopy "ZeroMQ 3.2.4" "%installdir%\deps\ZeroMQ 3.2.4" /s /e /i
mkdir "%installdir%\deps\ign-transport"
xcopy "ign-transport\build\install\Release" "%installdir%\deps\ign-transport\Release" /s /e /i
xcopy "ign-transport\build\install\Debug" "%installdir%\deps\ign-transport\Debug" /s /e /i
mkdir "%installdir%\haptix-comm"
xcopy "haptix-comm\build\install\Release" "%installdir%\haptix-comm\Release" /s /e /i
xcopy "haptix-comm\build\install\Debug" "%installdir%\haptix-comm\Debug" /s /e /i
xcopy "haptix-comm\haptix-comm.props" "%installdir%"
@rem "%tmpdir%\zip" -r hx_gz_sdk-0.0.0.zip hx_gz_sdk

