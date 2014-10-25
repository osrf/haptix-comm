@rem Our goal here is to create an "install" layout for all the stuff
@rem needed to use haptix_comm.  That layout can be then be zipped and
@rem distributed.  Lots of assumptions are being made here.

set installdir=..\hx_gz_sdk
rmdir "%installdir%" /s
mkdir "%installdir%"
mkdir "%installdir%\deps"
xcopy "..\protobuf-2.6.0-win32-vc12" "%installdir%\deps\protobuf-2.6.0-win32-vc12" /s /e /i
xcopy "..\ZeroMQ 3.2.4" "%installdir%\deps\ZeroMQ 3.2.4" /s /e /i
mkdir "%installdir%\deps\ign-transport"
xcopy "..\ign-transport\build\install\Release" "%installdir%\deps\ign-transport\Release" /s /e /i
xcopy "..\ign-transport\build\install\Debug" "%installdir%\deps\ign-transport\Debug" /s /e /i
mkdir "%installdir%\haptix_comm"
xcopy "..\haptix_comm\build\install\Release" "%installdir%\haptix_comm\Release" /s /e /i
xcopy "..\haptix_comm\build\install\Debug" "%installdir%\haptix_comm\Debug" /s /e /i
xcopy "..\haptix_comm\haptix_comm.props" "%installdir%"
