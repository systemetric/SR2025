@echo off
if "%~1"=="" goto :usage

copy *.* %~1:\ > NUL

echo Success.
goto :eof

:usage 
echo Usage: update_usb ^<drive letter^>
echo:

:eof
