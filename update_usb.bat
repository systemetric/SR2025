@echo off
if "%~1"=="" goto :usage

if not exist "%~1\myrobot\" mkdir "%~1:\myrobot" > NUL

copy *.* %~1:\ > NUL

echo Success.
goto :eof

:usage 
echo Usage: update_usb ^<drive letter^>
echo:

:eof
