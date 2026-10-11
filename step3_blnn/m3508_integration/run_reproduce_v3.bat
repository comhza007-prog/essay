@echo off
setlocal
cd /d "%~dp0"
set "BLNN_MATLAB=D:\Program Files\MATLAB\R2024b\bin\matlab.exe"
if not exist "%BLNN_MATLAB%" (
  echo MATLAB R2024b not found. Open this folder in MATLAB and run reproduce_v3.
  pause
  exit /b 1
)
"%BLNN_MATLAB%" -batch "resultRoot=reproduce_v3();"
if errorlevel 1 (
  echo Reproduction failed. Existing archived data were not overwritten.
  pause
  exit /b 1
)
echo Reproduction completed in a new reproduction_v3 folder.
pause
