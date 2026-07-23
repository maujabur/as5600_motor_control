@echo off
setlocal

set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" (
  echo Visual C++ environment not found: %VCVARS%
  exit /b 2
)

call "%VCVARS%" >nul
if errorlevel 1 exit /b %errorlevel%

if not exist ".pio\msvc-tests" mkdir ".pio\msvc-tests"

cl /nologo /std:c++17 /EHsc /Ilib\domain ^
  tests\cpp\test_angle_math.cpp lib\domain\AngleMath.cpp ^
  /Fe:.pio\msvc-tests\test_angle_math.exe /Fo:.pio\msvc-tests\
if errorlevel 1 exit /b %errorlevel%

.pio\msvc-tests\test_angle_math.exe
exit /b %errorlevel%
