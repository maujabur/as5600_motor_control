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

set "TARGET=%~1"
if "%TARGET%"=="" set "TARGET=all"

if "%TARGET%"=="angle_math" goto angle_math
if "%TARGET%"=="motion_sequence" goto motion_sequence
if "%TARGET%"=="bounded_integer" goto bounded_integer
if "%TARGET%"=="all" goto angle_math
echo Unknown C++ test target: %TARGET%
exit /b 2

:angle_math
cl /nologo /std:c++17 /EHsc /Ilib\domain ^
  tests\cpp\test_angle_math.cpp lib\domain\AngleMath.cpp ^
  /Fe:.pio\msvc-tests\test_angle_math.exe /Fo:.pio\msvc-tests\
if errorlevel 1 exit /b %errorlevel%
.pio\msvc-tests\test_angle_math.exe
if errorlevel 1 exit /b %errorlevel%
if not "%TARGET%"=="all" exit /b 0

:motion_sequence
cl /nologo /std:c++17 /EHsc /Ilib\domain /Ilib\sequence ^
  tests\cpp\test_motion_sequence.cpp ^
  lib\sequence\MotionSequenceController.cpp lib\domain\AngleMath.cpp ^
  /Fe:.pio\msvc-tests\test_motion_sequence.exe /Fo:.pio\msvc-tests\
if errorlevel 1 exit /b %errorlevel%
.pio\msvc-tests\test_motion_sequence.exe
if errorlevel 1 exit /b %errorlevel%
if not "%TARGET%"=="all" exit /b 0

:bounded_integer
cl /nologo /std:c++17 /EHsc /Ilib\web ^
  tests\cpp\test_bounded_integer.cpp lib\web\BoundedInteger.cpp ^
  /Fe:.pio\msvc-tests\test_bounded_integer.exe /Fo:.pio\msvc-tests\
if errorlevel 1 exit /b %errorlevel%
.pio\msvc-tests\test_bounded_integer.exe
exit /b %errorlevel%
