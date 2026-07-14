@echo off
REM ============================================================
REM  Build script for ESP32-S3 Gateway PC Simulation
REM  Requires: CMake 3.16+, MinGW-w64 (or compatible GCC)
REM ============================================================

echo.
echo === ESP32-S3 Gateway Simulation Build ===
echo.

if not exist build mkdir build
cd build

REM Configure with MinGW Makefiles generator
cmake .. -G "MinGW Makefiles"
if errorlevel 1 (
    echo.
    echo [ERROR] CMake configuration failed.
    echo Make sure CMake and MinGW are installed and in PATH.
    cd ..
    exit /b 1
)

echo.
echo === Compiling... ===
echo.

cmake --build .
if errorlevel 1 (
    echo.
    echo [ERROR] Build failed. Check compiler output above.
    cd ..
    exit /b 1
)

echo.
echo ============================================================
echo  Build successful!
echo  Run:  build\gateway_sim.exe
echo ============================================================
echo.

cd ..
