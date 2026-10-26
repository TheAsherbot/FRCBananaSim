@echo off
cd /d "%~dp0"

echo ========================================================
echo        FRCBananaSim - Build and Test Runner
echo ========================================================
echo.

if not exist build\CMakeCache.txt (
    echo [1/3] Generating CMake build files...
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
    if errorlevel 1 goto :failed
)

echo [2/3] Building FRCBananaSim_Tests in Release mode...
cmake --build build --config Release
if errorlevel 1 goto :failed

echo [3/3] Running test suite...
echo --------------------------------------------------------
ctest --test-dir build -C Release --output-on-failure
if errorlevel 1 goto :failed

echo --------------------------------------------------------
echo [SUCCESS] All tests compiled and passed successfully!
echo ========================================================
goto :end

:failed
echo ========================================================
echo [FAILURE] Build or tests encountered errors.
echo ========================================================

:end
echo.
pause