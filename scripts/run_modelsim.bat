@echo off
REM ============================================================================
REM run_modelsim.bat
REM One-click simulation regression script for PS-02 Hardware PID Controller
REM ============================================================================

set SCRIPT_DIR=%~dp0
set PROJ_DIR=%SCRIPT_DIR%..
cd /d "%PROJ_DIR%"

echo ============================================================================
echo   Running PS-02 Hardware PID Controller Regression Suite
echo ============================================================================

python "%SCRIPT_DIR%run_regression.py"

if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Regression failed!
    exit /b %ERRORLEVEL%
)

echo [SUCCESS] Regression completed successfully!
pause
