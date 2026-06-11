@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "DKIT_WORK_DIR=D:\dtf\plugins\dkit"
set "PYTHON_CMD="

py -3 -V >nul 2>&1
if not errorlevel 1 set "PYTHON_CMD=py -3"

if not defined PYTHON_CMD (
    python -V >nul 2>&1
    if not errorlevel 1 set "PYTHON_CMD=python"
)

if not defined PYTHON_CMD (
    echo [ERROR] Python 3 was not found in PATH.
    exit /b 1
)

if not exist "%DKIT_WORK_DIR%" mkdir "%DKIT_WORK_DIR%" >nul 2>nul
if not exist "%DKIT_WORK_DIR%" (
    echo [ERROR] Failed to access work directory: %DKIT_WORK_DIR%
    exit /b 1
)

pushd "%DKIT_WORK_DIR%" || exit /b 1
%PYTHON_CMD% "%SCRIPT_DIR%run_dkit_cmd.py" %*
set "EXIT_CODE=%ERRORLEVEL%"
popd

exit /b %EXIT_CODE%
