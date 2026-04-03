@echo off
setlocal

:: === pyocd: 优先 PATH，找不到给提示 ===
where pyocd >nul 2>&1
if %ERRORLEVEL%==0 (
    set PYOCD=pyocd
) else (
    echo [ERROR] pyocd not found in PATH
    echo         Install: pip install pyocd
    echo         Then:    pyocd pack install Renesas.RA
    pause
    exit /b 1
)

set ELF_PATH=%~dp0Objects\FSP_Project.axf
set TARGET=R7FA6M5BF

if not exist "%ELF_PATH%" (
    echo [ERROR] ELF not found: %ELF_PATH%
    echo         Build the project first.
    pause
    exit /b 1
)

echo Flashing FSP_Project...
echo Target: %TARGET%

"%PYOCD%" flash --target %TARGET% --frequency 100000 --erase sector "%ELF_PATH%"
if %ERRORLEVEL%==0 (echo Flash OK) else (echo Flash FAILED)
pause
