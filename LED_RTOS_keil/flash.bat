@echo off
set PYOCD=C:\Users\21181\AppData\Local\Programs\Python\Python310\Scripts\pyocd.exe
set ELF_PATH=%~dp0Objects\FSP_Project.axf
set UID=0001A0000001

echo Flashing FSP_Project...
"%PYOCD%" flash --target R7FA6M5BF --frequency 100000 --uid %UID% --erase sector "%ELF_PATH%"
if %ERRORLEVEL%==0 (echo Flash OK) else (echo Flash FAILED)
pause
