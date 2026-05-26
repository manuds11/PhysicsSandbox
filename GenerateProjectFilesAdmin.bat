@echo off

:: Auto-elevate if not admin
net session >nul 2>&1
if %errorLevel% neq 0 (
    powershell -Command "Start-Process '%~f0' -Verb RunAs"
    exit /b
)

title Unreal Generate Project Files

cd /d C:\dev\projects\unreal\PhysicsSandbox

echo.
echo ============================================
echo Generating Unreal Project Files...
echo ============================================
echo.

"C:\Program Files\Epic Games\UE_5.6\Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.exe" -projectfiles -project="C:\dev\projects\unreal\PhysicsSandbox\PhysicsSandbox.uproject" -game

echo.
echo ============================================
echo Done.
echo ============================================
echo.

pause