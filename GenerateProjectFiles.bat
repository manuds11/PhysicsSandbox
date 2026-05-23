@echo off

echo =========================================
echo Generating Unreal Visual Studio Files...
echo =========================================

"C:\Program Files\Epic Games\UE_5.6\Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.exe" ^
-projectfiles ^
-project="%~dp0PhysicsSandbox.uproject" ^
-game

echo.
echo Done.
pause