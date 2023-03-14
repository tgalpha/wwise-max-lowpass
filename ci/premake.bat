@echo off
setlocal EnableExtensions DisableDelayedExpansion
pushd

cd %~dp0\..

python "%WWISEROOT%/Scripts/Build/Plugins/wp.py" premake Authoring
