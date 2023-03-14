@echo off
setlocal EnableExtensions DisableDelayedExpansion
pushd

cd %~dp0

python _build.py %*

if NOT %errorlevel% == 0 (
	goto :fail
)

popd
goto :EOF

:fail
echo ** FAILED **
@pause
popd

