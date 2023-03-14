# wwise-max-lowpass
wwise lowpass filter powered by Max&amp;RNBO


## Build
> on windows
1. Install Wwise Authoring and SDK, add WWISEROOT, WWISESDK env variable from wwise launcher (tested in 2021.1.3)
2. Install Python3 and add pacakge `kkpyutil`
3. Run
```
ci\premake.bat
ci\build.bat -c Release
```
Plugin binaries will be copied to wwise authoring tool plugin folder.
- %wwiseroot%\Authoring\x64\Release\bin\Plugins