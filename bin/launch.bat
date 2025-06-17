@echo off
echo Launch DDsEngine Environment
echo Launch Web Server
start "" "./http.exe" 127.0.0.1 80 .
echo Launch Cuda Kernel
start "" "./ddsengine.cuda.exe"
echo Launch Web Server at http:\\localhost
pause
