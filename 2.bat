copy build\tinySA4.bin C:\tinySA_detector\firmware\athome.kaashoek.com\tinySA4\DFU\tinySA.bin 
if errorlevel 1 goto:eof
pushd C:\tinySA_detector\firmware\athome.kaashoek.com\tinySA4\DFU\
call DFU_LOAD_BIN.bat

