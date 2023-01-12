rem C:\cygwin64\bin\
path="C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\12.2 mpacbti-bet1\bin\";C:\cygwin64\bin\;%path%
REM sh file /usr/bin/make.exe
REM sh make.exe
set TARGET=F303
rem C:\cygwin64\bin\mintty.exe -e make clean
del build\tinySA4.bin
C:\cygwin64\bin\mintty.exe --hold always -e make 

