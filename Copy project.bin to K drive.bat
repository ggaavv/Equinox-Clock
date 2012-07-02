
cd %HOMEDRIVE%%HOMEPATH%\git\Equinox-Clock\Equinox
make
del K:\firmware.bin
copy %HOMEDRIVE%%HOMEPATH%\git\Equinox-Clock\Equinox\FLASH_RUN\project.bin K:\
pause