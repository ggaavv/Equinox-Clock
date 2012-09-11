set PROJECT_DIR="C:\Users\Jamie\Dropbox\workspace\Equinox-Clock\Equinox"

cd %PROJECT_DIR%
make

::FOR %A IN ( A B C D E F G H ) DO set echo %A
FOR %%A IN ( D E F G H I J ) DO (
	if EXIST %%A:\firmware.bin (
		set BOARD_DIR=%%A
		goto file_op
	)
	if EXIST %%A:\project.bin (
		set BOARD_DIR=%%A
		goto file_op
	)
)
goto end


:file_op
del %BOARD_DIR%:\firmware.bin
del %BOARD_DIR%:\project.bin
copy C:\Users\Jamie\Dropbox\workspace\Equinox-Clock\Equinox\FLASH_RUN\project.bin %BOARD_DIR%:\
:end
pause