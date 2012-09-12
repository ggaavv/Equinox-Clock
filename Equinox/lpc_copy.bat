set PROJECT_DIR="C:\Users\Jamie\Dropbox\workspace\Equinox-Clock\Equinox"
::set PROJECT_DIR="D:\Documents and Settings\xxx\workspace\Equinox-Clock\Equinox"
::set PROJECT_DIR="C:\Documents and Settings\Jamie\My Documents\Dropbox\workspace\Equinox-Clock\Equinox"

cd %PROJECT_DIR%
cs-make

@ECHO off

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

@ECHO on

copy %PROJECT_DIR%\FLASH_RUN\project.bin %BOARD_DIR%:\
:end
pause