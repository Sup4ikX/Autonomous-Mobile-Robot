@echo off
setlocal enabledelayedexpansion

:: Проверяем, передан ли параметр (расширение файлов)
if "%1"=="" (
    echo Usage: %~nx0 extension
    echo Example: %~nx0 txt
    pause
    exit /b 1
)

:: Убираем возможную точку в начале расширения
set "ext=%1"
if "%ext:~0,1%"=="." set "ext=%ext:~1%"

:: Проверяем, есть ли файлы с таким расширением в текущей папке
dir *.%ext% >nul 2>&1
if errorlevel 1 (
    echo No files with extension .%ext% found.
    pause
    exit /b 1
)

:: Получаем текущую дату и время в формате YYYYMMDDHHMMSS через wmic
for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value') do set "datetime=%%I"

:: Извлекаем нужные компоненты
set "YYYY=%datetime:~0,4%"
set "MM=%datetime:~4,2%"
set "DD=%datetime:~6,2%"
set "HH=%datetime:~8,2%"
set "Min=%datetime:~10,2%"

:: Формируем суффикс: DDMMYYYY_HH_MM
set "suffix=%DD%%MM%%YYYY%_%HH%_%Min%"

:: Имя архива
set "arcname=archive_%suffix%.rar"

:: Создаём RAR-архив с помощью rar.exe (консольная версия WinRAR)
rar a -r "%arcname%" *.%ext%
if errorlevel 1 (
    echo Failed to create archive. Make sure rar.exe is available in PATH.
    pause
    exit /b 1
)

echo Archive created: %arcname%
pause
exit /b 0