
set MATLAB=C:\Program Files\MATLAB\R2020b

cd .

if "%1"=="" ("C:\PROGRA~1\MATLAB\R2020b\bin\win64\gmake"  -f demanded_torque_calculator.mk all) else ("C:\PROGRA~1\MATLAB\R2020b\bin\win64\gmake"  -f demanded_torque_calculator.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1