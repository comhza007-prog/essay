@echo off
chcp 65001 >nul
title 第一阶段 C0 基线模型自动化自检
echo ============================================================
echo   正在调用 MATLAB 运行第一阶段 4 项关键测试自检...
echo ============================================================
echo.

"d:\Polyspace\R2020a\bin\matlab.exe" -batch "cd('C:\Users\Lenovo\Desktop\论文\早期\论文\起重机\output\step1_baseline_c0'); verify_step1;"

echo.
pause
