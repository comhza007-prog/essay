@echo off
chcp 65001 >nul
title 双M3508龙门基线仿真 (Baseline C0)
echo ============================================================
echo   正在调用本地 MATLAB 执行双 M3508 龙门基线仿真 (Baseline C0)...
echo ============================================================
echo.

"d:\Polyspace\R2020a\bin\matlab.exe" -batch "cd('C:\Users\Lenovo\Desktop\论文\早期\论文\起重机\output\step1_baseline_c0'); run_c0_benchmark;"

echo.
echo ============================================================
echo   仿真计算完成！指标表已打印在上，正在打开四合一结果对比图...
echo ============================================================
start "" "C:\Users\Lenovo\Desktop\论文\早期\论文\起重机\output\step1_baseline_c0\baseline_c0_comparison.png"
echo.
pause
