@echo off
chcp 65001 >nul
title 第二阶段初步对比仿真 (C0 vs C1 vs C2a)
echo ============================================================
echo   正在运行第二阶段初步对比测试 (C0 vs C1 vs C2a)...
echo ============================================================
echo.

"d:\Polyspace\R2020a\bin\matlab.exe" -batch "cd('C:\Users\Lenovo\Desktop\论文\早期\论文\起重机\output\step2_advanced_controllers'); run('run_c0_c1_c2a_benchmark.m');"

echo.
echo ============================================================
echo   计算完成！正在打开三算法横向对比图...
echo ============================================================
start "" "C:\Users\Lenovo\Desktop\论文\早期\论文\起重机\output\step2_advanced_controllers\benchmark_c0_c1_c2a_Imax4500.png"
echo.
pause
