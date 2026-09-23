%% ACTUATOR_MAP_M3508.M - 双 M3508 龙门执行器空间与广义力空间双向映射模块
% =========================================================================
% 本模块实现：
% 1. 正向映射: [iL, iR] -> [FG, T_alpha] (从电机电流指令到质心推力与偏转力矩)
% 2. 逆向解耦: [FG, T_alpha] -> [iL_ideal, iR_ideal] (从期望广义力到理想电机电流)
% 3. 支持左右驱动推力系数不对称 (Kf_L ≠ Kf_R, 用于工况 F 执行器退化仿真)
% 4. 严格对应硬件安装关系: 左电机正向向前, 右电机对向安装负向向前
% =========================================================================

classdef actuator_map_m3508
    methods (Static)
        %% 1. 前向映射矩阵与逆矩阵计算
        function [T_act, T_act_inv] = get_matrices(Le, Kf_L, Kf_R)
            % Le: 龙门跨距 (m)
            % Kf_L: 左侧电机推力系数 (N / count)
            % Kf_R: 右侧电机推力系数 (N / count)
            
            % 前向矩阵: [FG; T_alpha] = T_act * [iL; iR]
            % FL = Kf_L * iL, FR = -Kf_R * iR
            % FG = FL + FR = Kf_L * iL - Kf_R * iR
            % T_alpha = (Le/2) * (FR - FL) = - (Le*Kf_L/2)*iL - (Le*Kf_R/2)*iR
            T_act = [  Kf_L,              -Kf_R; ...
                      -0.5 * Le * Kf_L,   -0.5 * Le * Kf_R ];
                  
            % 解析逆矩阵: [iL; iR] = T_act_inv * [FG; T_alpha]
            % det(T_act) = -Le * Kf_L * Kf_R
            T_act_inv = [  1.0 / (2.0 * Kf_L),    -1.0 / (Le * Kf_L); ...
                          -1.0 / (2.0 * Kf_R),    -1.0 / (Le * Kf_R) ];
        end
        
        %% 2. 广义控制量 -> 理想电机电流 (逆向解耦分配)
        function [iL_ideal, iR_ideal] = force_to_current(FG, T_alpha, Le, Kf_L, Kf_R)
            iL_ideal =  FG / (2.0 * Kf_L) - T_alpha / (Le * Kf_L);
            iR_ideal = -(FG / (2.0 * Kf_R) + T_alpha / (Le * Kf_R));
        end
        
        %% 3. 电机电流 -> 广义力与力矩 (前向投影)
        function [FG, T_alpha] = current_to_force(iL, iR, Le, Kf_L, Kf_R)
            FL =  Kf_L * iL;
            FR = -Kf_R * iR;
            FG      = FL + FR;
            T_alpha = 0.5 * Le * (FR - FL);
        end
    end
end
