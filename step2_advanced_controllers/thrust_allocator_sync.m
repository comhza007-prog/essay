%% THRUST_ALLOCATOR_SYNC.M - 推力空间同步优先约束分配器 (闭式解析解)
% =========================================================================
% 本模块在二维物理推力空间 [FL, FR] 中实现同步优先约束分配：
% 1. 严格使用软硬件有效限幅交集:
%      I_eff_L = min(I_fw_L, Imax_L),  I_eff_R = min(I_fw_R, Imax_R)
%      FL_max  = Kf_L * I_eff_L,       FR_max  = Kf_R * I_eff_R
% 2. 物理可行域为闭矩形: U = [-FL_max, FL_max] x [-FR_max, FR_max]
% 3. 优先级分配原则:
%    - 第一优先级: 在可行域允许的最大力矩跨度内，完整保留偏转纠偏力矩 Delta_F；
%    - 第二优先级: 在保证已分配力矩前提下，最大化实现推进合力 FG；
% 4. 全局闭式解析投影解，无在线数值优化迭代开销；
% 5. 严格支持左右驱动推力系数不对称 (Kf_L ≠ Kf_R)。
% =========================================================================

function [i_actual, info] = thrust_allocator_sync( ...
    FG_des, Talpha_des, Le, Kf_L, Kf_R, Imax, I_fw_limit)

    % 1. 解算双极有效电流限幅 I_eff
    if nargin < 7 || isempty(I_fw_limit)
        I_fw_limit = 16000.0;
    end
    
    if isscalar(Imax)
        Imax_L = Imax; Imax_R = Imax;
    else
        Imax_L = Imax(1); Imax_R = Imax(2);
    end
    
    if isscalar(I_fw_limit)
        Ifw_L = I_fw_limit; Ifw_R = I_fw_limit;
    else
        Ifw_L = I_fw_limit(1); Ifw_R = I_fw_limit(2);
    end
    
    I_eff_L = min(Ifw_L, Imax_L);
    I_eff_R = min(Ifw_R, Imax_R);
    
    % 2. 物理推力上下边界 (定义向前为正)
    FL_max = Kf_L * I_eff_L;
    FR_max = Kf_R * I_eff_R;
    
    % 3. 期望物理推力与推力差解算
    % Talpha = 0.5 * Le * (FR - FL) ==> FR - FL = 2 * Talpha / Le
    % FG = FL + FR
    DeltaF_des = (2.0 * Talpha_des) / Le;
    
    % 4. 第一优先级: 纠偏力矩优先分配
    % 可行域内左右推力的最大可能跨度为 DeltaF_max = FR_max + FL_max
    DeltaF_max = FR_max + FL_max;
    DeltaF_alloc = max(-DeltaF_max, min(DeltaF_max, DeltaF_des));
    
    % 5. 第二优先级: 推进合力投影
    % 给定 FR = FL + DeltaF_alloc，且满足 -FR_max <= FR <= FR_max
    % 可求得 FL 的精确可行区间 [FL_lower, FL_upper]
    FL_lower = max(-FL_max, -FR_max - DeltaF_alloc);
    FL_upper = min( FL_max,  FR_max - DeltaF_alloc);
    
    % 期望的 FL 目标标量
    FL_target = 0.5 * FG_des - 0.5 * DeltaF_alloc;
    
    % 投影到允许区间
    FL_alloc = max(FL_lower, min(FL_upper, FL_target));
    FR_alloc = FL_alloc + DeltaF_alloc;
    
    % 6. 逆映射到电机电流指令与最终保护限幅
    % FL = Kf_L * iL, FR = -Kf_R * iR
    iL_raw = FL_alloc / Kf_L;
    iR_raw = -FR_alloc / Kf_R;
    
    iL_act = max(-I_eff_L, min(I_eff_L, iL_raw));
    iR_act = max(-I_eff_R, min(I_eff_R, iR_raw));
    i_actual = [iL_act; iR_act];
    
    % 7. 诊断信息解算
    FL_real =  Kf_L * iL_act;
    FR_real = -Kf_R * iR_act;
    FG_alloc = FL_real + FR_real;
    Talpha_alloc = 0.5 * Le * (FR_real - FL_real);
    
    tol = 1e-6;
    is_sat_torque = abs(Talpha_alloc - Talpha_des) > tol;
    is_sat_thrust = abs(FG_alloc - FG_des) > tol;
    
    info.i_raw = [iL_raw; iR_raw];
    info.i_actual = i_actual;
    info.FL_alloc = FL_real;
    info.FR_alloc = FR_real;
    info.FG_alloc = FG_alloc;
    info.Talpha_alloc = Talpha_alloc;
    info.Delta_Talpha = Talpha_alloc - Talpha_des;
    info.Delta_FG = FG_alloc - FG_des;
    info.is_sat_torque = is_sat_torque;
    info.is_sat_thrust = is_sat_thrust;
    info.is_sat_any = is_sat_torque || is_sat_thrust;
    info.I_eff = [I_eff_L; I_eff_R];
    info.F_bounds = [-FL_max, FL_max; -FR_max, FR_max];
end
