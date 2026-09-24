%% GANTRY_DYNAMICS_STEP_RK4.M - 统一公共双自由度龙门动力学单步推演函数 (经典 RK4)
% =========================================================================
% 项目权威公共单步积分器 (Step 3C 及后续阶段唯一公共推演入口)
%
% 输入:
%   x          : 当前状态向量 [yG; alpha; yG_dot; alpha_dot] (4x1 double)
%   iL_applied : 左侧执行器实际施加电流 [counts] (已考虑限幅与执行时滞)
%   iR_applied : 右侧执行器实际施加电流 [counts] (已考虑限幅与执行时滞)
%   mech       : 机械结构参数结构体 (mG_nom, J_alpha_nom, Le, Kf 等)
%   plant      : 系统动力学参数结构体 (b_nom, fc_nom, K_alpha, B_alpha 等)
%   delta_m    : 附加负载质量 [kg] (偏载工况下必须为非零载荷质量)
%   d_load     : 负载质心横向偏移距离 [m] (向右为正)
%   delta_fric : 导轨摩擦非对称系数 (标称 0.0)
%   dt         : 离散仿真时间步长 [s] (通常为 0.001 s)
%   Kf_L, Kf_R : 左右侧执行器推力系数 [N/count] (标称均为 mech.Kf)
%
% 输出:
%   x_next     : 下一时刻状态向量 (4x1 double)
%   details    : 中间物理诊断量结构体 (合推力、合摩擦力、偏航力矩等)
% =========================================================================

function [x_next, details] = gantry_dynamics_step_rk4(x, iL_applied, iR_applied, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R)
    % 参数缺省与容错处理
    if nargin < 10 || isempty(Kf_L)
        Kf_L = mech.Kf;
    end
    if nargin < 11 || isempty(Kf_R)
        Kf_R = mech.Kf;
    end
    if nargin < 8 || isempty(delta_fric)
        delta_fric = 0.0;
    end
    if nargin < 7 || isempty(d_load)
        d_load = 0.0;
    end
    if nargin < 6 || isempty(delta_m)
        delta_m = 0.0;
    end
    if nargin < 9 || isempty(dt)
        dt = 0.001;
    end

    % 经典四阶龙格-库塔 (RK4) 数值积分
    % 显式、严格调用 common/ 目录下的唯一微分方程核 gantry_dynamics_deriv
    [k1, d1] = gantry_dynamics_deriv(x, iL_applied, iR_applied, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k2, ~]  = gantry_dynamics_deriv(x + 0.5 * dt * k1, iL_applied, iR_applied, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k3, ~]  = gantry_dynamics_deriv(x + 0.5 * dt * k2, iL_applied, iR_applied, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k4, ~]  = gantry_dynamics_deriv(x + dt * k3, iL_applied, iR_applied, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);

    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    
    if nargout > 1
        details = d1;
    end
end
