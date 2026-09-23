%% GANTRY_DYNAMICS_STEP_STEP3B.M - 双自由度龙门动力学单步推演函数 (RK4)
% =========================================================================
% 特性说明:
% 1. 严格调用统一微分公开函数 gantry_dynamics_deriv_step3b，杜绝模型复写分歧
% 2. 采用经典四阶龙格-库塔法 (RK4) 保证高精度积分
% 3. 支持导出单步微分中间量 details
% =========================================================================

function [x_next, details] = gantry_dynamics_step_step3b(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R)
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

    % 四阶龙格-库塔 (RK4) - 统一调用公共底层动力学微分核 gantry_dynamics_deriv
    [k1, d1] = gantry_dynamics_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k2, ~]  = gantry_dynamics_deriv(x + 0.5 * dt * k1, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k3, ~]  = gantry_dynamics_deriv(x + 0.5 * dt * k2, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k4, ~]  = gantry_dynamics_deriv(x + dt * k3, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    if nargout > 1
        details = d1;
    end
end
