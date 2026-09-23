%% GANTRY_DYNAMICS_STEP.M - 双自由度龙门动力学单步积分推演 (支持非对称驱动推力系数)
% =========================================================================
% 本函数支持：
% 1. 左右非对称推力增益: Kf_L, Kf_R (用于工况 F 执行器退化仿真)
% 2. 严格对向安装符号: FL = Kf_L * iL_cmd, FR = -Kf_R * iR_cmd
% 3. 偏心附加质量 delta_m 在偏心距 d_load 处的平动-偏转惯性耦合
% 4. 左右非对称导轨摩擦 (库仑静摩擦 + 黏性阻尼) 与横梁扭转弹性恢复
% =========================================================================

function x_next = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R)
    % 如果未显式传入 Kf_L 和 Kf_R，默认沿用标称 Kf
    if nargin < 10 || isempty(Kf_L)
        Kf_L = mech.Kf;
    end
    if nargin < 11 || isempty(Kf_R)
        Kf_R = mech.Kf;
    end

    % 四阶龙格-库塔 (RK4) - 统一调用公共底层动力学微分核 gantry_dynamics_deriv
    [k1, ~] = gantry_dynamics_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k2, ~] = gantry_dynamics_deriv(x + 0.5 * dt * k1, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k3, ~] = gantry_dynamics_deriv(x + 0.5 * dt * k2, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    [k4, ~] = gantry_dynamics_deriv(x + dt * k3, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end

function dxdt = eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R)
    % 兼容性转发至公共动力学微分核
    dxdt = gantry_dynamics_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
end
