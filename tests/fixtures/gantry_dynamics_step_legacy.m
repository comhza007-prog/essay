%% GANTRY_DYNAMICS_STEP_LEGACY.M - 冻结的重构前 Step 2 原始 RK4 单步推演参考实现
% =========================================================================
% Frozen reference fixture.
% Source commit: 7a41488
% Do not modify.
% Original file: step2_advanced_controllers/gantry_dynamics_step.m
% =========================================================================

function x_next = gantry_dynamics_step_legacy(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R)
    if nargin < 10 || isempty(Kf_L)
        Kf_L = mech.Kf;
    end
    if nargin < 11 || isempty(Kf_R)
        Kf_R = mech.Kf;
    end

    % 四阶龙格-库塔 (RK4)
    k1 = gantry_dynamics_deriv_legacy(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k2 = gantry_dynamics_deriv_legacy(x + 0.5 * dt * k1, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k3 = gantry_dynamics_deriv_legacy(x + 0.5 * dt * k2, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k4 = gantry_dynamics_deriv_legacy(x + dt * k3, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end
