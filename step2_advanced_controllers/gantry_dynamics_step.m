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

    % 四阶龙格-库塔 (RK4)
    k1 = eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k2 = eval_deriv(x + 0.5 * dt * k1, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k3 = eval_deriv(x + 0.5 * dt * k2, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k4 = eval_deriv(x + dt * k3, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end

function dxdt = eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R)
    yG        = x(1);
    alpha     = x(2);
    yG_dot    = x(3);
    alpha_dot = x(4);
    
    Le = mech.Le;
    
    % 左右物理导轨线位移与线速度 (向前为正)
    vL = yG_dot - 0.5 * Le * alpha_dot;
    vR = yG_dot + 0.5 * Le * alpha_dot;
    
    % 实际推进力计算 (支持非对称推力增益与对向安装)
    FL =  Kf_L * iL_cmd;
    FR = -Kf_R * iR_cmd;
    
    % 左右非对称摩擦阻力
    bL = plant.b_nom;
    fcL = plant.fc_nom;
    bR = plant.b_nom * (1.0 + delta_fric);
    fcR = plant.fc_nom * (1.0 + delta_fric);
    
    F_fric_L = bL * vL + fcL * tanh(100.0 * vL);
    F_fric_R = bR * vR + fcR * tanh(100.0 * vR);
    
    % 惯性矩阵与平动-偏转惯性耦合
    M_tot = mech.mG_nom + delta_m;
    J_tot = mech.J_alpha_nom + delta_m * (d_load^2);
    coupling_m = delta_m * d_load;
    
    % 广义合外力
    F_total = (FL + FR) - (F_fric_L + F_fric_R);
    
    % 广义合外力矩
    Tau_total = (0.5 * Le) * (FR - FL) - (0.5 * Le) * (F_fric_R - F_fric_L) ...
                - plant.K_alpha * alpha - plant.B_alpha * alpha_dot;
            
    % 2x2 质量矩阵求逆解析解
    detM = M_tot * J_tot - (coupling_m^2);
    yG_ddot    = ( J_tot * F_total - coupling_m * Tau_total) / detM;
    alpha_ddot = (-coupling_m * F_total + M_tot * Tau_total) / detM;
    
    dxdt = [yG_dot; alpha_dot; yG_ddot; alpha_ddot];
end
