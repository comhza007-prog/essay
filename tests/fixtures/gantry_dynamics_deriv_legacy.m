%% GANTRY_DYNAMICS_DERIV_LEGACY.M - 冻结的重构前 Step 2 原始微分函数参考实现
% =========================================================================
% Frozen pre-refactor reference.
% Source commit: 7a41488 (fix(step3a): eliminate silent hardcoded fallback in controller_c3a_rls_robust)
% Original file: step2_advanced_controllers/gantry_dynamics_step.m (eval_deriv)
% Do not modify during equivalence validation.
% =========================================================================

function [dxdt, details] = gantry_dynamics_deriv_legacy(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R)
    if nargin < 10 || isempty(Kf_R)
        Kf_R = mech.Kf;
    end
    if nargin < 9 || isempty(Kf_L)
        Kf_L = mech.Kf;
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
    
    if nargout > 1
        details = struct();
        details.vL = vL;
        details.vR = vR;
        details.FL = FL;
        details.FR = FR;
        details.F_fric_L = F_fric_L;
        details.F_fric_R = F_fric_R;
        details.F_total = F_total;
        details.Tau_total = Tau_total;
        details.yG_ddot = yG_ddot;
        details.alpha_ddot = alpha_ddot;
    end
end
