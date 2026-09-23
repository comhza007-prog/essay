%% GANTRY_DYNAMICS_DERIV_STEP3B.M - 双自由度龙门动力学微分方程公开计算函数
% =========================================================================
% 特性说明:
% 1. 作为 Step 3B 动力学单步推演与测试检验的统一底层微分计算核，避免“测试复写模型”风险
% 2. 严格契合硬件实际对向安装几何关系:
%    - 左电机: FL =  Kf_L * iL_cmd (向前为正)
%    - 右电机: FR = -Kf_R * iR_cmd (对向安装，负指令向前为正)
% 3. 包含平动-偏转惯性耦合项 (coupling_m = delta_m * d_load)
% 4. 包含左右非对称导轨摩擦力与横梁扭转弹性恢复/阻尼
% =========================================================================

function [dxdt, details] = gantry_dynamics_deriv_step3b(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R)
    % 默认推力系数处理
    if nargin < 9 || isempty(Kf_L)
        Kf_L = mech.Kf;
    end
    if nargin < 10 || isempty(Kf_R)
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
    
    % 左右导轨摩擦阻力 (黏性 + 库仑)
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
        details.T_motor_alpha = 0.5 * Le * (FR - FL);
        details.T_fric_alpha  = 0.5 * Le * (F_fric_R - F_fric_L);
        details.yG_ddot = yG_ddot;
        details.alpha_ddot = alpha_ddot;
        details.coupling_m = coupling_m;
        details.M_tot = M_tot;
        details.J_tot = J_tot;
    end
end
