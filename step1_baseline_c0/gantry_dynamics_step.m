%% GANTRY_DYNAMICS_STEP.M - 双自由度龙门平台动力学单步推演函数 (RK4)
% =========================================================================
% 特性说明:
% 1. 包含平动-偏转惯性耦合项: 偏心附加质量 delta_m 在偏心距 d_load 处产生的耦合
% 2. 严格对应比赛硬件安装符号:
%    - 左电机: 正向 CAN 指令产生向前驱动力 FL =  Kf * iL_cmd
%    - 右电机: 对向安装，负向 CAN 指令产生向前驱动力 FR = -Kf * iR_cmd
% 3. 包含左右导轨非对称摩擦与横梁弹性扭转恢复力矩
% =========================================================================

function x_next = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, dt)
    % 状态向量 x = [yG; alpha; yG_dot; alpha_dot]
    % iL_cmd: 左电机 CAN 控制指令 (count, 向前为正)
    % iR_cmd: 右电机 CAN 控制指令 (count, 对向安装, 实际输出向前为负)
    % delta_m: 右侧偏心附加质量 (kg)
    % d_load: 附加质量偏心距 (m), 取正值表示偏向右侧导轨
    % delta_fric: 右侧摩擦相对左侧的偏差比例
    
    % 四阶龙格-库塔 (RK4)
    k1 = eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric);
    k2 = eval_deriv(x + 0.5 * dt * k1, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric);
    k3 = eval_deriv(x + 0.5 * dt * k2, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric);
    k4 = eval_deriv(x + dt * k3, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric);
    
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end

function dxdt = eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric)
    yG        = x(1);
    alpha     = x(2);
    yG_dot    = x(3);
    alpha_dot = x(4);
    
    Le = mech.Le;
    
    % 左右导轨几何位移与线速度 (以向前为正)
    vL = yG_dot - 0.5 * Le * alpha_dot;
    vR = yG_dot + 0.5 * Le * alpha_dot;
    
    % 实际推进力计算 (严格契合对向安装物理符号)
    FL =  mech.Kf * iL_cmd;
    FR = -mech.Kf * iR_cmd;
    
    % 左右非对称导轨摩擦力 (黏性摩擦 + 库仑摩擦, 使用平滑 tanh 避免数值抖振)
    bL = plant.b_nom;
    fcL = plant.fc_nom;
    bR = plant.b_nom * (1.0 + delta_fric);
    fcR = plant.fc_nom * (1.0 + delta_fric);
    
    F_fric_L = bL * vL + fcL * tanh(100.0 * vL);
    F_fric_R = bR * vR + fcR * tanh(100.0 * vR);
    
    % 惯性矩阵元素 (包含偏心附加质量的惯性耦合)
    M_tot = mech.mG_nom + delta_m;
    J_tot = mech.J_alpha_nom + delta_m * (d_load^2);
    coupling_m = delta_m * d_load; % 平动-偏转惯性耦合项
    
    % 广义推力合力
    F_total = (FL + FR) - (F_fric_L + F_fric_R);
    
    % 广义偏转力矩 (推力差力矩、摩擦差力矩、横梁扭转恢复刚度与阻尼)
    Tau_total = (0.5 * Le) * (FR - FL) - (0.5 * Le) * (F_fric_R - F_fric_L) ...
                - plant.K_alpha * alpha - plant.B_alpha * alpha_dot;
            
    % 2x2 质量矩阵求逆: [M_tot, coupling_m; coupling_m, J_tot] * [yG_ddot; alpha_ddot] = [F_total; Tau_total]
    detM = M_tot * J_tot - (coupling_m^2);
    yG_ddot    = ( J_tot * F_total - coupling_m * Tau_total) / detM;
    alpha_ddot = (-coupling_m * F_total + M_tot * Tau_total) / detM;
    
    dxdt = [yG_dot; alpha_dot; yG_ddot; alpha_ddot];
end
