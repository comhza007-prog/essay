%% CONTROLLER_C3A_RLS_ROBUST.M - 复合机械参数自适应前馈连续鲁棒同步控制器 (C3a)
% =========================================================================
% 功能：
% 1. 在 C2a 误差面鲁棒滑模控制基础上，引入 Step 3A 机械参数在线自适应回路
% 2. 状态变量滤波器 (SVF) 滤除高频微分噪声并构造因果回归方程
% 3. 递推最小二乘 (RLS) 在线估计 [M_tot, bG, fcG]
% 4. 自适应估计值经滑动窗 PE 门控、紧凑凸集投影、变化率限制与低通平滑后，
%    实时注入前馈补偿项 va:
%    M_hat(1,1) = M_tot_hat, B_hat(1,1) = bG_hat, A_hat(1,1) = fcG_hat
% 5. 保持与 C2a/C2b 完全一致的执行器三级硬限幅与残差输出结构
% =========================================================================

function [iL_cmd, iR_cmd, state_c3a, info] = controller_c3a_rls_robust( ...
    q, qdot, qd, qdot_d, qddot_d, ctrl_c2, Le, Kf_L, Kf_R, Imax, state_c3a, dt, yG_meas)

    if nargin < 12 || isempty(dt)
        dt = 0.001;
    end
    
    % 初始化内部滤波器与估计器状态
    if isempty(state_c3a) || ~isfield(state_c3a, 'is_initialized') || ~state_c3a.is_initialized
        state_c3a.filter = rls_filter_svf(10.0, dt);
        
        % 默认以标称值作为初态: M_nom=13.1kg, bG=70.0, fcG=16.0
        theta_init = [ctrl_c2.M_hat(1,1); ctrl_c2.B_hat(1,1); ctrl_c2.A_hat(1,1)];
        state_c3a.estimator = rls_estimator_mech(theta_init, 0.995, 1.0e-4, dt);
        state_c3a.theta_curr = theta_init;
        state_c3a.is_initialized = true;
    end
    
    % 1. 获取当前平滑参数估计值并动态更新名义模型矩阵
    theta_est = state_c3a.theta_curr;
    M_hat_adapt = ctrl_c2.M_hat;
    B_hat_adapt = ctrl_c2.B_hat;
    A_hat_adapt = ctrl_c2.A_hat;
    
    M_hat_adapt(1, 1) = theta_est(1);
    B_hat_adapt(1, 1) = theta_est(2);
    A_hat_adapt(1, 1) = theta_est(3);
    
    % 2. 误差与广义滑动面计算
    e = q - qd;
    edot = qdot - qdot_d;
    sigma = edot + ctrl_c2.Gamma * e;
    
    % 3. 自适应名义动力学前馈补偿 va
    if isfield(ctrl_c2, 'enable_ff') && ~ctrl_c2.enable_ff
        va = [0.0; 0.0];
    else
        Sf = [tanh(qdot(1) / ctrl_c2.eps_v); tanh(qdot(2) / ctrl_c2.eps_alpha)];
        va = B_hat_adapt * qdot + ctrl_c2.K_hat * q + A_hat_adapt * Sf + M_hat_adapt * qddot_d;
    end
    
    % 4. 连续 tanh 鲁棒反馈补偿 vr
    tanh_sigma = tanh(sigma ./ ctrl_c2.phi);
    vr = -ctrl_c2.K1 * sigma - ctrl_c2.K2 * tanh_sigma - M_hat_adapt * ctrl_c2.Gamma * edot;
    
    % 5. 综合期望广义力 v_beta
    v_beta = va + vr;
    FG_des = v_beta(1);
    Talpha_des = v_beta(2);
    
    % 6. 逆向推力映射
    [iL_req, iR_req] = actuator_map_m3508.force_to_current( ...
        FG_des, Talpha_des, Le, Kf_L, Kf_R);
    i_request = [iL_req; iR_req];
    
    % 7. 执行器多级限幅
    I_fw_limit = 16000.0;
    if isfield(ctrl_c2, 'I_fw_limit')
        I_fw_limit = ctrl_c2.I_fw_limit;
    end
    iL_fw = max(-I_fw_limit, min(I_fw_limit, i_request(1)));
    iR_fw = max(-I_fw_limit, min(I_fw_limit, i_request(2)));
    
    iL_cmd = max(-Imax, min(Imax, iL_fw));
    iR_cmd = max(-Imax, min(Imax, iR_fw));
    i_actual = [iL_cmd; iR_cmd];
    
    % 残差向量
    Delta_i_internal = [iL_fw; iR_fw] - i_request;
    Delta_i_external = i_actual - [iL_fw; iR_fw];
    Delta_i_total    = i_actual - i_request;
    
    % 8. 在线估计器单步更新 (采用实际施加推力与位置量)
    % 实际施加推力指令
    FG_applied = Kf_L * iL_cmd - Kf_R * iR_cmd;
    if nargin < 13 || isempty(yG_meas)
        % 若未显式传入量化测量，默认根据控制器配置或传动参数当量计算
        if isfield(ctrl_c2, 'dy_ecd')
            dy_ecd = ctrl_c2.dy_ecd;
        else
            dy_ecd = (2.0 * pi * 0.030) / (19.0 * 8192); % 2*pi*rp / (N * ecd_cpr)
        end
        yG_meas = round(q(1) / dy_ecd) * dy_ecd;
    end
    
    % 状态变量滤波递推
    [state_c3a.filter, yG_f, ydot_f, yddot_f, FG_f, Sf_f] = ...
        state_c3a.filter.step(yG_meas, FG_applied);
    
    % RLS 参数估计递推
    phi_reg = [yddot_f; ydot_f; Sf_f];
    [state_c3a.estimator, theta_new, rls_info] = ...
        state_c3a.estimator.step(FG_f, phi_reg);
    
    state_c3a.theta_curr = theta_new;
    
    % 9. 输出诊断信息
    info.v_beta = v_beta;
    info.v_actual = [FG_applied; 0.5 * Le * (-Kf_R * iR_cmd - Kf_L * iL_cmd)];
    info.Delta_i_internal = Delta_i_internal;
    info.Delta_i_external = Delta_i_external;
    info.Delta_i_total = Delta_i_total;
    info.is_sat_internal = any(abs(i_request) > I_fw_limit + 1e-6);
    info.is_sat_external = any(abs([iL_fw; iR_fw]) > Imax + 1e-6);
    info.is_sat_any = info.is_sat_internal || info.is_sat_external;
    
    info.theta_est = theta_new;
    info.M_tot_hat = theta_new(1);
    info.bG_hat = theta_new(2);
    info.fcG_hat = theta_new(3);
    info.is_pe_active = rls_info.is_pe_active;
    info.lambda_min = rls_info.lambda_min;
    info.trace_P = rls_info.trace_P;
end
