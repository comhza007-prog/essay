%% CONTROLLER_C2A_ROBUST.M - 广义误差面连续鲁棒同步控制器 (C2a 基线)
% =========================================================================
% 本控制器实现二自由度 (质心平动 yG + 横梁偏角 alpha) 鲁棒滑模面跟踪控制。
% 作为无抗饱和的对照基线，与 C2b 保持完全相同的物理执行器三级限幅结构：
%   i_request -> sat(16000) [i_fw] -> sat(Imax) [i_actual]
% 唯一的物理与算法区别是：C2a 不包含 z_aw 动态抗饱和补偿回路。
%
% 1. 广义滑动误差面:
%    e = q - qd
%    sigma = edot + Gamma * e
% 2. 名义动力学前馈补偿 va (支持 enable_ff=false 的 C2a-noFF 消融):
%    va = B_hat*qdot + K_hat*q + A_hat*Sf + M_hat*qddot_d
% 3. 连续化鲁棒反馈 vr:
%    vr = -K1*sigma - K2*tanh(sigma ./ phi) - M_hat*Gamma*edot
% 4. 理想广义力:
%    v_beta = va + vr
% 5. 逆向推力映射与三级电流限幅:
%    i_request -> sat(16000) -> sat(Imax)
% 6. 分解内部/外部饱和标志:
%    is_sat_internal, is_sat_external, is_sat_any
% =========================================================================

function [iL_cmd, iR_cmd, info] = controller_c2a_robust( ...
    q, qdot, qd, qdot_d, qddot_d, ctrl_c2, Le, Kf_L, Kf_R, Imax)

    % 1. 误差与广义误差面计算
    e = q - qd;
    edot = qdot - qdot_d;
    sigma = edot + ctrl_c2.Gamma * e;
    
    % 2. 名义动力学前馈补偿 va (支持 enable_ff=false 消融组)
    if isfield(ctrl_c2, 'enable_ff') && ~ctrl_c2.enable_ff
        va = [0.0; 0.0];
    else
        Sf = [tanh(qdot(1) / ctrl_c2.eps_v); tanh(qdot(2) / ctrl_c2.eps_alpha)];
        va = ctrl_c2.B_hat * qdot + ctrl_c2.K_hat * q + ctrl_c2.A_hat * Sf + ctrl_c2.M_hat * qddot_d;
    end
    
    % 3. 连续 tanh 鲁棒反馈补偿 vr
    tanh_sigma = tanh(sigma ./ ctrl_c2.phi);
    vr = -ctrl_c2.K1 * sigma - ctrl_c2.K2 * tanh_sigma - ctrl_c2.M_hat * ctrl_c2.Gamma * edot;
    
    % 4. 综合期望广义力 v_beta = [FG_des; Talpha_des]
    v_beta = va + vr;
    FG_des = v_beta(1);
    Talpha_des = v_beta(2);
    
    % 5. 逆向映射到原始理想电机控制电流 i_request (counts)
    [iL_req, iR_req] = actuator_map_m3508.force_to_current( ...
        FG_des, Talpha_des, Le, Kf_L, Kf_R);
    i_request = [iL_req; iR_req];
    
    % 6. 多级限幅与分级残差解算 (与 C2b 保持完全一致的执行器链条)
    % 6.1 软件/固件内部限幅 (标称 16000 counts)
    I_fw_limit = 16000.0;
    if isfield(ctrl_c2, 'I_fw_limit')
        I_fw_limit = ctrl_c2.I_fw_limit;
    end
    iL_fw = max(-I_fw_limit, min(I_fw_limit, i_request(1)));
    iR_fw = max(-I_fw_limit, min(I_fw_limit, i_request(2)));
    i_fw = [iL_fw; iR_fw];
    
    % 6.2 外部实验物理执行器硬限幅 (当前工况 Imax, 例如 4500 或 16000)
    iL_act = max(-Imax, min(Imax, i_fw(1)));
    iR_act = max(-Imax, min(Imax, i_fw(2)));
    i_actual = [iL_act; iR_act];
    
    % 6.3 分级残差定义
    Delta_i_internal = i_fw - i_request;
    Delta_i_external = i_actual - i_fw;
    Delta_i_total    = i_actual - i_request;
    
    % 7. 输出实际执行器指令
    iL_cmd = i_actual(1);
    iR_cmd = i_actual(2);
    
    % 8. 实际实现广义力回算 (前向投影)
    [FG_actual, Talpha_actual] = actuator_map_m3508.current_to_force( ...
        iL_cmd, iR_cmd, Le, Kf_L, Kf_R);
    
    % 9. 分级饱和状态辨识
    tol_sat = 1e-6;
    is_sat_internal = any(abs(Delta_i_internal) > tol_sat);
    is_sat_external = any(abs(Delta_i_external) > tol_sat);
    is_sat_any      = any(abs(Delta_i_total) > tol_sat);
    
    % 记录调试与诊断信息
    info.e = e;
    info.edot = edot;
    info.sigma = sigma;
    info.va = va;
    info.vr = vr;
    info.v_beta = v_beta;
    info.v_cmd = v_beta; % C2a 无抗饱和补偿，v_cmd 恒等于 v_beta
    info.i_request = i_request;
    info.i_fw = i_fw;
    info.i_actual = i_actual;
    info.iL_ideal = i_request(1); % 兼容旧接口
    info.iR_ideal = i_request(2);
    info.Delta_i_internal = Delta_i_internal;
    info.Delta_i_external = Delta_i_external;
    info.Delta_i_total = Delta_i_total;
    info.Delta_i = Delta_i_external; % 默认外部硬件残差
    
    info.is_sat_internal = is_sat_internal;
    info.is_sat_external = is_sat_external;
    info.is_sat_any = is_sat_any;
    info.is_sat = is_sat_any; % 兼容接口
    
    info.v_actual = [FG_actual; Talpha_actual];
end
