%% CONTROLLER_C2A_SYNC_ALLOC.M - 采用推力空间同步优先约束分配的鲁棒控制器 (C2a-SyncAlloc)
% =========================================================================
% 本控制器属于 2x2 消融矩阵中的“仅同步优先分配，无动态抗饱和”组别：
%   v_beta -> thrust_allocator_sync -> i_actual
% 孤立检验在无抗饱和动态回路时，仅依靠推力空间优先保留纠偏力矩对同步精度的影响。
% =========================================================================

function [iL_cmd, iR_cmd, info] = controller_c2a_sync_alloc( ...
    q, qdot, qd, qdot_d, qddot_d, ctrl_c2, Le, Kf_L, Kf_R, Imax)

    % 1. 误差与广义误差面计算
    e = q - qd;
    edot = qdot - qdot_d;
    sigma = edot + ctrl_c2.Gamma * e;
    
    % 2. 名义动力学前馈补偿 va
    if isfield(ctrl_c2, 'enable_ff') && ~ctrl_c2.enable_ff
        va = [0.0; 0.0];
    else
        Sf = [tanh(qdot(1) / ctrl_c2.eps_v); tanh(qdot(2) / ctrl_c2.eps_alpha)];
        va = ctrl_c2.B_hat * qdot + ctrl_c2.K_hat * q + ctrl_c2.A_hat * Sf + ctrl_c2.M_hat * qddot_d;
    end
    
    % 3. 连续 tanh 鲁棒反馈补偿 vr
    tanh_sigma = tanh(sigma ./ ctrl_c2.phi);
    vr = -ctrl_c2.K1 * sigma - ctrl_c2.K2 * tanh_sigma - ctrl_c2.M_hat * ctrl_c2.Gamma * edot;
    
    % 4. 期望广义力 v_beta
    v_beta = va + vr;
    FG_des = v_beta(1);
    Talpha_des = v_beta(2);
    
    % 5. 未限幅理想请求 i_request (用于残差计算)
    [iL_req, iR_req] = actuator_map_m3508.force_to_current( ...
        FG_des, Talpha_des, Le, Kf_L, Kf_R);
    i_request = [iL_req; iR_req];
    
    % 6. 推力空间同步优先约束分配
    I_fw_limit = 16000.0;
    if isfield(ctrl_c2, 'I_fw_limit')
        I_fw_limit = ctrl_c2.I_fw_limit;
    end
    
    [i_alloc, alloc_info] = thrust_allocator_sync( ...
        FG_des, Talpha_des, Le, Kf_L, Kf_R, Imax, I_fw_limit);
    
    % 7. 物理保护级限幅 (防止数值溢出或容差外溢)
    iL_act = max(-Imax, min(Imax, i_alloc(1)));
    iR_act = max(-Imax, min(Imax, i_alloc(2)));
    i_actual = [iL_act; iR_act];
    
    iL_cmd = i_actual(1);
    iR_cmd = i_actual(2);
    
    % 8. 分级残差解算 (分配器主动缺额 vs 下游硬件保护截断)
    Delta_i_alloc = i_alloc - i_request;
    Delta_i_external = i_actual - i_alloc;
    Delta_i_total = i_actual - i_request;
    
    tol_sat = 1e-6;
    is_sat_ext = any(abs(Delta_i_external) > tol_sat);
    
    % 9. 诊断与接口信息
    info.e = e;
    info.edot = edot;
    info.sigma = sigma;
    info.va = va;
    info.vr = vr;
    info.v_beta = v_beta;
    info.v_cmd = v_beta;
    info.i_request = i_request;
    info.i_alloc = i_alloc;
    info.i_actual = i_actual;
    info.Delta_i_alloc = Delta_i_alloc;
    info.Delta_i_external = Delta_i_external;
    info.Delta_i_total = Delta_i_total;
    info.Delta_i = Delta_i_alloc;
    info.alloc_info = alloc_info;
    info.v_actual = [alloc_info.FG_alloc; alloc_info.Talpha_alloc];
    info.is_sat_torque = alloc_info.is_sat_torque;
    info.is_sat_thrust = alloc_info.is_sat_thrust;
    info.is_sat_alloc = alloc_info.is_sat_any;
    info.is_sat_external = is_sat_ext;
    info.is_sat_any = alloc_info.is_sat_any || is_sat_ext;
    info.is_sat = info.is_sat_any;
end
