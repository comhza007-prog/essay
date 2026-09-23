%% CONTROLLER_C2B_SYNC_ALLOC.M - 复合动态抗饱和与推力空间同步优先分配控制器 (C2b-SyncAlloc)
% =========================================================================
% 本控制器属于 2x2 消融矩阵中的“复合提出方法”组别：
%   v_cmd = v_beta + T_act * z_aw -> thrust_allocator_sync -> i_actual
% 
% 核心闭环拓扑与残差驱动规范：
% 1. 广义力空间动态回退:
%    v_cmd = v_beta + T_act * z_aw
% 2. 未限幅理想电流请求:
%    i_request = T_act_inv * v_cmd
% 3. 推力空间同步优先约束分配:
%    i_actual = thrust_allocator_sync(v_cmd, I_eff)
% 4. 驱动动态抗饱和状态的闭环残差严格定义为分配缺额:
%    Delta_i_alloc = i_actual - i_request
%    dot(z_aw) = K_aw * Delta_i_alloc - lambda_aw * z_aw
%    z_aw(k+1) = z_aw(k) + Ts * dot(z_aw)(k)
% =========================================================================

function [iL_cmd, iR_cmd, state, info] = controller_c2b_sync_alloc( ...
    q, qdot, qd, qdot_d, qddot_d, ctrl_c2, Le, Kf_L, Kf_R, Imax, state, Ts)

    % 1. 误差与广义滑模面计算
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
    
    % 4. 标称广义力 v_beta
    v_beta = va + vr;
    
    % 5. 提取抗饱和动态补偿状态 z_aw 并形成补偿后广义力 v_cmd
    if ~isfield(state, 'z_aw') || isempty(state.z_aw)
        state.z_aw = [0.0; 0.0];
    end
    z_aw = state.z_aw;
    
    % 正向推力映射矩阵 T_act
    T_act = [Kf_L, -Kf_R; -0.5 * Le * Kf_L, -0.5 * Le * Kf_R];
    v_cmd = v_beta + T_act * z_aw;
    FG_cmd = v_cmd(1);
    Talpha_cmd = v_cmd(2);
    
    % 6. 未受限理想电流请求 i_request (基于补偿后的期望力)
    [iL_req, iR_req] = actuator_map_m3508.force_to_current( ...
        FG_cmd, Talpha_cmd, Le, Kf_L, Kf_R);
    i_request = [iL_req; iR_req];
    
    % 7. 推力空间同步优先约束分配
    I_fw_limit = 16000.0;
    if isfield(ctrl_c2, 'I_fw_limit')
        I_fw_limit = ctrl_c2.I_fw_limit;
    end
    
    [i_alloc, alloc_info] = thrust_allocator_sync( ...
        FG_cmd, Talpha_cmd, Le, Kf_L, Kf_R, Imax, I_fw_limit);
    
    % 8. 物理保护级限幅 (防止数值溢出或容差外溢)
    iL_act = max(-Imax, min(Imax, i_alloc(1)));
    iR_act = max(-Imax, min(Imax, i_alloc(2)));
    i_actual = [iL_act; iR_act];
    
    iL_cmd = i_actual(1);
    iR_cmd = i_actual(2);
    
    % 9. 分配缺额残差与抗饱和状态动态更新
    Delta_i_alloc = i_alloc - i_request;
    Delta_i_external = i_actual - i_alloc;
    Delta_i_total = i_actual - i_request;
    
    tol_sat = 1e-6;
    is_sat_ext = any(abs(Delta_i_external) > tol_sat);
    
    if isfield(ctrl_c2, 'K_aw')
        K_aw = ctrl_c2.K_aw;
    else
        K_aw = 20.0 * eye(2);
    end
    if isfield(ctrl_c2, 'lambda_aw')
        lambda_aw = ctrl_c2.lambda_aw;
    else
        lambda_aw = 20.0;
    end
    
    z_aw_dot = K_aw * Delta_i_alloc - lambda_aw * z_aw;
    z_aw_next = z_aw + Ts * z_aw_dot;
    
    state.z_aw = z_aw_next;
    state.z_aw_dot = z_aw_dot;
    
    % 10. 诊断与接口信息
    info.e = e;
    info.edot = edot;
    info.sigma = sigma;
    info.va = va;
    info.vr = vr;
    info.v_beta = v_beta;
    info.v_cmd = v_cmd;
    info.z_aw = z_aw;
    info.z_aw_dot = z_aw_dot;
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
