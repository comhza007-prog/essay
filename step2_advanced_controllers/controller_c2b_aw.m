%% CONTROLLER_C2B_AW.M - 考虑真实执行器饱和的动态抗饱和鲁棒同步控制器 (C2b)
% =========================================================================
% 本控制器在 C2a 鲁棒误差面骨架基础上，引入基于执行器饱和残差的动态抗饱和补偿器。
% 严格满足以下学术与工程设计规范：
% 
% 1. 明确区分三级电流物理量与分级残差：
%    - i_request       : 逆向映射计算得到的未限幅理想电流请求 (counts)
%    - i_fw            : 电控固件/控制器内部限幅后的请求 (sat(i_request, 16000))
%    - i_actual        : 外部物理/实验执行器硬限幅后的实际电流 (sat(i_fw, Imax))
%    - Delta_i_internal: 软件内部限幅残差 = i_fw - i_request
%    - Delta_i_external: 外部硬件限幅残差 = i_actual - i_fw (针对执行器能力不足)
%    - Delta_i_total   : 总饱和残差     = i_actual - i_request
% 
% 2. 真正的离散动态状态记忆更新 (跨采样周期严格保存):
%    dot(z_aw) = K_aw * Delta_i_aw - lambda_aw * z_aw
%    z_aw(k+1) = z_aw(k) + Ts * dot(z_aw)(k)
% 
% 3. 广义力空间动态回退补偿:
%    v_cmd = v_beta + T_act * z_aw
%    当执行器发生饱和时，通过 T_act * z_aw 动态拉回虚拟控制力，抑制不可实现的大控制请求。
% =========================================================================

function [iL_cmd, iR_cmd, state, info] = controller_c2b_aw( ...
    q, qdot, qd, qdot_d, qddot_d, ctrl_c2, Le, Kf_L, Kf_R, Imax, state, Ts)

    % 1. 误差与广义滑模面计算 (与 C2a 完全相同，确保基线可比)
    e = q - qd;
    edot = qdot - qdot_d;
    sigma = edot + ctrl_c2.Gamma * e;
    
    % 2. 名义动力学前馈补偿 v_a (支持 enable_ff=false 消融)
    if isfield(ctrl_c2, 'enable_ff') && ~ctrl_c2.enable_ff
        va = [0.0; 0.0];
    else
        Sf = [tanh(qdot(1) / ctrl_c2.eps_v); tanh(qdot(2) / ctrl_c2.eps_alpha)];
        va = ctrl_c2.B_hat * qdot + ctrl_c2.K_hat * q + ctrl_c2.A_hat * Sf + ctrl_c2.M_hat * qddot_d;
    end
    
    % 3. 连续 tanh 鲁棒反馈补偿 v_r
    tanh_sigma = tanh(sigma ./ ctrl_c2.phi);
    vr = -ctrl_c2.K1 * sigma - ctrl_c2.K2 * tanh_sigma - ctrl_c2.M_hat * ctrl_c2.Gamma * edot;
    
    % 4. 标称未补偿虚拟广义力
    v_beta = va + vr;
    
    % 5. 提取并应用抗饱和动态补偿状态 z_aw (跨采样周期维持)
    if ~isfield(state, 'z_aw') || isempty(state.z_aw)
        state.z_aw = [0.0; 0.0];
    end
    z_aw = state.z_aw;
    
    % 执行器正向推力映射矩阵 T_act: [FG; Talpha] = T_act * [iL; iR]
    T_act = [Kf_L, -Kf_R; -0.5 * Le * Kf_L, -0.5 * Le * Kf_R];
    
    % 补偿后的综合期望广义力 v_cmd
    v_cmd = v_beta + T_act * z_aw;
    FG_cmd = v_cmd(1);
    Talpha_cmd = v_cmd(2);
    
    % 6. 逆向映射到原始理想电机电流请求 i_request (counts)
    [iL_req, iR_req] = actuator_map_m3508.force_to_current( ...
        FG_cmd, Talpha_cmd, Le, Kf_L, Kf_R);
    i_request = [iL_req; iR_req];
    
    % 7. 多级限幅与分级残差解算
    % 7.1 软件/固件内部限幅 (标称 16000 counts)
    I_fw_limit = 16000.0;
    if isfield(ctrl_c2, 'I_fw_limit')
        I_fw_limit = ctrl_c2.I_fw_limit;
    end
    iL_fw = max(-I_fw_limit, min(I_fw_limit, i_request(1)));
    iR_fw = max(-I_fw_limit, min(I_fw_limit, i_request(2)));
    i_fw = [iL_fw; iR_fw];
    
    % 7.2 外部实验物理执行器硬限幅 (当前工况 Imax, 例如 4500 或 16000)
    iL_act = max(-Imax, min(Imax, i_fw(1)));
    iR_act = max(-Imax, min(Imax, i_fw(2)));
    i_actual = [iL_act; iR_act];
    
    % 7.3 分级残差定义
    Delta_i_internal = i_fw - i_request;
    Delta_i_external = i_actual - i_fw;
    Delta_i_total    = i_actual - i_request;
    
    % 7.4 根据控制策略选取抗饱和残差源 (默认使用外部硬件饱和残差 external)
    aw_mode = 'external';
    if isfield(ctrl_c2, 'aw_mode')
        aw_mode = ctrl_c2.aw_mode;
    end
    
    switch lower(aw_mode)
        case 'external'
            Delta_i_aw = Delta_i_external;
        case 'total'
            Delta_i_aw = Delta_i_total;
        case 'internal'
            Delta_i_aw = Delta_i_internal;
        otherwise
            Delta_i_aw = Delta_i_external;
    end
    
    % 8. 离散动态状态更新 (前向欧拉积分离散化，保持真实记忆)
    % dot(z_aw) = K_aw * Delta_i_aw - lambda_aw * z_aw
    if isfield(ctrl_c2, 'K_aw')
        K_aw = ctrl_c2.K_aw;           % 2x2 增益矩阵 (例如 diag([k_aw, k_aw]))
    else
        K_aw = 20.0 * eye(2);
    end
    if isfield(ctrl_c2, 'lambda_aw')
        lambda_aw = ctrl_c2.lambda_aw; % 衰减耗散因子 (1/s)
    else
        lambda_aw = 20.0;
    end
    
    z_aw_dot = K_aw * Delta_i_aw - lambda_aw * z_aw;
    z_aw_next = z_aw + Ts * z_aw_dot;
    
    % 将状态更新写回 state
    state.z_aw = z_aw_next;
    state.z_aw_dot = z_aw_dot;
    
    % 输出实际发送给 CAN 驱动器的电流指令
    iL_cmd = i_actual(1);
    iR_cmd = i_actual(2);
    
    % 9. 回算实际施加的广义力
    [FG_actual, Talpha_actual] = actuator_map_m3508.current_to_force( ...
        iL_cmd, iR_cmd, Le, Kf_L, Kf_R);
    
    % 10. 记录详尽诊断信息
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
    info.i_fw = i_fw;
    info.i_actual = i_actual;
    info.Delta_i_internal = Delta_i_internal;
    info.Delta_i_external = Delta_i_external;
    info.Delta_i_total = Delta_i_total;
    info.Delta_i_aw = Delta_i_aw;
    info.Delta_i = Delta_i_external; % 默认对外外部硬件残差
    
    tol_sat = 1e-6;
    info.is_sat_internal = any(abs(Delta_i_internal) > tol_sat);
    info.is_sat_external = any(abs(Delta_i_external) > tol_sat);
    info.is_sat_any      = any(abs(Delta_i_total) > tol_sat);
    info.is_sat          = info.is_sat_any;
    
    info.v_actual = [FG_actual; Talpha_actual];
end
