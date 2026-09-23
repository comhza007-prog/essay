%% CONTROLLER_C1_CCC.M - 物理速度坐标下的传统交叉耦合控制器 (CCC)
% =========================================================================
% 本控制器严格执行以下规范：
% 1. 物理速度坐标设计: 同步误差 e_sync = yR - yL (m), de_sync = vR - vL (m/s)
% 2. 纠偏控制量 u_sync = Kp_sync * e_sync + Kd_sync * de_sync (m/s)
%    - Kp_sync 单位: 1/s (频率量纲)
%    - Kd_sync 单位: 无量纲标量
% 3. 严格执行动态速度限幅钳位 (保证与 C0 对比绝对公平):
%    v_limit = dynamic_max_out / ms_to_rpm
%    vL_ref = sat(vL_base + u_sync, v_limit)
%    vR_ref = sat(vR_base - u_sync, v_limit)
% 4. 正确处理右电机对向安装负向转速映射:
%    rpm_L_ref =  ms_to_rpm * vL_ref
%    rpm_R_ref = -ms_to_rpm * vR_ref
% =========================================================================

function [iL_cmd, iR_cmd, states, info] = controller_c1_ccc( ...
    yL, yR, vL, vR, yd_target, dynamic_max_out, ctrl, mech, Imax, states)

    % 传动换算比例
    m_to_ecd = (mech.N * ctrl.ecd_cpr) / (2.0 * pi * mech.rp);
    ms_to_rpm = (60.0 * mech.N) / (2.0 * pi * mech.rp);
    
    % 1. 物理位置环基础设定值 (与 C0 严格一致)
    ecd_L =  yL * m_to_ecd;
    ecd_R = -yR * m_to_ecd; % 右侧电机反馈为负
    
    ecd_target_L =  yd_target * m_to_ecd;
    ecd_target_R = -yd_target * m_to_ecd;
    
    % 位置环限幅
    states.pid_ang_L.max_out = dynamic_max_out;
    states.pid_ang_R.max_out = dynamic_max_out;
    
    % 基础位置环计算 -> 输出基准转速 (rpm)
    [rpm_base_L, states.pid_ang_L] = pid_calc(states.pid_ang_L, ecd_L, ecd_target_L);
    [rpm_base_R, states.pid_ang_R] = pid_calc(states.pid_ang_R, ecd_R, ecd_target_R);
    
    % 换算为物理基准前向速度 (m/s)
    vL_base =  rpm_base_L / ms_to_rpm;
    vR_base = -rpm_base_R / ms_to_rpm; % 右电机负转速对应正前向速度
    
    % 2. 物理几何同步误差与纠偏速度计算
    e_sync  = yR - yL;      % 右领先为正 (m)
    de_sync = vR - vL;      % 右速度快为正 (m/s)
    
    % 交叉耦合控制量 (m/s)
    u_sync = ctrl.Kp_sync * e_sync + ctrl.Kd_sync * de_sync;
    
    % 3. 施加同步速度修正 (右侧领先则左侧加速、右侧减速)
    vL_uncapped = vL_base + u_sync;
    vR_uncapped = vR_base - u_sync;
    
    % 4. 严格执行动态速度上限截断 (杜绝绕过 C0 位置环限幅)
    v_limit = dynamic_max_out / ms_to_rpm;
    vL_ref = max(-v_limit, min(v_limit, vL_uncapped));
    vR_ref = max(-v_limit, min(v_limit, vR_uncapped));
    
    % 5. 映射回电机端目标转速 (严格保持右侧对向安装负号)
    rpm_cmd_L =  ms_to_rpm * vL_ref;
    rpm_cmd_R = -ms_to_rpm * vR_ref;
    
    % 6. 底层速度环计算 -> CAN 电流指令 (raw counts)
    % 保证速度环内部限幅固定使用比赛控制器上限 (ctrl.spd_max_out = 16000)，不随实验 Imax 改变
    if isfield(ctrl, 'spd_max_out')
        states.pid_spd_L.max_out = ctrl.spd_max_out;
        states.pid_spd_R.max_out = ctrl.spd_max_out;
    end
    rpm_fdb_L =  vL * ms_to_rpm;
    rpm_fdb_R = -vR * ms_to_rpm;
    
    [iL_raw, states.pid_spd_L, iL_unsat] = pid_calc(states.pid_spd_L, rpm_fdb_L, rpm_cmd_L);
    [iR_raw, states.pid_spd_R, iR_unsat] = pid_calc(states.pid_spd_R, rpm_fdb_R, rpm_cmd_R);
    
    % 7. 执行器电流硬限幅与分级残差
    [iL_cmd, iR_cmd, Delta_i_ext, is_sat_ext] = saturation_model.hard_sat(iL_raw, iR_raw, Imax);
    
    i_request = [iL_unsat; iR_unsat];
    i_fw      = [iL_raw; iR_raw];
    i_actual  = [iL_cmd; iR_cmd];
    
    Delta_i_internal = i_fw - i_request;
    Delta_i_external = i_actual - i_fw;
    Delta_i_total    = i_actual - i_request;
    
    tol_sat = 1e-6;
    is_sat_internal = any(abs(Delta_i_internal) > tol_sat);
    is_sat_any      = any(abs(Delta_i_total) > tol_sat);
    
    % 记录调试信息
    info.e_sync = e_sync;
    info.de_sync = de_sync;
    info.u_sync = u_sync;
    info.vL_base = vL_base;
    info.vR_base = vR_base;
    info.vL_ref = vL_ref;
    info.vR_ref = vR_ref;
    info.i_request = i_request;
    info.i_fw = i_fw;
    info.i_actual = i_actual;
    info.iL_ideal = i_request(1);
    info.iR_ideal = i_request(2);
    info.Delta_i_internal = Delta_i_internal;
    info.Delta_i_external = Delta_i_external;
    info.Delta_i_total    = Delta_i_total;
    info.Delta_i          = Delta_i_external;
    info.is_sat_internal = is_sat_internal;
    info.is_sat_external = is_sat_ext;
    info.is_sat_any      = is_sat_any;
    info.is_sat          = is_sat_any;
end
