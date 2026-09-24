%% ANALYZE_STEP3C_TRIAL.M - Step 3C 单次非理想扰动开环回放与 RLS 辨识分析核
% =========================================================================
% 功能说明:
% 1. 调用 step3c_apply_imperfections 注入三层电流、执行/测量时滞及传感噪声
% 2. 严格调用 build_step3b_regression 从 (yL_meas, yR_meas, iL_meas, iR_meas)
%    构建真实 4 阶因果巴特沃斯 SVF 滤波回归量 (严禁读取状态真值)
% 3. 运行 rls_estimator_delta_kf 单参数因果因果递推与凸集投影
% 4. 显式记录并导出完整的时序变量序列:
%    - theta_unprojected : 未受限估计流
%    - theta_projected   : 投影保护流
%    - P_history         : 协方差演化流
%    - projection_mask   : 投影触发时间掩码
%    - pe_mask           : PE 能量门控激活掩码
% 5. 调用 step3b_offline_calibration 解算前馈重分配增益 gamma_L, gamma_R
% 6. 在评测窗口 [t_eval_start, 2.3]s 内计算基线与补偿后的偏航力矩残差 RMS 与抑制比 eta_sat
% =========================================================================

function res = analyze_step3c_trial(base_data, cfg)
    if nargin < 2
        cfg = struct();
    end

    dt = base_data.dt;
    Le = base_data.mech.Le;
    Kf_mean = base_data.Kf_mean;
    Kf_L = base_data.Kf_L;
    Kf_R = base_data.Kf_R;
    if isfield(base_data, 'Imax')
        Imax = base_data.Imax;
    else
        Imax = 16000.0;
    end

    % 1. 注入物理非理想扰动 (三层电流、时滞因果重积分、传感器噪声)
    pert_data = step3c_apply_imperfections(base_data, cfg);
    N = pert_data.N;
    t = pert_data.t;

    % 2. 构造真实传感器滤波回归信号 (严格使用测量电流 i_meas)
    reg = build_step3b_regression(...
        pert_data.yL_meas, pert_data.yR_meas, ...
        pert_data.iL_meas, pert_data.iR_meas, ...
        dt, base_data.mech, base_data.plant, Kf_mean, 'step3c_sensor');

    % 3. 初始化 RLS 估计器 (继承 Phase 1 权威超参数与物理非对称凸集)
    opts_rls = struct();
    opts_rls.lambda        = 1.0;
    opts_rls.window_length = 200;
    opts_rls.sigma_PE_th   = 50.0;
    opts_rls.theta_min     = -0.0026295;
    opts_rls.theta_max     =  0.0018465;
    opts_rls.P0            = 1.0e-4;
    opts_rls.P_min         = 1.0e-12;
    opts_rls.P_max         = 1.0;
    if isfield(cfg, 'lambda'), opts_rls.lambda = cfg.lambda; end
    if isfield(cfg, 'P0'), opts_rls.P0 = cfg.P0; end
    if isfield(cfg, 'theta_min'), opts_rls.theta_min = cfg.theta_min; end
    if isfield(cfg, 'theta_max'), opts_rls.theta_max = cfg.theta_max; end

    estimator = rls_estimator_delta_kf(opts_rls);

    % 4. 因果逐步更新与时序变量全记录
    theta_unprojected = zeros(N, 1);
    theta_projected   = zeros(N, 1);
    P_history         = zeros(N, 1);
    projection_mask   = false(N, 1);
    pe_mask           = false(N, 1);

    for k = 1:N
        phi_k = reg.phi_f(k);
        y_k   = reg.y_f(k);

        [estimator, ~, info] = estimator.update(phi_k, y_k);

        theta_unprojected(k) = info.theta_unprojected;
        theta_projected(k)   = info.theta_projected;
        P_history(k)         = info.P_next;
        projection_mask(k)   = info.is_projected;
        pe_mask(k)           = info.is_pe;
    end

    % 5. 提取强激励段收敛估计值 (取 t = 2.3s 激励段末尾稳态估计)
    idx_eval_end = find(t <= pert_data.t_eval_end, 1, 'last');
    final_theta = theta_projected(idx_eval_end);
    unproj_final_theta = theta_unprojected(idx_eval_end);

    % 6. 离线前馈标定计算
    calib = step3b_offline_calibration(final_theta, Kf_mean);

    % 7. 执行器前馈重分配与偏航残差计算
    dL_act = pert_data.cfg.d_act_L;
    dR_act = pert_data.cfg.d_act_R;

    % 7.1 基准未补偿应用电流 (含执行时滞与物理限幅)
    iL_base_cmd = pert_data.iL_cmd;
    iR_base_cmd = pert_data.iR_cmd;
    iL_base_delayed = zeros(N, 1);
    iR_base_delayed = zeros(N, 1);
    if dL_act < N
        iL_base_delayed((dL_act + 1):N) = iL_base_cmd(1:(N - dL_act));
    end
    if dR_act < N
        iR_base_delayed((dR_act + 1):N) = iR_base_cmd(1:(N - dR_act));
    end
    iL_base_applied = max(-Imax, min(Imax, iL_base_delayed));
    iR_base_applied = max(-Imax, min(Imax, iR_base_delayed));

    % 7.2 补偿后应用电流 (前馈缩放 -> 执行时滞 -> 物理限幅)
    iL_comp_cmd = calib.gamma_L * iL_base_cmd;
    iR_comp_cmd = calib.gamma_R * iR_base_cmd;
    iL_comp_delayed = zeros(N, 1);
    iR_comp_delayed = zeros(N, 1);
    if dL_act < N
        iL_comp_delayed((dL_act + 1):N) = iL_comp_cmd(1:(N - dL_act));
    end
    if dR_act < N
        iR_comp_delayed((dR_act + 1):N) = iR_comp_cmd(1:(N - dR_act));
    end
    iL_comp_applied = max(-Imax, min(Imax, iL_comp_delayed));
    iR_comp_applied = max(-Imax, min(Imax, iR_comp_delayed));

    % 7.3 执行器在机械本体上激发的偏航推力矩与标称残差
    % (物理真实推力 FL = Kf_L * iL, FR = -Kf_R * iR)
    % 偏航力矩 T_alpha = 0.5 * Le * (FR - FL) = -0.5 * Le * (Kf_L * iL + Kf_R * iR)
    T_alpha_base = -0.5 * Le * (Kf_L * iL_base_applied + Kf_R * iR_base_applied);
    T_alpha_comp = -0.5 * Le * (Kf_L * iL_comp_applied + Kf_R * iR_comp_applied);
    
    % 标称目标期望偏航力矩 (名义无偏差期望值)
    T_alpha_nom  = -0.5 * Le * Kf_mean * (iL_base_delayed + iR_base_delayed);
    
    e_T_base = T_alpha_base - T_alpha_nom;
    e_T_comp = T_alpha_comp - T_alpha_nom;

    % 8. 窗口统计 (严格限制在有效评测窗口 [t_eval_start, t_eval_end])
    mask_eval = (t >= pert_data.t_eval_start & t <= pert_data.t_eval_end);
    N_eval = sum(mask_eval);

    rms_base = sqrt(mean(e_T_base(mask_eval).^2));
    rms_comp = sqrt(mean(e_T_comp(mask_eval).^2));

    if rms_base < 1.0e-12
        eta_sat = NaN;
    else
        eta_sat = (1.0 - rms_comp / rms_base) * 100.0;
    end

    % 8.1 准静态偏航角推算 (alpha_ss = e_T / K_alpha)
    K_alpha = base_data.plant.K_alpha;
    alpha_ss_base = rms_base / K_alpha;
    alpha_ss_comp = rms_comp / K_alpha;

    % 9. 电流饱和率统计 (全时段统计)
    base_sat_mask_L = (abs(iL_base_applied) >= Imax - 1e-6);
    base_sat_mask_R = (abs(iR_base_applied) >= Imax - 1e-6);
    base_sat_ratio_L = 100.0 * mean(base_sat_mask_L);
    base_sat_ratio_R = 100.0 * mean(base_sat_mask_R);
    base_sat_ratio_total = 100.0 * mean(base_sat_mask_L | base_sat_mask_R);

    comp_sat_mask_L = (abs(iL_comp_applied) >= Imax - 1e-6);
    comp_sat_mask_R = (abs(iR_comp_applied) >= Imax - 1e-6);
    comp_sat_ratio_L = 100.0 * mean(comp_sat_mask_L);
    comp_sat_ratio_R = 100.0 * mean(comp_sat_mask_R);
    comp_sat_ratio_total = 100.0 * mean(comp_sat_mask_L | comp_sat_mask_R);

    % 10. 越界与安全诊断
    unproj_eval = theta_unprojected(mask_eval);
    proj_eval   = theta_projected(mask_eval);
    unproj_clipped_count = sum(unproj_eval < opts_rls.theta_min - 1e-9 | unproj_eval > opts_rls.theta_max + 1e-9);
    sample_clip_ratio = unproj_clipped_count / N_eval;
    has_any_clip = (unproj_clipped_count > 0);
    unproj_max_peak = max(abs(unproj_eval));
    pe_active_ratio = mean(pe_mask(mask_eval));

    % 11. 打包单次试验输出
    res = struct();
    res.cfg                 = cfg;
    res.Delta_Kf_true       = base_data.Delta_Kf_true;
    res.theta_hat           = final_theta;
    res.theta_unproj_final  = unproj_final_theta;
    res.Kf_L_hat            = calib.Kf_L_hat;
    res.Kf_R_hat            = calib.Kf_R_hat;
    res.gamma_L             = calib.gamma_L;
    res.gamma_R             = calib.gamma_R;
    res.is_calib_valid      = calib.is_valid;
    if calib.is_valid
        res.calib_validity_str = 'VALID';
    else
        res.calib_validity_str = 'INVALID';
    end

    res.rms_base            = rms_base;
    res.rms_comp            = rms_comp;
    res.eta_sat             = eta_sat;
    res.alpha_ss_base       = alpha_ss_base;
    res.alpha_ss_comp       = alpha_ss_comp;

    res.base_sat_ratio_L    = base_sat_ratio_L;
    res.base_sat_ratio_R    = base_sat_ratio_R;
    res.base_sat_ratio_total= base_sat_ratio_total;
    res.comp_sat_ratio_L    = comp_sat_ratio_L;
    res.comp_sat_ratio_R    = comp_sat_ratio_R;
    res.comp_sat_ratio_total= comp_sat_ratio_total;

    res.unproj_max_peak     = unproj_max_peak;
    res.unproj_clipped_count= unproj_clipped_count;
    res.sample_clip_ratio   = sample_clip_ratio;
    res.has_any_clip        = has_any_clip;
    res.pe_active_ratio     = pe_active_ratio;

    % 完整时序数据
    res.t                   = t;
    res.theta_unprojected   = theta_unprojected;
    res.theta_projected     = theta_projected;
    res.P_history           = P_history;
    res.projection_mask     = projection_mask;
    res.pe_mask             = pe_mask;
end
