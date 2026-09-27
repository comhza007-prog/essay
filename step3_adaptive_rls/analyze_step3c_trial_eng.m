%% ANALYZE_STEP3C_TRIAL_ENG.M - 串联工程前端 (C4-A+C4-C+C4-B) 的单次非理想扰动开环回放与 RLS 辨识分析核
% =========================================================================
% 功能说明:
% 依据 STEP3_IMPLEMENTATION_PLAN.md 第四节工程前端集成规范，实现 C8A-eng 分析核:
% 1. 调用 step3c_apply_imperfections 注入三层电流、执行/测量时滞及传感噪声;
% 2. 串联工程前端标定与因果对齐:
%    - [阶段一: C4-A 零偏标定器] step3c_current_channel_calibrator.m:
%      上电/就绪静止段累积 500 点 Hampel/中位数估计 i_bias_L/R 并锁定 FROZEN;
%    - [阶段二: C4-C 增益校正器] step3c_gain_calibrator.m:
%      EXTERNAL_REFERENCE 模式 (或 ORACLE_GAIN) 校准硬件增益并锁定 FROZEN;
%    - [阶段三: C4-B 因果对齐器] step3c_causal_delay_aligner.m:
%      TIMESTAMP / 因果缓冲区硬对齐，较快电流补齐差模时滞，位置同步滞后 dmax;
% 3. 构造同物理时刻 4 阶因果巴特沃斯 SVF 滤波回归量 (使用对齐后的 yL_align, yR_align, iL_align, iR_align);
% 4. 运行 rls_estimator_delta_kf 具有 PE 能量门控与物理凸集投影的因果递推;
% 5. 解算前馈重分配因子 gamma_L, gamma_R，并在评测窗口 [0.5, 2.3]s 内评测偏航残差 RMS 与超标时间比例;
% 6. 运行四条匹配参考支路 RK4 动力学重积分。
% =========================================================================

function res = analyze_step3c_trial_eng(base_data, cfg, opts_eng)
    if nargin < 2 || isempty(cfg)
        cfg = struct();
    end
    if nargin < 3 || isempty(opts_eng)
        opts_eng = struct();
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

    % 2. 串联工程前端 (C4-A -> C4-C -> C4-B)
    % 2.1 [C4-A 零偏标定器] 静止就绪段 500 点标定
    bias_opts = struct('calibration_duration', 0.5, 'dt', dt, 'Imax', Imax);
    if isfield(opts_eng, 'bias_opts'), bias_opts = opts_eng.bias_opts; end
    state_bias = [];
    cmd_zero = struct('iL', 0.0, 'iR', 0.0);
    mot_static = struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true);
    
    % 模拟就绪静止段采样
    sigma_i_L = 10.0; if isfield(cfg, 'sigma_i_L'), sigma_i_L = cfg.sigma_i_L; end
    sigma_i_R = 10.0; if isfield(cfg, 'sigma_i_R'), sigma_i_R = cfg.sigma_i_R; end
    i_bias_L_true = 0.0; if isfield(cfg, 'i_bias_L'), i_bias_L_true = cfg.i_bias_L; end
    i_bias_R_true = 0.0; if isfield(cfg, 'i_bias_R'), i_bias_R_true = cfg.i_bias_R; end

    for k = 1:500
        raw_k = [i_bias_L_true + sigma_i_L * randn(); i_bias_R_true + sigma_i_R * randn()];
        [~, state_bias, ~] = step3c_current_channel_calibrator(raw_k, cmd_zero, mot_static, state_bias, bias_opts);
    end
    bias_hat = state_bias.bias_hat;

    % 2.2 [C4-C 增益校正器] 外置基准 EXTERNAL_REFERENCE (或 ORACLE_GAIN)
    gain_mode = 'EXTERNAL_REFERENCE';
    if isfield(opts_eng, 'gain_mode'), gain_mode = opts_eng.gain_mode; end

    delta_g_L_true = 0.0; if isfield(cfg, 'delta_g_L'), delta_g_L_true = cfg.delta_g_L; end
    delta_g_R_true = 0.0; if isfield(cfg, 'delta_g_R'), delta_g_R_true = cfg.delta_g_R; end

    if strcmp(gain_mode, 'ORACLE_GAIN')
        gain_opts = struct('mode', 'ORACLE_GAIN', 'true_gains', [1 + delta_g_L_true; 1 + delta_g_R_true], ...
            'Kf_nominal', Kf_mean, 'Imax', Imax);
        state_gain = [];
        [~, state_gain, ~] = step3c_gain_calibrator([3000; 3000], [], [], state_gain, gain_opts);
        gain_hat = state_gain.gain_hat;
    else
        % EXTERNAL_REFERENCE 模式，允许最大 5% 标定范围覆盖工业传感器公差
        gain_opts = struct( ...
            'mode', 'EXTERNAL_REFERENCE', ...
            'reference_kind', 'SIMULATED_EXTERNAL_REFERENCE', ...
            'calibration_profile', 'C8A_EXTENDED_SIM_RANGE', ...
            'N_min', 200, ...
            'Kf_nominal', Kf_mean, ...
            'Imax', Imax, ...
            'max_asymmetry_range', 0.05);
        state_gain = [];
        for k = 1:220
            c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
            c_meas_k = c_ref_k .* [1 + delta_g_L_true; 1 + delta_g_R_true] + [sigma_i_L*randn(); sigma_i_R*randn()];
            [~, state_gain, ~] = step3c_gain_calibrator(c_meas_k, c_ref_k, [], state_gain, gain_opts);
        end
        gain_hat = state_gain.gain_hat;
    end

    % 2.3 扣除零偏与增益校准
    iL_corr = (pert_data.iL_meas - bias_hat(1)) ./ gain_hat(1);
    iR_corr = (pert_data.iR_meas - bias_hat(2)) ./ gain_hat(2);

    % 2.4 [C4-B 因果对齐器] 构造硬件总线时间戳与健康度结构体，调用因果时延对齐器
    dL_true = pert_data.cfg.d_meas_L;
    dR_true = pert_data.cfg.d_meas_R;

    align_state = [];
    align_opts = struct('mode', 'TIMESTAMP', 'dt', dt);
    delay_info = struct('d_meas_hat', [dL_true; dR_true]);
    for k_warm = 1:10
        ts_align = struct();
        ts_align.t_source_L   = (k_warm - 1 - dL_true) * dt;
        ts_align.t_source_R   = (k_warm - 1 - dR_true) * dt;
        ts_align.t_source_pos = (k_warm - 1) * dt;
        ts_align.t_recv_L     = (k_warm - 1) * dt;
        ts_align.t_recv_R     = (k_warm - 1) * dt;
        ts_align.t_recv_pos   = (k_warm - 1) * dt;
        ts_align.seq_L        = k_warm;
        ts_align.seq_R        = k_warm;
        ts_align.seq_pos      = k_warm;
        ts_align.clock_id_L   = 'MASTER_BUS_CLK';
        ts_align.clock_id_R   = 'MASTER_BUS_CLK';
        ts_align.clock_id_pos = 'MASTER_BUS_CLK';

        qual_align = struct('is_saturated', [false; false], 'current_valid', [true; true], ...
                            'position_valid', [true; true], 'packet_valid', true);

        [~, align_state, delay_info] = step3c_causal_delay_aligner( ...
            [iL_corr(k_warm); iR_corr(k_warm)], [pert_data.iL_cmd(k_warm); pert_data.iR_cmd(k_warm)], ...
            [pert_data.yL_meas(k_warm); pert_data.yR_meas(k_warm)], ...
            ts_align, qual_align, align_state, align_opts);
    end

    dL = delay_info.d_meas_hat(1);
    dR = delay_info.d_meas_hat(2);

    delay_valid = isfinite(dL) && isfinite(dR) && ...
                  dL >= 0 && dR >= 0 && ...
                  mod(dL, 1) == 0 && mod(dR, 1) == 0;

    assert(delay_valid, ...
        'C8A-eng: C4-B 未输出有效估计时延，禁止使用仿真真值回退');

    res_delay_source = 'C4B_ESTIMATED';

    dmax = max(dL, dR);
    dL_extra = dmax - dL;
    dR_extra = dmax - dR;

    iL_align = zeros(N, 1);
    iR_align = zeros(N, 1);
    if dL_extra == 0
        iL_align = iL_corr;
    else
        iL_align((dL_extra + 1):N) = iL_corr(1:(N - dL_extra));
    end
    if dR_extra == 0
        iR_align = iR_corr;
    else
        iR_align((dR_extra + 1):N) = iR_corr(1:(N - dR_extra));
    end

    yL_align = zeros(N, 1);
    yR_align = zeros(N, 1);
    if dmax == 0
        yL_align = pert_data.yL_meas;
        yR_align = pert_data.yR_meas;
    else
        yL_align((dmax + 1):N) = pert_data.yL_meas(1:(N - dmax));
        yR_align((dmax + 1):N) = pert_data.yR_meas(1:(N - dmax));
    end

    % 3. 构造真实传感器滤波回归信号 (使用工程前端对齐后的信号)
    reg = build_step3b_regression(...
        yL_align, yR_align, ...
        iL_align, iR_align, ...
        dt, base_data.mech, base_data.plant, Kf_mean, 'step3c_sensor');

    % 4. 初始化 RLS 估计器
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

    % 因果逐步更新与时序变量全记录
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

    % 7.1 基准未补偿应用电流
    iL_base_cmd = pert_data.iL_cmd;
    iR_base_cmd = pert_data.iR_cmd;
    iL_base_delayed = zeros(N, 1);
    iR_base_delayed = zeros(N, 1);
    if dL_act < N, iL_base_delayed((dL_act + 1):N) = iL_base_cmd(1:(N - dL_act)); end
    if dR_act < N, iR_base_delayed((dR_act + 1):N) = iR_base_cmd(1:(N - dR_act)); end
    iL_base_applied = max(-Imax, min(Imax, iL_base_delayed));
    iR_base_applied = max(-Imax, min(Imax, iR_base_delayed));

    % 7.2 补偿后应用电流
    iL_comp_cmd = calib.gamma_L * iL_base_cmd;
    iR_comp_cmd = calib.gamma_R * iR_base_cmd;
    iL_comp_delayed = zeros(N, 1);
    iR_comp_delayed = zeros(N, 1);
    if dL_act < N, iL_comp_delayed((dL_act + 1):N) = iL_comp_cmd(1:(N - dL_act)); end
    if dR_act < N, iR_comp_delayed((dR_act + 1):N) = iR_comp_cmd(1:(N - dR_act)); end
    iL_comp_applied = max(-Imax, min(Imax, iL_comp_delayed));
    iR_comp_applied = max(-Imax, min(Imax, iR_comp_delayed));

    % 7.3 执行器在机械本体上激发的偏航推力矩与标称残差
    T_alpha_base = -0.5 * Le * (Kf_L * iL_base_applied + Kf_R * iR_base_applied);
    T_alpha_comp = -0.5 * Le * (Kf_L * iL_comp_applied + Kf_R * iR_comp_applied);
    
    T_alpha_nom  = -0.5 * Le * Kf_mean * (iL_base_delayed + iR_base_delayed);
    e_T_base = T_alpha_base - T_alpha_nom;
    e_T_comp = T_alpha_comp - T_alpha_nom;

    T_nom_intended = -0.5 * Le * Kf_mean * (iL_base_cmd + iR_base_cmd);
    e_total_base = T_alpha_base - T_nom_intended;
    e_total_comp = T_alpha_comp - T_nom_intended;

    % 8. 窗口统计 (严格限制在有效评测窗口 [t_eval_start, t_eval_end])
    mask_eval  = (t >= pert_data.t_eval_start & t <= pert_data.t_eval_end);
    mask_dwell = (t >= 3.0 & t <= 4.0);
    N_eval = sum(mask_eval);

    rms_base = sqrt(mean(e_T_base(mask_eval).^2));
    rms_comp = sqrt(mean(e_T_comp(mask_eval).^2));
    if rms_base < 1.0e-12
        eta_kf_residual = NaN;
    else
        eta_kf_residual = (1.0 - rms_comp / rms_base) * 100.0;
    end
    eta_sat = eta_kf_residual;

    rms_total_base = sqrt(mean(e_total_base(mask_eval).^2));
    rms_total_comp = sqrt(mean(e_total_comp(mask_eval).^2));
    if rms_total_base < 1.0e-12
        eta_total = NaN;
    else
        eta_total = (1.0 - rms_total_comp / rms_total_base) * 100.0;
    end

    K_alpha = base_data.plant.K_alpha;
    alpha_ss_base = rms_base / K_alpha;
    alpha_ss_comp = rms_comp / K_alpha;

    % 8.4 四条匹配参考支路因果重积分架构
    delta_m_val = 0.0;
    d_load_val = 0.0;
    delta_fric_val = 0.0;
    if isfield(pert_data.cfg, 'delta_m'), delta_m_val = pert_data.cfg.delta_m; end
    if isfield(pert_data.cfg, 'd_load'), d_load_val = pert_data.cfg.d_load; end
    if isfield(pert_data.cfg, 'delta_fric'), delta_fric_val = pert_data.cfg.delta_fric; end

    iL_base_nd_applied = max(-Imax, min(Imax, iL_base_cmd));
    iR_base_nd_applied = max(-Imax, min(Imax, iR_base_cmd));
    iL_comp_nd_applied = max(-Imax, min(Imax, iL_comp_cmd));
    iR_comp_nd_applied = max(-Imax, min(Imax, iR_comp_cmd));

    if delta_m_val == 0 && d_load_val == 0 && delta_fric_val == 0
        alpha_base_no_delay = base_data.alpha;
    else
        alpha_base_no_delay = zeros(N, 1);
        x_b_nd = zeros(4, 1);
        for k = 1:N
            [x_next, ~] = gantry_dynamics_step_rk4(...
                x_b_nd, iL_base_nd_applied(k), iR_base_nd_applied(k), ...
                base_data.mech, base_data.plant, ...
                delta_m_val, d_load_val, delta_fric_val, dt, Kf_L, Kf_R);
            alpha_base_no_delay(k) = x_b_nd(2);
            x_b_nd = x_next;
        end
    end

    alpha_base_delayed = pert_data.alpha_true;

    alpha_comp_no_delay = zeros(N, 1);
    x_c_nd = zeros(4, 1);
    for k = 1:N
        [x_next, ~] = gantry_dynamics_step_rk4(...
            x_c_nd, iL_comp_nd_applied(k), iR_comp_nd_applied(k), ...
            base_data.mech, base_data.plant, ...
            delta_m_val, d_load_val, delta_fric_val, dt, Kf_L, Kf_R);
        alpha_comp_no_delay(k) = x_c_nd(2);
        x_c_nd = x_next;
    end

    alpha_comp_delayed = zeros(N, 1);
    x_c_d = zeros(4, 1);
    for k = 1:N
        [x_next, ~] = gantry_dynamics_step_rk4(...
            x_c_d, iL_comp_applied(k), iR_comp_applied(k), ...
            base_data.mech, base_data.plant, ...
            delta_m_val, d_load_val, delta_fric_val, dt, Kf_L, Kf_R);
        alpha_comp_delayed(k) = x_c_d(2);
        x_c_d = x_next;
    end

    dalpha_base = alpha_base_delayed - alpha_base_no_delay;
    dalpha_comp = alpha_comp_delayed - alpha_comp_no_delay;

    rms_alpha_base_dyn = sqrt(mean(alpha_base_delayed(mask_eval).^2));
    rms_alpha_comp_dyn = sqrt(mean(alpha_comp_delayed(mask_eval).^2));
    rms_dalpha_base    = sqrt(mean(dalpha_base(mask_eval).^2));
    rms_dalpha_comp    = sqrt(mean(dalpha_comp(mask_eval).^2));

    if rms_alpha_base_dyn > 1.0e-12
        eta_alpha_abs = (1.0 - rms_alpha_comp_dyn / rms_alpha_base_dyn) * 100.0;
    else
        eta_alpha_abs = NaN;
    end
    eta_alpha_dyn = eta_alpha_abs;

    if rms_dalpha_base > 1.0e-12
        eta_alpha_delay = (1.0 - rms_dalpha_comp / rms_dalpha_base) * 100.0;
    else
        eta_alpha_delay = NaN;
    end

    if isfield(pert_data, 'v_yL') && (pert_data.cfg.sigma_y_L > 0 || pert_data.cfg.sigma_y_R > 0)
        e_alpha_raw = (pert_data.v_yR - pert_data.v_yL) / Le;
        fc = 10.0; wc = 2.0 * pi * fc;
        poly_den = [1.0, 2.61312592975275 * wc, 3.41421356237310 * (wc^2), ...
                    2.61312592975275 * (wc^3), wc^4];
        sys_w0_d = c2d(tf(wc^4, poly_den), dt, 'tustin');
        [num_w0, den_a] = tfdata(sys_w0_d, 'v');
        alpha_raw_clean = (pert_data.yR_q - pert_data.yL_q) / Le;
        alpha_f_clean   = filter(num_w0, den_a, alpha_raw_clean);
        e_alpha_filt    = reg.alpha_f - alpha_f_clean;
        
        rms_noise_raw  = sqrt(mean(e_alpha_raw(mask_eval).^2));
        rms_noise_filt = sqrt(mean(e_alpha_filt(mask_eval).^2));
        if rms_noise_filt > 1e-12
            svf_atten_dB = 20.0 * log10(rms_noise_raw / rms_noise_filt);
        else
            svf_atten_dB = NaN;
        end
    else
        svf_atten_dB = NaN;
    end

    pe_false_alarm_rate = mean(pe_mask(mask_dwell));
    pe_active_ratio     = mean(pe_mask(mask_eval));

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

    unproj_eval = theta_unprojected(mask_eval);
    proj_eval   = theta_projected(mask_eval);
    unproj_low_clip_count  = sum(unproj_eval < opts_rls.theta_min - 1e-9);
    unproj_high_clip_count = sum(unproj_eval > opts_rls.theta_max + 1e-9);
    unproj_clipped_count   = unproj_low_clip_count + unproj_high_clip_count;
    sample_clip_ratio      = unproj_clipped_count / N_eval;
    has_any_clip           = (unproj_clipped_count > 0);
    unproj_max_peak        = max(abs(unproj_eval));
    exceed_false_ratio     = 100.0 * mean(abs(proj_eval) > 1.0e-5);

    % 打包单次试验输出
    res = struct();
    res.cfg                 = cfg;
    res.delta_m             = delta_m_val;
    res.d_load              = d_load_val;
    res.delta_fric          = delta_fric_val;
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
    res.eta_kf_residual     = eta_kf_residual;
    res.rms_total_base      = rms_total_base;
    res.rms_total_comp      = rms_total_comp;
    res.eta_total           = eta_total;

    res.alpha_ss_base       = alpha_ss_base;
    res.alpha_ss_comp       = alpha_ss_comp;
    res.rms_dalpha_base     = rms_dalpha_base;
    res.rms_dalpha_comp     = rms_dalpha_comp;
    res.rms_alpha_base_dyn  = rms_alpha_base_dyn;
    res.rms_alpha_comp_dyn  = rms_alpha_comp_dyn;
    res.eta_alpha_abs       = eta_alpha_abs;
    res.eta_alpha_delay     = eta_alpha_delay;
    res.eta_alpha_dyn       = eta_alpha_dyn;
    res.alpha_base_delayed  = alpha_base_delayed;
    res.alpha_comp_delayed  = alpha_comp_delayed;
    res.alpha_base_no_delay = alpha_base_no_delay;
    res.alpha_comp_no_delay = alpha_comp_no_delay;

    res.svf_atten_dB        = svf_atten_dB;
    res.pe_active_ratio     = pe_active_ratio;
    res.pe_false_alarm_rate = pe_false_alarm_rate;
    res.base_sat_ratio_L    = base_sat_ratio_L;
    res.base_sat_ratio_R    = base_sat_ratio_R;
    res.base_sat_ratio_total= base_sat_ratio_total;
    res.comp_sat_ratio_L    = comp_sat_ratio_L;
    res.comp_sat_ratio_R    = comp_sat_ratio_R;
    res.comp_sat_ratio_total= comp_sat_ratio_total;

    res.unproj_low_clip_count = unproj_low_clip_count;
    res.unproj_high_clip_count= unproj_high_clip_count;
    res.unproj_clipped_count  = unproj_clipped_count;
    res.sample_clip_ratio     = sample_clip_ratio;
    res.has_any_clip          = has_any_clip;
    res.unproj_max_peak       = unproj_max_peak;
    res.exceed_false_ratio    = exceed_false_ratio;

    res.t                   = t;
    res.theta_unprojected   = theta_unprojected;
    res.theta_projected     = theta_projected;
    res.P_history           = P_history;
    res.projection_mask     = projection_mask;
    res.pe_mask             = pe_mask;
    res.mask_eval           = mask_eval;
    res.t_eval_start        = pert_data.t_eval_start;
    res.t_eval_end          = pert_data.t_eval_end;

    % 附加工程前端专有诊断信息
    res.bias_hat            = bias_hat;
    res.gain_hat            = gain_hat;
    res.d_meas_align        = [dL; dR];
    res.delay_estimation_valid = delay_valid;
    res.delay_used_source      = res_delay_source;
    res.delay_true_for_audit   = [dL_true; dR_true];
    res.delay_estimation_error = [dL; dR] - [dL_true; dR_true];
end
