%% ANALYZE_STEP3B_CALIBRATION.M - 标定补偿开环回放与偏航力矩残差解算分析函数
% =========================================================================
% 功能说明:
% 1. 执行严格的开环离线数据回放 (Replay), 保持动力学与闭环隔离
% 2. 严格执行“先增益缩放、后物理限幅”的电流补偿执行时序:
%      iL_comp_cmd = gamma_L * iL_nom
%      iR_comp_cmd = gamma_R * iR_nom
%      iL_applied  = sat(iL_comp_cmd, -Imax, Imax)
%      iR_applied  = sat(iR_comp_cmd, -Imax, Imax)
% 3. 严格计算补偿后偏航力矩残差物理方程:
%      e_T_comp = T_alpha_comp - T_alpha_nom
%      e_T_base = T_alpha_base - T_alpha_nom
% 4. 形式化时间窗口切分:
%      强激励评测段: t in [0.5, 2.3] s
%      停顿静止段:   t in [3.0, 4.0] s (独立报告, 杜绝与激励段混淆)
% 5. 严格处理分母零除保护 (BASELINE_TOO_SMALL) 与三阶抑制比计算
% =========================================================================

function res = analyze_step3b_calibration(dataset, calib, Imax, current_scale)
    assert(nargin >= 3 && ~isempty(Imax), ...
        '必须显式传入 Imax，禁止使用隐含默认限幅');
    if nargin < 4 || isempty(current_scale)
        current_scale = 1.0;
    end
    
    assert(isfinite(Imax) && Imax > 0, 'Imax 必须为正有限值');
    assert(isfinite(current_scale) && current_scale > 0, 'current_scale 必须为正有限值');
    
    t = dataset.t(:);
    Le = dataset.Le;
    Kf_L = dataset.Kf_L;
    Kf_R = dataset.Kf_R;
    Kf_mean = dataset.Kf_mean;
    
    % 1. 名义电流信号提取与幅值缩放
    iL_nom = dataset.iL_actual(:) * current_scale;
    iR_nom = dataset.iR_actual(:) * current_scale;
    
    % 2. 未补偿基线分支 (按标称 Imax 限幅)
    iL_applied_base = min(max(iL_nom, -Imax), Imax);
    iR_applied_base = min(max(iR_nom, -Imax), Imax);
    
    % 实际施加偏航力矩 (基线)
    T_alpha_base = -0.5 * Le * (Kf_L * iL_applied_base + Kf_R * iR_applied_base);
    
    % 标称目标偏航力矩 (名义无偏差期望值)
    T_alpha_nom = -0.5 * Le * Kf_mean * (iL_nom + iR_nom);
    
    % 基线偏航力矩残差
    e_T_base = T_alpha_base - T_alpha_nom;
    
    % 3. 补偿分支: 先增益缩放, 后物理限幅
    gamma_L = calib.gamma_L;
    gamma_R = calib.gamma_R;
    
    iL_comp_cmd = gamma_L * iL_nom;
    iR_comp_cmd = gamma_R * iR_nom;
    
    % 实际驱动器限幅执行
    iL_applied_comp = min(max(iL_comp_cmd, -Imax), Imax);
    iR_applied_comp = min(max(iR_comp_cmd, -Imax), Imax);
    
    % 补偿后实际偏航力矩 (含饱和)
    T_alpha_comp = -0.5 * Le * (Kf_L * iL_applied_comp + Kf_R * iR_applied_comp);
    e_T_comp = T_alpha_comp - T_alpha_nom;
    
    % 补偿后无饱和理论力矩 (用于计算无饱和抑制比)
    T_alpha_comp_unsat = -0.5 * Le * (Kf_L * iL_comp_cmd + Kf_R * iR_comp_cmd);
    e_T_comp_unsat = T_alpha_comp_unsat - T_alpha_nom;
    
    % 4. 饱和状态逐点统计 (严格解耦基线名义支路与补偿后命令支路)
    % 4.1 基线支路饱和统计 (名义未补偿指令是否触及 Imax)
    base_sat_mask_L = (abs(iL_nom) >= Imax);
    base_sat_mask_R = (abs(iR_nom) >= Imax);
    base_sat_mask_total = (base_sat_mask_L | base_sat_mask_R);
    
    base_sat_ratio_L = mean(base_sat_mask_L) * 100.0;
    base_sat_ratio_R = mean(base_sat_mask_R) * 100.0;
    base_sat_ratio_total = mean(base_sat_mask_total) * 100.0;
    
    % 4.2 补偿支路饱和统计 (经增益缩放后的补偿指令是否触及 Imax)
    comp_sat_mask_L = (abs(iL_comp_cmd) >= Imax);
    comp_sat_mask_R = (abs(iR_comp_cmd) >= Imax);
    comp_sat_mask_total = (comp_sat_mask_L | comp_sat_mask_R);
    
    comp_sat_ratio_L = mean(comp_sat_mask_L) * 100.0;
    comp_sat_ratio_R = mean(comp_sat_mask_R) * 100.0;
    comp_sat_ratio_total = mean(comp_sat_mask_total) * 100.0;
    
    % 5. 严格时间窗口掩码
    mask_eval = (t >= 0.5 & t <= 2.3);    % 强激励主评测窗口
    mask_dwell = (t >= 3.0 & t <= 4.0);   % 停顿静止窗口
    
    % 6. 各时间窗口 RMS 均方根残差计算
    rms_base = sqrt(mean(e_T_base(mask_eval).^2));
    rms_comp = sqrt(mean(e_T_comp(mask_eval).^2));
    rms_comp_unsat = sqrt(mean(e_T_comp_unsat(mask_eval).^2));
    
    rms_dwell_base = sqrt(mean(e_T_base(mask_dwell).^2));
    rms_dwell_comp = sqrt(mean(e_T_comp(mask_dwell).^2));
    
    % 7. 评测窗口内驱动电流统计
    IL_rms_before = sqrt(mean(iL_applied_base(mask_eval).^2));
    IR_rms_before = sqrt(mean(iR_applied_base(mask_eval).^2));
    IL_rms_after  = sqrt(mean(iL_applied_comp(mask_eval).^2));
    IR_rms_after  = sqrt(mean(iR_applied_comp(mask_eval).^2));
    
    IL_peak_before = max(abs(iL_applied_base(mask_eval)));
    IR_peak_before = max(abs(iR_applied_base(mask_eval)));
    IL_peak_after  = max(abs(iL_applied_comp(mask_eval)));
    IR_peak_after  = max(abs(iR_applied_comp(mask_eval)));
    
    % 8. 偏航力矩抑制比计算与分母零保护
    tol_baseline = 1.0e-12;
    if rms_base <= tol_baseline
        eta_sat = NaN;
        eta_unsat = NaN;
        suppression_status = 'BASELINE_TOO_SMALL';
    else
        eta_sat = (1.0 - rms_comp / rms_base) * 100.0;
        eta_unsat = (1.0 - rms_comp_unsat / rms_base) * 100.0;
        suppression_status = 'COMPUTED';
    end
    
    % 9. 准静态偏航角改善理论推算 (alpha_ss = e_T / K_alpha)
    K_alpha = dataset.plant.K_alpha;
    alpha_ss_base = rms_base / K_alpha; % [rad]
    alpha_ss_comp = rms_comp / K_alpha; % [rad]
    if rms_base <= tol_baseline
        alpha_ss_improve_pct = NaN;
    else
        alpha_ss_improve_pct = (1.0 - alpha_ss_comp / alpha_ss_base) * 100.0;
    end
    
    % 10. 组装输出结果
    res = struct();
    res.current_scale        = current_scale;
    res.Imax                 = Imax;
    res.gamma_L              = gamma_L;
    res.gamma_R              = gamma_R;
    res.rms_base             = rms_base;
    res.rms_comp             = rms_comp;
    res.rms_comp_unsat       = rms_comp_unsat;
    res.rms_dwell_base       = rms_dwell_base;
    res.rms_dwell_comp       = rms_dwell_comp;
    res.eta_sat              = eta_sat;
    res.eta_unsat            = eta_unsat;
    res.suppression_status   = suppression_status;
    res.base_sat_ratio_L     = base_sat_ratio_L;
    res.base_sat_ratio_R     = base_sat_ratio_R;
    res.base_sat_ratio_total = base_sat_ratio_total;
    res.comp_sat_ratio_L     = comp_sat_ratio_L;
    res.comp_sat_ratio_R     = comp_sat_ratio_R;
    res.comp_sat_ratio_total = comp_sat_ratio_total;
    % 兼容别名
    res.sat_ratio_L          = comp_sat_ratio_L;
    res.sat_ratio_R          = comp_sat_ratio_R;
    res.sat_ratio_total      = comp_sat_ratio_total;
    res.IL_rms_before        = IL_rms_before;
    res.IR_rms_before        = IR_rms_before;
    res.IL_rms_after         = IL_rms_after;
    res.IR_rms_after         = IR_rms_after;
    res.IL_peak_before       = IL_peak_before;
    res.IR_peak_before       = IR_peak_before;
    res.IL_peak_after        = IL_peak_after;
    res.IR_peak_after        = IR_peak_after;
    res.alpha_ss_base        = alpha_ss_base;
    res.alpha_ss_comp        = alpha_ss_comp;
    res.alpha_ss_improve_pct = alpha_ss_improve_pct;
end
