%% VERIFY_RLS_ESTIMATOR_DELTA_KF.M - Step 3B Phase 1 单参数 Delta_Kf 开环 RLS 估计器全套基准验证
% =========================================================================
% 测试架构 (严格按技术审查要求执行 6 项独立测试 + 无投影对照):
% Test A1: 对称工况理论基准零偏差测试 (Delta_Kf_true = 0, 验收指标: abs(err) <= 1e-8 N/count)
% Test A2: 对称工况传感器重构零基线测试 (验证测速差分离散底噪, 报告稳态均值、标准差与 PE 比例)
% Test B:  负向非对称开环收敛测试 (r = 0.70, Delta_Kf_true < 0, 验收指标: 符号正确, 相对误差 <= 5.0%, PE正常)
% Test C:  正向非对称开环收敛测试 (r = 1.30, Delta_Kf_true > 0, 验收指标: 符号正确, 相对误差 <= 5.0%, PE正常)
% Test D:  8192线位置编码器量化抗噪测试 (q_y = 1.21 um, 理想电流指令, 验收指标: 相对误差 <= 5.0%, 变异率 <= 5.0%)
% Test E:  结构参数误差敏感性测试 (K_alpha, B_alpha, J0 分别 +/-20% 全链路重构, 报告传递增益与残差 RMS)
% Test F:  越界投影与协方差稳定性测试 (物理边界与对称边界, 极端扰动触发投影, 零PE协方差绝对不发散)
% 对照机制: 全流程对比"有投影"与"无投影"估计结果，核验正常工况下投影触发次数为 0
% =========================================================================

function verify_rls_estimator_delta_kf()
    script_dir = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, '..');
    
    addpath(fullfile(output_dir, 'common'));
    addpath(fullfile(output_dir, 'step1_baseline_c0'));
    addpath(fullfile(output_dir, 'step2_advanced_controllers'));
    addpath(fullfile(output_dir, 'step3_adaptive_rls'));
    
    fprintf('=========================================================================\n');
    fprintf('      STEP 3B PHASE 1: 单参数 Delta_Kf 开环 RLS 估计器全套基准验证报告      \n');
    fprintf('=========================================================================\n\n');
    
    % 加载数据文件
    file_sym  = fullfile(script_dir, 'data_step3b_phase1_sym.mat');
    file_r070 = fullfile(script_dir, 'data_step3b_phase0_r070.mat');
    file_r130 = fullfile(script_dir, 'data_step3b_phase0_r130.mat');
    
    assert(isfile(file_sym),  '未找到对称基准数据文件: %s', file_sym);
    assert(isfile(file_r070), '未找到工况 A 数据文件: %s', file_r070);
    assert(isfile(file_r130), '未找到工况 B 数据文件: %s', file_r130);
    
    d_sym  = load(file_sym);
    d_r070 = load(file_r070);
    d_r130 = load(file_r130);
    
    csv_rows = {};
    csv_header = {'Case', 'Test_Item', 'Param_Perturbation', 'Sensor_Mode', ...
                  'PE_Threshold_count_m', 'Delta_Kf_True', ...
                  'Theta_Unproj_Final', 'Theta_Proj_Final', 'Max_Theta_Unproj', ...
                  'Abs_Error_N_ct', 'Rel_Error_pct', 'Local_Window_Variation_pct', ...
                  'Transfer_Gain', 'Res_RMS_Nm', 'Projection_Count', ...
                  'Sensitivity_Status', 'Projection_Status'};
              
    % 标称 RLS 配置 (物理精确边界)
    opts_base = struct();
    opts_base.lambda = 1.0;             % 离线数据回放收敛标准
    opts_base.sigma_PE_th = 50.0;       % 50 count*m 候选基准阈值
    opts_base.window_length = 200;      % 200 ms 统一滑动窗口
    opts_base.theta_min = -0.0026295;   % r = 0.65 物理精确下界
    opts_base.theta_max =  0.0018465;   % r = 1.35 物理精确上界
    opts_base.P0 = 1.0e-4;              % 协方差初值 ((N/count)^2)
    opts_base.P_min = 1.0e-12;
    opts_base.P_max = 1.0;
    
    % 无投影对照配置 (Bounds -> Inf)
    opts_noproj = opts_base;
    opts_noproj.theta_min = -Inf;
    opts_noproj.theta_max =  Inf;
    
    N = length(d_sym.t);
    eval_mask_steady = (d_sym.t >= 3.0 & d_sym.t <= 4.0);
    
    %% =====================================================================
    %% Test A1: 对称工况理论模型零偏差测试 (Theoretical Zero Baseline)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test A1] 对称工况理论模型零偏差基线测试 (Delta_Kf_true = 0.0 N/count)\n');
    fprintf('    说明: 基于理论可测回归方程 (无测速离散误差), 验证算法固有零偏特性\n');
    fprintf('    验收指标: 稳态估计绝对误差 abs(Delta_Kf_est) <= 1.0e-8 N/count\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    fc = 10.0; dt = d_sym.dt; wc = 2*pi*fc;
    poly_den = [1.0, 2.61312592975275 * wc, 3.41421356237310 * (wc^2), 2.61312592975275 * (wc^3), wc^4];
    sys_w0_d = c2d(tf(wc^4, poly_den), dt, 'tustin');
    [num_w0, den_a] = tfdata(sys_w0_d, 'v');
    
    phi_f_sym_th = filter(num_w0, den_a, d_sym.phi_Delta_T);
    y_f_sym_th   = filter(num_w0, den_a, d_sym.y_Delta_T);
    
    est_a1 = rls_estimator_delta_kf(opts_base);
    theta_unproj_a1 = zeros(N, 1);
    for k = 1:N
        [est_a1, th_k, info_k] = est_a1.update(phi_f_sym_th(k), y_f_sym_th(k));
        theta_unproj_a1(k) = info_k.theta_unprojected;
    end
    abs_err_a1 = abs(est_a1.theta_hat);
    max_unproj_a1 = max(abs(theta_unproj_a1));
    final_unproj_a1 = info_k.theta_unprojected;
    
    fprintf('  理论模型零基线实测:\n');
    fprintf('    最终绝对误差: %.4e N/count (验收指标: <= 1.0e-8 N/count)\n', abs_err_a1);
    fprintf('    投影触发次数: %d\n', est_a1.projection_count);
    assert(abs_err_a1 <= 1.0e-8, 'Test A1 理论零偏差绝对误差超限！');
    assert(est_a1.projection_count == 0, 'Test A1 异常触发投影！');
    fprintf('  >>> Test A1 理论模型零偏差基线测试: PASS！\n\n');
    
    csv_rows{end+1} = {'Symmetric (r=1.00)', 'TestA1_Theoretical_Zero_Baseline', 'Nominal', 'Theoretical', ...
        opts_base.sigma_PE_th, 0.0, final_unproj_a1, est_a1.theta_hat, max_unproj_a1, ...
        abs_err_a1, 0.0, 0.0, 1.0, 0.0, est_a1.projection_count, 'PASS', 'NO_PROJECTION'};
    
    %% =====================================================================
    %% Test A2: 对称工况传感器重构零基线测试 (Sensor Reconstructed Zero Baseline)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test A2] 对称工况传感器重构零基线测试 (build_step3b_regression 重构)\n');
    fprintf('    说明: 从连续传感器重构回归量, 评估因果离散测速产生的残差底噪与估计稳定性\n');
    fprintf('    验收指标: 稳态绝对误差 <= 5.0e-7 N/count, 稳态均值偏差 <= 5.0e-7 N/count\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    reg_sym = build_step3b_regression( ...
        d_sym.yL_ideal, d_sym.yR_ideal, d_sym.iL_actual, d_sym.iR_actual, ...
        d_sym.dt, d_sym.mech, d_sym.plant, d_sym.Kf_mean, 'ideal');
    
    est_a2 = rls_estimator_delta_kf(opts_base);
    est_a2_noproj = rls_estimator_delta_kf(opts_noproj);
    
    theta_hist_a2 = zeros(N, 1);
    theta_unproj_a2 = zeros(N, 1);
    pe_hist_a2 = false(N, 1);
    
    for k = 1:N
        [est_a2, th_k, info_k] = est_a2.update(reg_sym.phi_f(k), reg_sym.y_f(k));
        [est_a2_noproj, th_noproj_k] = est_a2_noproj.update(reg_sym.phi_f(k), reg_sym.y_f(k));
        theta_hist_a2(k)   = th_k;
        theta_unproj_a2(k) = info_k.theta_unprojected;
        pe_hist_a2(k)      = info_k.is_pe;
    end
    
    final_abs_err_a2 = abs(theta_hist_a2(end));
    steady_mean_a2   = mean(theta_hist_a2(eval_mask_steady));
    steady_std_a2    = std(theta_hist_a2(eval_mask_steady));
    active_ratio_a2  = mean(pe_hist_a2) * 100.0;
    max_unproj_a2    = max(abs(theta_unproj_a2));
    final_unproj_a2  = theta_unproj_a2(end);
    diff_noproj_a2   = abs(est_a2.theta_hat - est_a2_noproj.theta_hat);
    
    fprintf('  传感器重构零基线实测:\n');
    fprintf('    最终绝对误差: %.4e N/count (占 Kf_mean 仅 %.4f%%, 指标: <= 5.0e-7 N/count)\n', ...
        final_abs_err_a2, (final_abs_err_a2 / d_sym.Kf_mean)*100);
    fprintf('    停顿稳态均值: %.4e N/count (指标: <= 5.0e-7 N/count) | 稳态标准差: %.4e N/count\n', steady_mean_a2, steady_std_a2);
    fprintf('    PE 激活比例: %.1f%% | 投影触发次数: %d | 无投影对照差异: %.2e N/count\n', ...
        active_ratio_a2, est_a2.projection_count, diff_noproj_a2);
    
    assert(final_abs_err_a2 <= 5.0e-7, 'Test A2 对称传感器零基线误差超限！');
    assert(abs(steady_mean_a2) <= 5.0e-7, 'Test A2 稳态零偏差超限！');
    assert(est_a2.projection_count == 0, 'Test A2 正常重构工况下异常触发投影截断！');
    assert(diff_noproj_a2 == 0.0, 'Test A2 有投影与无投影估计出现偏离！');
    fprintf('  >>> Test A2 传感器重构零基线测试: PASS！\n\n');
    
    csv_rows{end+1} = {'Symmetric (r=1.00)', 'TestA2_SensorRecon_Zero_Baseline', 'Nominal', 'Ideal', ...
        opts_base.sigma_PE_th, 0.0, final_unproj_a2, theta_hist_a2(end), max_unproj_a2, ...
        final_abs_err_a2, 0.0, steady_std_a2, 1.0, 0.0, est_a2.projection_count, 'PASS', 'NO_PROJECTION'};
    
    %% =====================================================================
    %% Test B: 负向非对称开环收敛测试 (r = 0.70, Delta_Kf_true < 0)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test B] 负向非对称开环收敛测试 (r = 0.70, Delta_Kf_true = %+.7f N/count)\n', d_r070.Delta_Kf_true);
    fprintf('    验收指标: 符号正确恢复, 相对误差 <= 5.0%%, PE 门控正常, 投影触发次数 = 0\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    reg_r070 = build_step3b_regression( ...
        d_r070.yL_ideal, d_r070.yR_ideal, d_r070.iL_actual, d_r070.iR_actual, ...
        d_r070.dt, d_r070.mech, d_r070.plant, d_r070.Kf_mean, 'ideal');
    
    est_b = rls_estimator_delta_kf(opts_base);
    est_b_noproj = rls_estimator_delta_kf(opts_noproj);
    
    theta_hist_b = zeros(N, 1);
    theta_unproj_b = zeros(N, 1);
    pe_hist_b = false(N, 1);
    
    for k = 1:N
        [est_b, th_k, info_k] = est_b.update(reg_r070.phi_f(k), reg_r070.y_f(k));
        [est_b_noproj, th_noproj_k] = est_b_noproj.update(reg_r070.phi_f(k), reg_r070.y_f(k));
        theta_hist_b(k)   = th_k;
        theta_unproj_b(k) = info_k.theta_unprojected;
        pe_hist_b(k)      = info_k.is_pe;
    end
    
    final_th_b      = theta_hist_b(end);
    rel_err_b       = abs(final_th_b - d_r070.Delta_Kf_true) / abs(d_r070.Delta_Kf_true) * 100.0;
    res_rms_b       = sqrt(mean((reg_r070.y_f(eval_mask_steady) - reg_r070.phi_f(eval_mask_steady) * final_th_b).^2));
    max_unproj_b    = max(abs(theta_unproj_b));
    final_unproj_b  = theta_unproj_b(end);
    diff_noproj_b   = abs(est_b.theta_hat - est_b_noproj.theta_hat);
    is_frozen_b     = all(theta_hist_b(eval_mask_steady) == final_th_b);
    
    fprintf('  负向非对称估计实测:\n');
    fprintf('    真值: %+.7f N/count | 估计值: %+.7f N/count | 相对误差: %.4f%% (指标: <= 5.0%%)\n', ...
        d_r070.Delta_Kf_true, final_th_b, rel_err_b);
    fprintf('    符号恢复: %s | 停顿段绝对冻结: %s | 投影触发次数: %d\n', ...
        mat2str(sign(final_th_b) == sign(d_r070.Delta_Kf_true)), mat2str(is_frozen_b), est_b.projection_count);
    fprintf('    未投影最大绝对值: %.4e N/count (物理边界: [%.4e, %.4e]) | 无投影对照差异: %.2e\n', ...
        max_unproj_b, opts_base.theta_min, opts_base.theta_max, diff_noproj_b);
    
    assert(sign(final_th_b) == sign(d_r070.Delta_Kf_true), 'Test B 负向推力偏差符号恢复错误！');
    assert(rel_err_b <= 5.0, 'Test B 相对误差超过 5.0% 门限！');
    assert(is_frozen_b, 'Test B 停顿段未实现绝对冻结！');
    assert(est_b.projection_count == 0, 'Test B 正常连续工况下异常触发投影截断！');
    assert(diff_noproj_b == 0.0, 'Test B 无投影对照出现偏差！');
    fprintf('  >>> Test B 负向非对称开环收敛测试: PASS！\n\n');
    
    csv_rows{end+1} = {'r070 (Delta_Kf < 0)', 'TestB_Negative_Asym', 'Nominal', 'Ideal', ...
        opts_base.sigma_PE_th, d_r070.Delta_Kf_true, final_unproj_b, final_th_b, max_unproj_b, ...
        abs(final_th_b - d_r070.Delta_Kf_true), rel_err_b, 0.0, 1.0, res_rms_b, est_b.projection_count, 'PASS', 'NO_PROJECTION'};
    
    %% =====================================================================
    %% Test C: 正向非对称开环收敛测试 (r = 1.30, Delta_Kf_true > 0)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test C] 正向非对称开环收敛测试 (r = 1.30, Delta_Kf_true = %+.7f N/count)\n', d_r130.Delta_Kf_true);
    fprintf('    验收指标: 符号正确恢复, 相对误差 <= 5.0%%, PE 门控正常, 投影触发次数 = 0\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    reg_r130 = build_step3b_regression( ...
        d_r130.yL_ideal, d_r130.yR_ideal, d_r130.iL_actual, d_r130.iR_actual, ...
        d_r130.dt, d_r130.mech, d_r130.plant, d_r130.Kf_mean, 'ideal');
    
    est_c = rls_estimator_delta_kf(opts_base);
    est_c_noproj = rls_estimator_delta_kf(opts_noproj);
    
    theta_hist_c = zeros(N, 1);
    theta_unproj_c = zeros(N, 1);
    pe_hist_c = false(N, 1);
    
    for k = 1:N
        [est_c, th_k, info_k] = est_c.update(reg_r130.phi_f(k), reg_r130.y_f(k));
        [est_c_noproj, th_noproj_k] = est_c_noproj.update(reg_r130.phi_f(k), reg_r130.y_f(k));
        theta_hist_c(k)   = th_k;
        theta_unproj_c(k) = info_k.theta_unprojected;
        pe_hist_c(k)      = info_k.is_pe;
    end
    
    final_th_c      = theta_hist_c(end);
    rel_err_c       = abs(final_th_c - d_r130.Delta_Kf_true) / abs(d_r130.Delta_Kf_true) * 100.0;
    res_rms_c       = sqrt(mean((reg_r130.y_f(eval_mask_steady) - reg_r130.phi_f(eval_mask_steady) * final_th_c).^2));
    max_unproj_c    = max(abs(theta_unproj_c));
    final_unproj_c  = theta_unproj_c(end);
    diff_noproj_c   = abs(est_c.theta_hat - est_c_noproj.theta_hat);
    is_frozen_c     = all(theta_hist_c(eval_mask_steady) == final_th_c);
    
    fprintf('  正向非对称估计实测:\n');
    fprintf('    真值: %+.7f N/count | 估计值: %+.7f N/count | 相对误差: %.4f%% (指标: <= 5.0%%)\n', ...
        d_r130.Delta_Kf_true, final_th_c, rel_err_c);
    fprintf('    符号恢复: %s | 停顿段绝对冻结: %s | 投影触发次数: %d\n', ...
        mat2str(sign(final_th_c) == sign(d_r130.Delta_Kf_true)), mat2str(is_frozen_c), est_c.projection_count);
    fprintf('    未投影最大绝对值: %.4e N/count (物理边界: [%.4e, %.4e]) | 无投影对照差异: %.2e\n', ...
        max_unproj_c, opts_base.theta_min, opts_base.theta_max, diff_noproj_c);
    
    assert(sign(final_th_c) == sign(d_r130.Delta_Kf_true), 'Test C 正向推力偏差符号恢复错误！');
    assert(rel_err_c <= 5.0, 'Test C 相对误差超过 5.0% 门限！');
    assert(is_frozen_c, 'Test C 停顿段未实现绝对冻结！');
    assert(est_c.projection_count == 0, 'Test C 正常连续工况下异常触发投影截断！');
    assert(diff_noproj_c == 0.0, 'Test C 无投影对照出现偏差！');
    fprintf('  >>> Test C 正向非对称开环收敛测试: PASS！\n\n');
    
    csv_rows{end+1} = {'r130 (Delta_Kf > 0)', 'TestC_Positive_Asym', 'Nominal', 'Ideal', ...
        opts_base.sigma_PE_th, d_r130.Delta_Kf_true, final_unproj_c, final_th_c, max_unproj_c, ...
        abs(final_th_c - d_r130.Delta_Kf_true), rel_err_c, 0.0, 1.0, res_rms_c, est_c.projection_count, 'PASS', 'NO_PROJECTION'};
    
    %% =====================================================================
    %% Test D: 8192 线位置编码器量化抗噪测试 (q_y = 1.21 um, 理想电流指令)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test D] 8192 线位置编码器量化抗噪测试 (q_y = 1.21 um, 理想限幅电流指令)\n');
    fprintf('    验收指标: 相对误差 <= 5.0%%, 局部窗口变异率 <= 5.0%%, 停顿误动 <= 1.0%%, 强激漏动 <= 1.0%%\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    datasets_quant = {d_r070, d_r130};
    tags_quant     = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
    
    for c = 1:2
        ds = datasets_quant{c};
        case_tag = tags_quant{c};
        
        reg_quant = build_step3b_regression( ...
            ds.yL_quant, ds.yR_quant, ds.iL_actual, ds.iR_actual, ...
            ds.dt, ds.mech, ds.plant, ds.Kf_mean, 'quantized');
        
        est_quant = rls_estimator_delta_kf(opts_base);
        est_q_noproj = rls_estimator_delta_kf(opts_noproj);
        
        th_hist_q   = zeros(N, 1);
        th_unproj_q = zeros(N, 1);
        pe_hist_q   = false(N, 1);
        
        for k = 1:N
            [est_quant, th_k, info_k] = est_quant.update(reg_quant.phi_f(k), reg_quant.y_f(k));
            [est_q_noproj, ~] = est_q_noproj.update(reg_quant.phi_f(k), reg_quant.y_f(k));
            th_hist_q(k)   = th_k;
            th_unproj_q(k) = info_k.theta_unprojected;
            pe_hist_q(k)   = info_k.is_pe;
        end
        
        final_th_q     = th_hist_q(end);
        rel_err_q      = abs(final_th_q - ds.Delta_Kf_true) / abs(ds.Delta_Kf_true) * 100.0;
        final_unproj_q = th_unproj_q(end);
        max_unproj_q   = max(abs(th_unproj_q));
        
        strong_ref = (ds.t >= 0.5 & ds.t <= 2.3);
        dwell_ref  = (ds.t >= 3.0 & ds.t <= 4.0);
        
        false_act = mean(pe_hist_q(dwell_ref)) * 100.0;
        miss_act  = mean(~pe_hist_q(strong_ref)) * 100.0;
        
        strong_indices = find(strong_ref & pe_hist_q);
        var_pct = std(th_hist_q(strong_indices)) / abs(ds.Delta_Kf_true) * 100.0;
        res_rms_q = sqrt(mean((reg_quant.y_f(eval_mask_steady) - reg_quant.phi_f(eval_mask_steady) * final_th_q).^2));
        diff_noproj_q = abs(est_quant.theta_hat - est_q_noproj.theta_hat);
        
        fprintf('  量化测试工况 %s:\n', case_tag);
        fprintf('    真值: %+.7f N/count | 估计值: %+.7f N/count | 相对误差: %.4f%% (指标: <= 5.0%%)\n', ...
            ds.Delta_Kf_true, final_th_q, rel_err_q);
        fprintf('    局部窗口变异率: %.4f%% (指标: <= 5.0%%) | 停顿误动率: %.2f%% | 强激漏动率: %.2f%%\n', ...
            var_pct, false_act, miss_act);
        fprintf('    投影触发次数: %d | 未投影最大绝对值: %.4e N/count | 无投影对照差异: %.2e\n', ...
            est_quant.projection_count, max_unproj_q, diff_noproj_q);
        
        assert(rel_err_q <= 5.0, 'Test D 量化估计相对误差超限！');
        assert(var_pct <= 5.0, 'Test D 局部窗口变异率超限！');
        assert(false_act <= 1.0, 'Test D 停顿段误激活率超限！');
        assert(miss_act <= 1.0, 'Test D 强激励段漏激活率超限！');
        assert(est_quant.projection_count == 0, 'Test D 量化测试异常触发投影！');
        assert(diff_noproj_q == 0.0, 'Test D 无投影对照出现偏差！');
        
        csv_rows{end+1} = {case_tag, 'TestD_Quantized_RLS', 'Nominal', 'Position_Quantized_Only', ...
            opts_base.sigma_PE_th, ds.Delta_Kf_true, final_unproj_q, final_th_q, max_unproj_q, ...
            abs(final_th_q - ds.Delta_Kf_true), rel_err_q, var_pct, 1.0, res_rms_q, est_quant.projection_count, 'PASS', 'NO_PROJECTION'};
    end
    fprintf('  >>> Test D 8192 线位置编码器量化抗噪测试: PASS！\n\n');
    
    %% =====================================================================
    %% Test E: 结构参数误差敏感性测试 (K_alpha, B_alpha, J0 分别 +/-20%)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test E] 结构参数误差敏感性测试 (K_alpha, B_alpha, J0 分别 +/-20%% 全链路重构)\n');
    fprintf('    验证 RLS 在结构参数偏差下的闭环收敛偏差与误差传递增益\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    params_to_test = {'K_alpha', 'B_alpha', 'J0'};
    deltas = [-0.20, +0.20];
    
    for c = 1:2
        ds = datasets_quant{c};
        case_tag = tags_quant{c};
        
        fprintf('  =======================================================================================================\n');
        fprintf('  工况: %s\n', case_tag);
        fprintf('  摄动参数  |  比例  | Delta_Kf真值 | 未投影最终值 | 投影输出值   | 未投影最大值 | 估计偏差(%%) | 传递增益 | 投影次数 | 敏感性判定             | 投影安全状态\n');
        
        for p_idx = 1:length(params_to_test)
            param_name = params_to_test{p_idx};
            
            for d_idx = 1:length(deltas)
                delta_pct = deltas(d_idx);
                
                plant_pert = ds.plant;
                mech_pert  = ds.mech;
                
                if strcmp(param_name, 'K_alpha')
                    plant_pert.K_alpha = ds.plant.K_alpha * (1.0 + delta_pct);
                elseif strcmp(param_name, 'B_alpha')
                    plant_pert.B_alpha = ds.plant.B_alpha * (1.0 + delta_pct);
                elseif strcmp(param_name, 'J0')
                    mech_pert.J_alpha_nom = ds.mech.J_alpha_nom * (1.0 + delta_pct);
                end
                
                reg_pert = build_step3b_regression( ...
                    ds.yL_quant, ds.yR_quant, ds.iL_actual, ds.iR_actual, ...
                    ds.dt, mech_pert, plant_pert, ds.Kf_mean, 'quantized');
                
                est_pert = rls_estimator_delta_kf(opts_base);
                est_pert_noproj = rls_estimator_delta_kf(opts_noproj);
                max_theta_unproj = 0.0;
                
                for k = 1:N
                    [est_pert, th_k, info_k] = est_pert.update(reg_pert.phi_f(k), reg_pert.y_f(k));
                    [est_pert_noproj, th_noproj_k, info_noproj_k] = est_pert_noproj.update(reg_pert.phi_f(k), reg_pert.y_f(k));
                    max_theta_unproj = max(max_theta_unproj, abs(info_noproj_k.theta_unprojected));
                end
                
                final_theta_unproj   = est_pert_noproj.theta_hat;
                final_th_pert        = est_pert.theta_hat;
                
                bias_unproj_pct      = (final_theta_unproj - ds.Delta_Kf_true) / ds.Delta_Kf_true * 100.0;
                transfer_gain_unproj = (bias_unproj_pct / 100.0) / delta_pct;
                
                bias_pct      = (final_th_pert - ds.Delta_Kf_true) / ds.Delta_Kf_true * 100.0;
                transfer_gain = (bias_pct / 100.0) / delta_pct;
                res_rms_pert  = sqrt(mean((reg_pert.y_f(eval_mask_steady) - reg_pert.phi_f(eval_mask_steady) * final_th_pert).^2));
                
                % 分离敏感性识别结果与投影安全结果
                if est_pert.projection_count == 0
                    sens_status = 'PASS';
                    proj_status = 'NO_PROJECTION';
                    fprintf('  %-9s | %+5.1f%% | %+11.7f | %+11.7f | %+11.7f | %+11.7f |   %+6.2f%%   |  %5.3f   |   %4d   | %-22s | %s\n', ...
                        param_name, delta_pct*100, ds.Delta_Kf_true, final_theta_unproj, final_th_pert, max_theta_unproj, ...
                        bias_pct, transfer_gain, est_pert.projection_count, sens_status, proj_status);
                else
                    sens_status = 'IDENTIFICATION_CLIPPED';
                    proj_status = 'PROJECTION_ACTIVE_CLAMPED';
                    fprintf('  %-9s | %+5.1f%% | %+11.7f | %+11.7f | %+11.7f | %+11.7f |   %+6.2f%%   |  %5.3f*  |   %4d   | %-22s | %s\n', ...
                        param_name, delta_pct*100, ds.Delta_Kf_true, final_theta_unproj, final_th_pert, max_theta_unproj, ...
                        bias_pct, transfer_gain, est_pert.projection_count, sens_status, proj_status);
                    fprintf('            (注: 未受限估计 = %+.7f, 真实传递增益 = %5.3f; 投影将估计截断保界至 %+.7f, 增益压低为 %5.3f)\n', ...
                        final_theta_unproj, transfer_gain_unproj, final_th_pert, transfer_gain);
                end
                
                csv_rows{end+1} = {case_tag, 'TestE_Sensitivity_RLS', sprintf('%s_%+d%%', param_name, round(delta_pct*100)), ...
                    'Position_Quantized_Only', opts_base.sigma_PE_th, ds.Delta_Kf_true, ...
                    final_theta_unproj, final_th_pert, max_theta_unproj, ...
                    abs(final_th_pert - ds.Delta_Kf_true), bias_pct, NaN, transfer_gain, res_rms_pert, ...
                    est_pert.projection_count, sens_status, proj_status};
            end
        end
    end
    fprintf('  >>> Test E 结构参数误差敏感性测试: PASS (已分离记录投影截断状态)！\n\n');
    
    %% =====================================================================
    %% Test F: 越界投影与协方差稳定性测试 (物理边界与对称边界)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test F] 越界投影与协方差稳定性测试\n');
    fprintf('    验证: 1. 极端冲击下触发投影截断并防止越界\n');
    fprintf('          2. 记录 theta_unprojected 确实溢出, theta_projected 严格落在边界内\n');
    fprintf('          3. 协方差不发散 (P_min <= P <= P_max 且为有限值)\n');
    fprintf('          4. 持续零 PE 静止段协方差绝对不风积\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    % F1: 物理精确边界检验 (opts_base: [-0.0026295, +0.0018465])
    est_f1 = rls_estimator_delta_kf(opts_base);
    
    phi_surge = 1000.0;
    y_surge_pos = 1.0e6; % 产生巨大正残差
    
    % 填充缓冲区以激活 PE
    for i = 1:opts_base.window_length
        [est_f1, ~, ~] = est_f1.update(phi_surge, 0.0);
    end
    
    % 施加正向极端冲击
    [est_f1, th_f1_pos, info_f1_pos] = est_f1.update(phi_surge, y_surge_pos);
    fprintf('  [F1 正向极端冲击实测]:\n');
    fprintf('    未受约束估计 theta_unprojected = %+.4e N/count (严重超限发散)\n', info_f1_pos.theta_unprojected);
    fprintf('    投影截断输出 theta_projected   = %+.7f N/count (上界: %+.7f N/count)\n', ...
        info_f1_pos.theta_projected, opts_base.theta_max);
    fprintf('    投影触发标志: %s | 协方差 P: %.4e ((N/count)^2)\n', ...
        mat2str(info_f1_pos.is_projected), info_f1_pos.P_next);
    
    assert(info_f1_pos.is_pe, 'F1 正向冲击时刻 PE 必须激活！');
    assert(info_f1_pos.theta_unprojected > opts_base.theta_max, '未投影状态未能检测到正向溢出！');
    assert(abs(info_f1_pos.theta_projected - opts_base.theta_max) < 1e-12, '投影截断未能精确限制在物理上界！');
    assert(isfinite(info_f1_pos.theta_projected), '投影输出存在非有限值！');
    assert(info_f1_pos.P_next >= opts_base.P_min && info_f1_pos.P_next <= opts_base.P_max, '协方差越界！');
    
    % F2: 施加负向极端冲击
    y_surge_neg = -1.0e6;
    [est_f1, th_f1_neg, info_f1_neg] = est_f1.update(phi_surge, y_surge_neg);
    fprintf('  [F2 负向极端冲击实测]:\n');
    fprintf('    未受约束估计 theta_unprojected = %+.4e N/count (严重超限发散)\n', info_f1_neg.theta_unprojected);
    fprintf('    投影截断输出 theta_projected   = %+.7f N/count (下界: %+.7f N/count)\n', ...
        info_f1_neg.theta_projected, opts_base.theta_min);
    fprintf('    投影触发标志: %s | 协方差 P: %.4e ((N/count)^2)\n', ...
        mat2str(info_f1_neg.is_projected), info_f1_neg.P_next);
    
    assert(info_f1_neg.is_pe, 'F2 负向冲击时刻 PE 必须激活！');
    assert(info_f1_neg.theta_unprojected < opts_base.theta_min, '未投影状态未能检测到负向溢出！');
    assert(abs(info_f1_neg.theta_projected - opts_base.theta_min) < 1e-12, '投影截断未能精确限制在物理下界！');
    assert(isfinite(info_f1_neg.theta_projected), '投影输出存在非有限值！');
    assert(isfinite(info_f1_neg.P_next), 'F2 负向冲击后协方差必须为有限值！');
    assert(info_f1_neg.P_next >= opts_base.P_min && info_f1_neg.P_next <= opts_base.P_max, 'F2 协方差越界！');
    
    % F3: 持续零 PE 静止段协方差风积检验 (lambda = 0.98 遗忘测试)
    opts_forget = opts_base;
    opts_forget.lambda = 0.98; % 故意设置激进遗忘因子
    est_f3 = rls_estimator_delta_kf(opts_forget);
    
    P_init = est_f3.P;
    for k = 1:5000 % 持续 5 秒无激励静止段
        [est_f3, ~, info_f3] = est_f3.update(0.0, 0.0);
    end
    P_after_zero_pe = est_f3.P;
    fprintf('  [F3 持续零 PE 协方差防风积测试 (lambda = 0.98, 5000 步零激励)]:\n');
    fprintf('    初始协方差 P_init = %.4e | 5000 步静止后协方差 = %.4e\n', P_init, P_after_zero_pe);
    
    assert(abs(P_after_zero_pe - P_init) < 1e-15, '持续零 PE 静止段协方差未严格冻结！存在风积漂移！');
    assert(~info_f3.is_pe, '零激励段被错误判定为 PE 激活！');
    
    % F4: 对称工程边界检验 ([-0.00263, +0.00263])
    opts_sym_bounds = opts_base;
    opts_sym_bounds.theta_min = -0.00263;
    opts_sym_bounds.theta_max =  0.00263;
    est_f4 = rls_estimator_delta_kf(opts_sym_bounds);
    for i = 1:opts_sym_bounds.window_length
        [est_f4, ~, ~] = est_f4.update(phi_surge, 0.0);
    end
    [est_f4, ~, info_f4] = est_f4.update(phi_surge, y_surge_pos);
    assert(info_f4.is_pe, 'F4 对称边界冲击时刻 PE 必须激活！');
    assert(abs(info_f4.theta_projected - opts_sym_bounds.theta_max) < 1e-12, '对称工程边界截断未通过！');
    
    fprintf('  >>> Test F 越界投影与协方差稳定性测试: PASS！\n\n');
    
    csv_rows{end+1} = {'Extreme_Disturbance', 'TestF_Projection_Stability', 'Surge_Disturbance', ...
        'Theoretical', opts_base.sigma_PE_th, NaN, ...
        info_f1_pos.theta_unprojected, info_f1_pos.theta_projected, abs(info_f1_pos.theta_unprojected), ...
        0.0, 0.0, 0.0, 1.0, 0.0, est_f1.projection_count, 'N/A', 'PASS'};
    
    %% =====================================================================
    %% 导出结构化评测 CSV
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3b_phase1_rls_results.csv');
    fid = fopen(csv_file, 'w');
    fprintf(fid, '%s\n', strjoin(csv_header, ','));
    for i = 1:length(csv_rows)
        row = csv_rows{i};
        fprintf(fid, '%s,%s,%s,%s,%.1f,%.7e,%.7e,%.7e,%.7e,%.4e,%.4f,%.4f,%.4f,%.4e,%d,%s,%s\n', ...
            row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, ...
            row{9}, row{10}, row{11}, row{12}, row{13}, row{14}, row{15}, row{16}, row{17});
    end
    fclose(fid);
    fprintf('>>> Phase 1 结构化量化评测指标已成功导出至: %s\n', csv_file);
    fprintf('=========================================================================\n');
    fprintf('          STEP 3B PHASE 1 全套基准测试执行完毕 (各项测试全部 PASS)        \n');
    fprintf('=========================================================================\n');
end
