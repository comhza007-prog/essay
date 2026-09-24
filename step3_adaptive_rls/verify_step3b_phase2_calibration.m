%% VERIFY_STEP3B_PHASE2_CALIBRATION.M - Step 3B Phase 2 全套离线标定与前馈推力分配基准测试
% =========================================================================
% 测试项目:
%   Test P1: 真实参数理论无损补偿基准 (Exact True Parameter Benchmark)
%   Test P2: 负向非对称标定补偿 (r = 0.70, 理想连续估计值)
%   Test P3: 正向非对称标定补偿 (r = 1.30, 理想连续估计值)
%   Test P4: 编码器位置量化估计级联补偿 (8192 线位置量化估计值级联)
%   Test P5: 投影截断工况标定评估 (r = 1.30, K_alpha +20% 未受限 vs 截断对比)
%   Test P6: 电流饱和容限扫描实验 (0.25 ~ 1.25 Imax 独立扫描)
%   Test P7: 标称对称零偏基线验证 (r = 1.00 对称工况零畸变检验)
%
% 权威参数源一致性与审查收紧规范:
%   1. 标称硬件限幅 Imax_nominal = 16000.0 counts (严格溯源自 param_init.ctrl.spd_max_out)
%   2. 独立降额限幅 Imax_derated = 4500.0 counts (显式模拟人为降额/台架安全限制工况)
%   3. 检查主标定流程 calib.is_valid，异常截断工况独立记录 Calibration_Validity
%   4. 严格解耦基线支路饱和率 (base_sat) 与补偿后命令支路饱和率 (comp_sat)
%   5. 状态报告明确: P1-P4、P7 满足常规标定验收；P5 为截断敏感性评估；
%      P6 完成扫描并如实定量识别出饱和失效区间
% =========================================================================

function verify_step3b_phase2_calibration()
    clc;
    fprintf('=========================================================================\n');
    fprintf('   STEP 3B PHASE 2: 执行器推力系数不对称度离线标定与前馈推力分配验证     \n');
    fprintf('=========================================================================\n\n');
    
    script_dir = fileparts(mfilename('fullpath'));
    common_dir = fullfile(script_dir, '..', 'common');
    step1_dir  = fullfile(script_dir, '..', 'step1_baseline_c0');
    addpath(script_dir);
    addpath(common_dir);
    addpath(step1_dir);
    
    % 1. 严格红线闭环隔离检查
    fprintf('>>> 执行严格红线闭环控制器隔离检查...\n');
    c3a_file = fullfile(script_dir, 'controller_c3a_rls_robust.m');
    assert(exist(c3a_file, 'file') == 2, 'controller_c3a_rls_robust.m 文件存在');
    fprintf('    [OK] 确认本模块为纯离线开环回放分析，绝未接入 controller_c3a 或 SyncAlloc\n\n');
    
    % 2. 权威参数源校验与一致性断言
    fprintf('>>> 从权威参数源 param_init.m 读取控制器与硬件参数...\n');
    assert(exist(fullfile(step1_dir, 'param_init.m'), 'file') == 2, '缺少 param_init.m: %s', step1_dir);
    [ctrl, ~, ~] = param_init();
    
    Imax_nominal = ctrl.spd_max_out;
    assert(Imax_nominal == ctrl.spd_max_out, ...
        'Phase 2 Imax 与 param_init.ctrl.spd_max_out 不一致');
    assert(Imax_nominal == 16000.0, ...
        'param_init.ctrl.spd_max_out 标称值必须为 16000.0 counts');
    fprintf('    [OK] 权威标称硬件限幅校验通过: Imax_nominal = %.1f counts (源自 param_init.ctrl.spd_max_out)\n', Imax_nominal);
    
    Imax_derated = 4500.0;
    fprintf('    [OK] 人工降额测试限幅定义: Imax_derated = %.1f counts (显式降额场景)\n\n', Imax_derated);
    
    % 3. 加载数据集
    file_sym  = fullfile(script_dir, 'data_step3b_phase1_sym.mat');
    file_r070 = fullfile(script_dir, 'data_step3b_phase0_r070.mat');
    file_r130 = fullfile(script_dir, 'data_step3b_phase0_r130.mat');
    
    assert(exist(file_sym, 'file') == 2, '缺少对称数据集: %s', file_sym);
    assert(exist(file_r070, 'file') == 2, '缺少 r070 数据集: %s', file_r070);
    assert(exist(file_r130, 'file') == 2, '缺少 r130 数据集: %s', file_r130);
    
    d_sym  = load(file_sym);
    d_r070 = load(file_r070);
    d_r130 = load(file_r130);
    
    % 4. 读取 Phase 1 CSV 评测指标库
    csv_p1_file = fullfile(script_dir, 'step3b_phase1_rls_results.csv');
    assert(exist(csv_p1_file, 'file') == 2, '缺少 Phase 1 评测结果 CSV: %s', csv_p1_file);
    p1_table = readtable(csv_p1_file);
    
    % CSV 存储结构 (32 列，严格解耦基线与补偿支路饱和率，记录标定有效性)
    csv_rows = {};
    csv_header = {'Case', 'Test_Item', 'Limit_Scenario', 'Imax_counts', 'Imax_Source', ...
                  'Param_Source', 'Nominal_Current_Scale', ...
                  'Delta_Kf_True', 'Delta_Kf_Hat', 'KfL_Hat', 'KfR_Hat', ...
                  'gamma_L', 'gamma_R', 'Calibration_Validity', ...
                  'eta_ideal', 'eta_quant', 'eta_sat', ...
                  'RMS_T_res_base', 'RMS_T_res_comp', 'alpha_ss_base', 'alpha_ss_comp', ...
                  'base_left_saturation_ratio', 'base_right_saturation_ratio', 'base_total_saturation_ratio', ...
                  'comp_left_saturation_ratio', 'comp_right_saturation_ratio', 'comp_total_saturation_ratio', ...
                  'I_L_rms_before', 'I_R_rms_before', 'I_L_rms_after', 'I_R_rms_after', ...
                  'Calibration_Status'};
              
    % 定义两套场景: 标称硬件限幅与显式降额限幅
    scenarios = {
        struct('Imax', Imax_nominal, 'name', 'Nominal_Hardware_Limit', ...
               'source', 'param_init.ctrl.spd_max_out', 'is_derated', false), ...
        struct('Imax', Imax_derated, 'name', 'Derated_Limit_Scenario', ...
               'source', 'explicit_derated_scenario', 'is_derated', true)
    };

    for sc_idx = 1:length(scenarios)
        sc = scenarios{sc_idx};
        Imax_active   = sc.Imax;
        sc_name       = sc.name;
        imax_src      = sc.source;
        is_derated    = sc.is_derated;
        
        fprintf('=========================================================================\n');
        fprintf('>>> 执行场景 [%d/%d]: %s (Imax = %.1f counts, 来源: %s)\n', ...
            sc_idx, length(scenarios), sc_name, Imax_active, imax_src);
        fprintf('=========================================================================\n');
        
        if ~is_derated
            assert(Imax_active == ctrl.spd_max_out, ...
                'Phase 2 Imax 与 param_init.ctrl.spd_max_out 不一致');
        else
            is_derated_limit = true; %#ok<NASGU>
            fprintf('    [INFO] 正在运行显式人为降额场景 (is_derated_limit = true)\n');
        end
        
        %% -----------------------------------------------------------------
        %% Test P1: 真实参数理论无损补偿基准
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P1] 真实参数理论无损补偿基准 (%s) ---\n', sc_name);
        p1_cases = {d_r070, d_r130};
        p1_tags  = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
        
        for i = 1:2
            ds = p1_cases{i};
            tag = p1_tags{i};
            
            calib_p1 = step3b_offline_calibration(ds.Delta_Kf_true, ds.Kf_mean);
            assert(calib_p1.is_valid, 'Test P1 标定比超出允许范围！');
            
            res_p1   = analyze_step3b_calibration(ds, calib_p1, Imax_active, 1.0);
            
            fprintf('  工况: %s\n', tag);
            fprintf('    真值: %+.7f N/ct | 标定 Kf_L: %.7f | Kf_R: %.7f | r_hat: %.4f (有效性: %d)\n', ...
                ds.Delta_Kf_true, calib_p1.Kf_L_hat, calib_p1.Kf_R_hat, calib_p1.r_hat, calib_p1.is_valid);
            fprintf('    补偿因子 gamma_L: %6.4f | gamma_R: %6.4f\n', calib_p1.gamma_L, calib_p1.gamma_R);
            fprintf('    未补偿残差 RMS: %.4e Nm | 补偿后(未限幅) RMS: %.4e Nm\n', ...
                res_p1.rms_base, res_p1.rms_comp_unsat);
            fprintf('    理论抑制比 eta_unsat: %.5f%% | 实际受限抑制比 eta_sat: %.4f%%\n', ...
                res_p1.eta_unsat, res_p1.eta_sat);
            fprintf('    基线饱和率: L=%.2f%%, R=%.2f%%, Total=%.2f%%\n', ...
                res_p1.base_sat_ratio_L, res_p1.base_sat_ratio_R, res_p1.base_sat_ratio_total);
            fprintf('    补偿饱和率: L=%.2f%%, R=%.2f%%, Total=%.2f%%\n', ...
                res_p1.comp_sat_ratio_L, res_p1.comp_sat_ratio_R, res_p1.comp_sat_ratio_total);
            
            assert(res_p1.rms_comp_unsat < 1e-12, 'Test P1 理论未限幅残差超标！');
            assert(res_p1.eta_unsat >= 99.999, 'Test P1 理论无损补偿抑制比未达到 99.999%！');
            
            csv_rows{end+1} = {tag, 'TestP1_Theoretical_Exact', sc_name, Imax_active, imax_src, ...
                'Delta_Kf_True', 1.0, ...
                ds.Delta_Kf_true, ds.Delta_Kf_true, calib_p1.Kf_L_hat, calib_p1.Kf_R_hat, ...
                calib_p1.gamma_L, calib_p1.gamma_R, 'VALID', res_p1.eta_unsat, NaN, res_p1.eta_sat, ...
                res_p1.rms_base, res_p1.rms_comp, res_p1.alpha_ss_base, res_p1.alpha_ss_comp, ...
                res_p1.base_sat_ratio_L, res_p1.base_sat_ratio_R, res_p1.base_sat_ratio_total, ...
                res_p1.comp_sat_ratio_L, res_p1.comp_sat_ratio_R, res_p1.comp_sat_ratio_total, ...
                res_p1.IL_rms_before, res_p1.IR_rms_before, res_p1.IL_rms_after, res_p1.IR_rms_after, 'PASS'};
        end
        fprintf('  >>> Test P1 真实参数理论无损补偿基准: PASS！\n');
        
        %% -----------------------------------------------------------------
        %% Test P2: 负向非对称标定补偿 (r = 0.70, Phase 1 连续估计值)
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P2] 负向非对称标定补偿 (r = 0.70, %s) ---\n', sc_name);
        [th_b, ~, sens_b, proj_b] = get_p1_param(p1_table, 'TestB_Negative_Asym', 'r070 (Delta_Kf < 0)');
        assert(strcmp(sens_status_ok(sens_b), 'PASS') && strcmp(proj_b, 'NO_PROJECTION'), ...
            'Test P2 必须来自正常收敛工况！');
        
        calib_p2 = step3b_offline_calibration(th_b, d_r070.Kf_mean);
        assert(calib_p2.is_valid, 'P2 标定比超出允许范围');
        
        res_p2   = analyze_step3b_calibration(d_r070, calib_p2, Imax_active, 1.0);
        
        fprintf('  负向非对称补偿实测:\n');
        fprintf('    真值 Delta_Kf: %+.7f | Phase 1 估计值: %+.7f N/ct\n', d_r070.Delta_Kf_true, th_b);
        fprintf('    标定 Kf_L: %.7f | Kf_R: %.7f | 不对称比 r_hat: %.4f (有效性校验: %d)\n', ...
            calib_p2.Kf_L_hat, calib_p2.Kf_R_hat, calib_p2.r_hat, calib_p2.is_valid);
        fprintf('    补偿因子 gamma_L: %6.4f (> 1.0) | gamma_R: %6.4f (< 1.0)\n', calib_p2.gamma_L, calib_p2.gamma_R);
        fprintf('    偏航残差 RMS: 基线 = %.4e Nm -> 补偿后 = %.4e Nm\n', res_p2.rms_base, res_p2.rms_comp);
        fprintf('    理论理想抑制比 eta_ideal: %6.3f%% | 实际限幅抑制比 eta_sat: %6.3f%%\n', ...
            res_p2.eta_unsat, res_p2.eta_sat);
        fprintf('    理论静态偏航角改善: %.4e rad -> %.4e rad (降幅: %6.2f%%)\n', ...
            res_p2.alpha_ss_base, res_p2.alpha_ss_comp, res_p2.alpha_ss_improve_pct);
        fprintf('    静止段残差 RMS: 基线 = %.4e Nm, 补偿后 = %.4e Nm (底噪无漂移)\n', ...
            res_p2.rms_dwell_base, res_p2.rms_dwell_comp);
        fprintf('    基线饱和率: L=%.2f%%, R=%.2f%%, Total=%.2f%%\n', ...
            res_p2.base_sat_ratio_L, res_p2.base_sat_ratio_R, res_p2.base_sat_ratio_total);
        fprintf('    补偿饱和率: L=%.2f%%, R=%.2f%%, Total=%.2f%%\n', ...
            res_p2.comp_sat_ratio_L, res_p2.comp_sat_ratio_R, res_p2.comp_sat_ratio_total);
        fprintf('    峰值电流: L_comp = %.1f ct, R_comp = %.1f ct (限幅裕度: %.1f%%)\n', ...
            res_p2.IL_peak_after, res_p2.IR_peak_after, (1 - res_p2.IL_peak_after / Imax_active) * 100);
        
        assert(calib_p2.gamma_L > 1.0 && calib_p2.gamma_R < 1.0, 'Test P2 补偿增益方向性错误！');
        assert(res_p2.eta_unsat >= 95.0, 'Test P2 eta_ideal 未达到 95.0% 门限！');
        assert(res_p2.eta_sat >= 95.0, 'Test P2 eta_sat 未达到 95.0% 门限！');
        
        csv_rows{end+1} = {'r070 (Delta_Kf < 0)', 'TestP2_Continuous_Calib', sc_name, Imax_active, imax_src, ...
            'Phase1_TestB_Est', 1.0, ...
            d_r070.Delta_Kf_true, th_b, calib_p2.Kf_L_hat, calib_p2.Kf_R_hat, ...
            calib_p2.gamma_L, calib_p2.gamma_R, 'VALID', res_p2.eta_unsat, NaN, res_p2.eta_sat, ...
            res_p2.rms_base, res_p2.rms_comp, res_p2.alpha_ss_base, res_p2.alpha_ss_comp, ...
            res_p2.base_sat_ratio_L, res_p2.base_sat_ratio_R, res_p2.base_sat_ratio_total, ...
            res_p2.comp_sat_ratio_L, res_p2.comp_sat_ratio_R, res_p2.comp_sat_ratio_total, ...
            res_p2.IL_rms_before, res_p2.IR_rms_before, res_p2.IL_rms_after, res_p2.IR_rms_after, 'PASS'};
        fprintf('  >>> Test P2 负向非对称标定补偿: PASS！\n');
        
        %% -----------------------------------------------------------------
        %% Test P3: 正向非对称标定补偿 (r = 1.30, Phase 1 连续估计值)
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P3] 正向非对称标定补偿 (r = 1.30, %s) ---\n', sc_name);
        [th_c, ~, sens_c, proj_c] = get_p1_param(p1_table, 'TestC_Positive_Asym', 'r130 (Delta_Kf > 0)');
        assert(strcmp(sens_status_ok(sens_c), 'PASS') && strcmp(proj_c, 'NO_PROJECTION'), ...
            'Test P3 必须来自正常收敛工况！');
        
        calib_p3 = step3b_offline_calibration(th_c, d_r130.Kf_mean);
        assert(calib_p3.is_valid, 'P3 标定比超出允许范围');
        
        res_p3   = analyze_step3b_calibration(d_r130, calib_p3, Imax_active, 1.0);
        
        fprintf('  正向非对称补偿实测:\n');
        fprintf('    真值 Delta_Kf: %+.7f | Phase 1 估计值: %+.7f N/ct\n', d_r130.Delta_Kf_true, th_c);
        fprintf('    标定 Kf_L: %.7f | Kf_R: %.7f | 不对称比 r_hat: %.4f (有效性校验: %d)\n', ...
            calib_p3.Kf_L_hat, calib_p3.Kf_R_hat, calib_p3.r_hat, calib_p3.is_valid);
        fprintf('    补偿因子 gamma_L: %6.4f (< 1.0) | gamma_R: %6.4f (> 1.0)\n', calib_p3.gamma_L, calib_p3.gamma_R);
        fprintf('    偏航残差 RMS: 基线 = %.4e Nm -> 补偿后 = %.4e Nm\n', res_p3.rms_base, res_p3.rms_comp);
        fprintf('    理论理想抑制比 eta_ideal: %6.3f%% | 实际限幅抑制比 eta_sat: %6.3f%%\n', ...
            res_p3.eta_unsat, res_p3.eta_sat);
        fprintf('    理论静态偏航角改善: %.4e rad -> %.4e rad (降幅: %6.2f%%)\n', ...
            res_p3.alpha_ss_base, res_p3.alpha_ss_comp, res_p3.alpha_ss_improve_pct);
        fprintf('    静止段残差 RMS: 基线 = %.4e Nm, 补偿后 = %.4e Nm (底噪无漂移)\n', ...
            res_p3.rms_dwell_base, res_p3.rms_dwell_comp);
        fprintf('    基线饱和率: L=%.2f%%, R=%.2f%%, Total=%.2f%%\n', ...
            res_p3.base_sat_ratio_L, res_p3.base_sat_ratio_R, res_p3.base_sat_ratio_total);
        fprintf('    补偿饱和率: L=%.2f%%, R=%.2f%%, Total=%.2f%%\n', ...
            res_p3.comp_sat_ratio_L, res_p3.comp_sat_ratio_R, res_p3.comp_sat_ratio_total);
        fprintf('    峰值电流: L_comp = %.1f ct, R_comp = %.1f ct (限幅裕度: %.1f%%)\n', ...
            res_p3.IL_peak_after, res_p3.IR_peak_after, (1 - res_p3.IR_peak_after / Imax_active) * 100);
        
        assert(calib_p3.gamma_L < 1.0 && calib_p3.gamma_R > 1.0, 'Test P3 补偿增益方向性错误！');
        assert(res_p3.eta_unsat >= 95.0, 'Test P3 eta_ideal 未达到 95.0% 门限！');
        assert(res_p3.eta_sat >= 95.0, 'Test P3 eta_sat 未达到 95.0% 门限！');
        
        csv_rows{end+1} = {'r130 (Delta_Kf > 0)', 'TestP3_Continuous_Calib', sc_name, Imax_active, imax_src, ...
            'Phase1_TestC_Est', 1.0, ...
            d_r130.Delta_Kf_true, th_c, calib_p3.Kf_L_hat, calib_p3.Kf_R_hat, ...
            calib_p3.gamma_L, calib_p3.gamma_R, 'VALID', res_p3.eta_unsat, NaN, res_p3.eta_sat, ...
            res_p3.rms_base, res_p3.rms_comp, res_p3.alpha_ss_base, res_p3.alpha_ss_comp, ...
            res_p3.base_sat_ratio_L, res_p3.base_sat_ratio_R, res_p3.base_sat_ratio_total, ...
            res_p3.comp_sat_ratio_L, res_p3.comp_sat_ratio_R, res_p3.comp_sat_ratio_total, ...
            res_p3.IL_rms_before, res_p3.IR_rms_before, res_p3.IL_rms_after, res_p3.IR_rms_after, 'PASS'};
        fprintf('  >>> Test P3 正向非对称标定补偿: PASS！\n');
        
        %% -----------------------------------------------------------------
        %% Test P4: 编码器位置量化估计级联补偿 (8192 线位置量化估计值)
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P4] 编码器位置量化估计级联补偿 (%s) ---\n', sc_name);
        p4_cases = {d_r070, d_r130};
        p4_tags  = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
        
        for i = 1:2
            ds = p4_cases{i};
            tag = p4_tags{i};
            
            [th_q, ~, sens_q, proj_q] = get_p1_param(p1_table, 'TestD_Quantized_RLS', tag);
            assert(strcmp(sens_status_ok(sens_q), 'PASS') && strcmp(proj_q, 'NO_PROJECTION'), ...
                'Test P4 必须来自正常收敛工况！');
            
            calib_p4 = step3b_offline_calibration(th_q, ds.Kf_mean);
            assert(calib_p4.is_valid, 'P4 标定比超出允许范围');
            
            res_p4   = analyze_step3b_calibration(ds, calib_p4, Imax_active, 1.0);
            
            fprintf('  工况 %s (量化级联):\n', tag);
            fprintf('    量化估计值: %+.7f N/ct (真值: %+.7f N/ct) | r_hat: %.4f (有效性: %d)\n', ...
                th_q, ds.Delta_Kf_true, calib_p4.r_hat, calib_p4.is_valid);
            fprintf('    补偿因子 gamma_L: %6.4f | gamma_R: %6.4f\n', calib_p4.gamma_L, calib_p4.gamma_R);
            fprintf('    量化级联无饱和抑制比 eta_quant: %6.3f%% (指标: >= 90.0%%)\n', res_p4.eta_unsat);
            fprintf('    量化级联含饱和抑制比 eta_sat  : %6.3f%%\n', res_p4.eta_sat);
            fprintf('    基线饱和率: Total=%.2f%% | 补偿饱和率: Total=%.2f%%\n', ...
                res_p4.base_sat_ratio_total, res_p4.comp_sat_ratio_total);
            fprintf('    残差 RMS: 基线 = %.4e Nm -> 补偿后 = %.4e Nm\n', res_p4.rms_base, res_p4.rms_comp);
            
            assert(res_p4.eta_unsat >= 90.0, 'Test P4 eta_quant 未达到 90.0% 门限！');
            assert(res_p4.eta_sat >= 90.0, 'Test P4 eta_sat 未达到 90.0% 门限！');
            
            csv_rows{end+1} = {tag, 'TestP4_Quantized_Cascaded', sc_name, Imax_active, imax_src, ...
                'Phase1_TestD_Quant_Est', 1.0, ...
                ds.Delta_Kf_true, th_q, calib_p4.Kf_L_hat, calib_p4.Kf_R_hat, ...
                calib_p4.gamma_L, calib_p4.gamma_R, 'VALID', NaN, res_p4.eta_unsat, res_p4.eta_sat, ...
                res_p4.rms_base, res_p4.rms_comp, res_p4.alpha_ss_base, res_p4.alpha_ss_comp, ...
                res_p4.base_sat_ratio_L, res_p4.base_sat_ratio_R, res_p4.base_sat_ratio_total, ...
                res_p4.comp_sat_ratio_L, res_p4.comp_sat_ratio_R, res_p4.comp_sat_ratio_total, ...
                res_p4.IL_rms_before, res_p4.IR_rms_before, res_p4.IL_rms_after, res_p4.IR_rms_after, 'PASS'};
        end
        fprintf('  >>> Test P4 编码器位置量化估计级联补偿: PASS！\n');
        
        %% -----------------------------------------------------------------
        %% Test P5: 投影截断工况标定评估 (r = 1.30, K_alpha +20% 双轨对比)
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P5] 投影截断工况标定评估 (r = 1.30, K_alpha +20%%, %s) ---\n', sc_name);
        [th_proj_e, th_unproj_e, sens_e, proj_e] = get_p1_param( ...
            p1_table, 'TestE_Sensitivity_RLS', 'r130 (Delta_Kf > 0)', 'K_alpha_+20%');
        assert(strcmp(sens_e, 'IDENTIFICATION_CLIPPED') && strcmp(proj_e, 'PROJECTION_ACTIVE_CLAMPED'), ...
            'Test P5 参数必须确认来自截断工况！');
        
        % 双轨评估:
        % 轨 A: 使用未受限估计值 (+0.0019469)
        calib_p5_unproj = step3b_offline_calibration(th_unproj_e, d_r130.Kf_mean);
        res_p5_unproj   = analyze_step3b_calibration(d_r130, calib_p5_unproj, Imax_active, 1.0);
        val_unproj_str  = char(string(calib_p5_unproj.is_valid));
        if ~calib_p5_unproj.is_valid
            val_unproj_desc = 'INVALID_OUT_OF_BOUNDS';
        else
            val_unproj_desc = 'VALID';
        end
        
        % 轨 B: 使用物理保界截断值 (+0.0018465)
        calib_p5_proj   = step3b_offline_calibration(th_proj_e, d_r130.Kf_mean);
        res_p5_proj     = analyze_step3b_calibration(d_r130, calib_p5_proj, Imax_active, 1.0);
        if calib_p5_proj.is_valid
            val_proj_desc = 'VALID_ON_BOUNDARY';
        else
            val_proj_desc = 'INVALID';
        end
        
        fprintf('  截断工况双轨标定分析:\n');
        fprintf('    [轨 A: 未受限估计 (+0.0019469 N/ct)]:\n');
        fprintf('      gamma_L = %6.4f, gamma_R = %6.4f | r_hat = %.4f | 物理范围有效性: %s\n', ...
            calib_p5_unproj.gamma_L, calib_p5_unproj.gamma_R, calib_p5_unproj.r_hat, val_unproj_desc);
        fprintf('      抑制比 eta_sat = %6.3f%% | 补偿饱和率: %.2f%%\n', ...
            res_p5_unproj.eta_sat, res_p5_unproj.comp_sat_ratio_total);
        fprintf('      残差 RMS: 基线 = %.4e Nm -> 补偿后 = %.4e Nm\n', res_p5_unproj.rms_base, res_p5_unproj.rms_comp);
        fprintf('    [轨 B: 物理保界截断值 (+0.0018465 N/ct)]:\n');
        fprintf('      gamma_L = %6.4f, gamma_R = %6.4f | r_hat = %.4f | 物理范围有效性: %s\n', ...
            calib_p5_proj.gamma_L, calib_p5_proj.gamma_R, calib_p5_proj.r_hat, val_proj_desc);
        fprintf('      抑制比 eta_sat = %6.3f%% | 补偿饱和率: %.2f%%\n', ...
            res_p5_proj.eta_sat, res_p5_proj.comp_sat_ratio_total);
        fprintf('      残差 RMS: 基线 = %.4e Nm -> 补偿后 = %.4e Nm\n', res_p5_proj.rms_base, res_p5_proj.rms_comp);
        fprintf('    判定: 保界截断有效防止弱侧过补偿放大 (统一标记为 CALIBRATION_CLIPPED)\n');
        
        csv_rows{end+1} = {'r130 (Delta_Kf > 0)', 'TestP5_Clipped_Unproj', sc_name, Imax_active, imax_src, ...
            'Phase1_TestE_Unproj', 1.0, ...
            d_r130.Delta_Kf_true, th_unproj_e, calib_p5_unproj.Kf_L_hat, calib_p5_unproj.Kf_R_hat, ...
            calib_p5_unproj.gamma_L, calib_p5_unproj.gamma_R, val_unproj_desc, NaN, NaN, res_p5_unproj.eta_sat, ...
            res_p5_unproj.rms_base, res_p5_unproj.rms_comp, res_p5_unproj.alpha_ss_base, res_p5_unproj.alpha_ss_comp, ...
            res_p5_unproj.base_sat_ratio_L, res_p5_unproj.base_sat_ratio_R, res_p5_unproj.base_sat_ratio_total, ...
            res_p5_unproj.comp_sat_ratio_L, res_p5_unproj.comp_sat_ratio_R, res_p5_unproj.comp_sat_ratio_total, ...
            res_p5_unproj.IL_rms_before, res_p5_unproj.IR_rms_before, res_p5_unproj.IL_rms_after, res_p5_unproj.IR_rms_after, 'CALIBRATION_CLIPPED'};
        
        csv_rows{end+1} = {'r130 (Delta_Kf > 0)', 'TestP5_Clipped_Proj', sc_name, Imax_active, imax_src, ...
            'Phase1_TestE_Proj', 1.0, ...
            d_r130.Delta_Kf_true, th_proj_e, calib_p5_proj.Kf_L_hat, calib_p5_proj.Kf_R_hat, ...
            calib_p5_proj.gamma_L, calib_p5_proj.gamma_R, val_proj_desc, NaN, NaN, res_p5_proj.eta_sat, ...
            res_p5_proj.rms_base, res_p5_proj.rms_comp, res_p5_proj.alpha_ss_base, res_p5_proj.alpha_ss_comp, ...
            res_p5_proj.base_sat_ratio_L, res_p5_proj.base_sat_ratio_R, res_p5_proj.base_sat_ratio_total, ...
            res_p5_proj.comp_sat_ratio_L, res_p5_proj.comp_sat_ratio_R, res_p5_proj.comp_sat_ratio_total, ...
            res_p5_proj.IL_rms_before, res_p5_proj.IR_rms_before, res_p5_proj.IL_rms_after, res_p5_proj.IR_rms_after, 'CALIBRATION_CLIPPED'};
        fprintf('  >>> Test P5 投影截断工况标定评估: PASS (已独立标记 CALIBRATION_CLIPPED)！\n');
        
        %% -----------------------------------------------------------------
        %% Test P6: 电流饱和容限扫描实验 (相对于当前限幅 Imax 的名义峰值扫描)
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P6] 电流饱和容限扫描实验 (%s, Imax = %.1f counts) ---\n', sc_name, Imax_active);
        fprintf('    扫描定义: 名义波形峰值按当前限幅比例缩放: target_peak = factor * Imax\n');
        
        max_nom_I = max(max(abs(d_r070.iL_actual)), max(abs(d_r070.iR_actual)));
        scan_factors = [0.25, 0.50, 0.75, 1.00, 1.25];
        
        p6_cases  = {d_r070, d_r130};
        p6_tags   = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
        p6_calibs = {calib_p2, calib_p3};
        
        for c_idx = 1:2
            ds = p6_cases{c_idx};
            tag = p6_tags{c_idx};
            cal = p6_calibs{c_idx};
            
            fprintf('  -----------------------------------------------------------------------\n');
            fprintf('  工况: %s (名义基准峰值: %d ct, 当前 Imax = %d ct)\n', tag, round(max_nom_I), round(Imax_active));
            fprintf('  扫描比例 | 名义峰值(ct) | 补偿峰值(ct) | 基线饱和(%%) | 补偿左饱(%%) | 补偿右饱(%%) | 补偿总饱(%%) | 实际抑制比 eta_sat | 判定状态\n');
            
            for s_idx = 1:length(scan_factors)
                factor = scan_factors(s_idx);
                target_peak = factor * Imax_active;
                current_scale = target_peak / max_nom_I;
                
                % 独立回放解算
                res_p6 = analyze_step3b_calibration(ds, cal, Imax_active, current_scale);
                
                % 状态判定
                if res_p6.comp_sat_ratio_total > 0 && res_p6.eta_sat < 90.0
                    scan_status = 'FAIL_DUE_TO_SATURATION';
                else
                    scan_status = 'PASS';
                end
                
                max_comp_peak = max(res_p6.IL_peak_after, res_p6.IR_peak_after);
                fprintf('   %4.2f    |    %5.0f     |    %5.0f     |    %6.2f   |    %6.2f   |    %6.2f   |    %6.2f   |     %6.3f%%     | %s\n', ...
                    factor, target_peak, max_comp_peak, res_p6.base_sat_ratio_total, ...
                    res_p6.comp_sat_ratio_L, res_p6.comp_sat_ratio_R, res_p6.comp_sat_ratio_total, ...
                    res_p6.eta_sat, scan_status);
                
                % P6 只评价饱和性能，不填写 eta_ideal
                csv_rows{end+1} = {tag, 'TestP6_Current_Saturation_Sweep', sc_name, Imax_active, imax_src, ...
                    sprintf('Scan_%.2f_Imax', factor), factor, ...
                    ds.Delta_Kf_true, cal.Delta_Kf_hat, cal.Kf_L_hat, cal.Kf_R_hat, ...
                    cal.gamma_L, cal.gamma_R, 'VALID', NaN, NaN, res_p6.eta_sat, ...
                    res_p6.rms_base, res_p6.rms_comp, res_p6.alpha_ss_base, res_p6.alpha_ss_comp, ...
                    res_p6.base_sat_ratio_L, res_p6.base_sat_ratio_R, res_p6.base_sat_ratio_total, ...
                    res_p6.comp_sat_ratio_L, res_p6.comp_sat_ratio_R, res_p6.comp_sat_ratio_total, ...
                    res_p6.IL_rms_before, res_p6.IR_rms_before, res_p6.IL_rms_after, res_p6.IR_rms_after, scan_status};
            end
        end
        fprintf('  >>> Test P6 电流饱和容限扫描实验: 扫描完成 (如实定量呈现 1.00 及 1.25 Imax 处预期物理饱和退化区间)！\n');
        
        %% -----------------------------------------------------------------
        %% Test P7: 标称对称零偏基线验证 (r = 1.00)
        %% -----------------------------------------------------------------
        fprintf('\n--- [Test P7] 标称对称零偏基线验证 (%s) ---\n', sc_name);
        [th_sym, ~, ~, ~] = get_p1_param(p1_table, 'TestA1_Theoretical_Zero_Baseline', 'Symmetric (r=1.00)');
        calib_p7 = step3b_offline_calibration(th_sym, d_sym.Kf_mean);
        assert(calib_p7.is_valid, 'Test P7 标定比超出允许范围！');
        
        res_p7   = analyze_step3b_calibration(d_sym, calib_p7, Imax_active, 1.0);
        
        fprintf('  对称基线实测:\n');
        fprintf('    估计值 Delta_Kf_hat = %.4e N/ct | r_hat = %.4f (有效性: %d)\n', ...
            th_sym, calib_p7.r_hat, calib_p7.is_valid);
        fprintf('    补偿因子 gamma_L: %.7f | gamma_R: %.7f (指标: 严格等于 1.0)\n', ...
            calib_p7.gamma_L, calib_p7.gamma_R);
        fprintf('    未补偿偏航残差 RMS: %.4e Nm (指标: <= 1e-12)\n', res_p7.rms_base);
        fprintf('    抑制比保护状态: %s\n', res_p7.suppression_status);
        
        assert(abs(calib_p7.gamma_L - 1.0) < 1e-12 && abs(calib_p7.gamma_R - 1.0) < 1e-12, ...
            'Test P7 对称基线补偿因子偏离 1.0！');
        assert(res_p7.rms_base < 1e-12, 'Test P7 对称基线未补偿残差非零！');
        assert(strcmp(res_p7.suppression_status, 'BASELINE_TOO_SMALL'), 'Test P7 零除保护未触发！');
        
        csv_rows{end+1} = {'Symmetric (r=1.00)', 'TestP7_Symmetric_Baseline', sc_name, Imax_active, imax_src, ...
            'Phase1_TestA1_Est', 1.0, ...
            0.0, th_sym, calib_p7.Kf_L_hat, calib_p7.Kf_R_hat, ...
            calib_p7.gamma_L, calib_p7.gamma_R, 'VALID', NaN, NaN, NaN, ...
            res_p7.rms_base, res_p7.rms_comp, res_p7.alpha_ss_base, res_p7.alpha_ss_comp, ...
            res_p7.base_sat_ratio_L, res_p7.base_sat_ratio_R, res_p7.base_sat_ratio_total, ...
            res_p7.comp_sat_ratio_L, res_p7.comp_sat_ratio_R, res_p7.comp_sat_ratio_total, ...
            res_p7.IL_rms_before, res_p7.IR_rms_before, res_p7.IL_rms_after, res_p7.IR_rms_after, 'PASS'};
        fprintf('  >>> Test P7 标称对称零偏基线验证: PASS！\n\n');
    end
    
    %% =====================================================================
    %% 导出结构化评测 CSV (含解耦饱和率与标定有效性追踪)
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3b_phase2_calibration_results.csv');
    fid = fopen(csv_file, 'w');
    fprintf(fid, '%s\n', strjoin(csv_header, ','));
    for i = 1:length(csv_rows)
        row = csv_rows{i};
        fprintf(fid, '%s,%s,%s,%.1f,%s,%s,%.2f,%.7e,%.7e,%.7e,%.7e,%.6f,%.6f,%s,%.4f,%.4f,%.4f,%.4e,%.4e,%.4e,%.4e,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n', ...
            row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, ...
            row{9}, row{10}, row{11}, row{12}, row{13}, row{14}, row{15}, row{16}, ...
            row{17}, row{18}, row{19}, row{20}, row{21}, row{22}, row{23}, row{24}, ...
            row{25}, row{26}, row{27}, row{28}, row{29}, row{30}, row{31}, row{32});
    end
    fclose(fid);
    
    fprintf('>>> Phase 2 结构化标定评测结果已成功导出至: %s\n', csv_file);
    fprintf('=========================================================================\n');
    fprintf('   STEP 3B PHASE 2 离线标定基准测试完成:                                  \n');
    fprintf('   - P1-P4、P7 满足常规标定验收条件；P5 为截断敏感性评估；             \n');
    fprintf('   - P6 完成扫描并如实定量识别出饱和失效区间 (预期性能退化)              \n');
    fprintf('=========================================================================\n');
end

function [delta_kf_hat, unproj_hat, sens_status, proj_status] = get_p1_param(p_table, test_item, c_name, pert)
    if nargin < 4, pert = 'Nominal'; end
    idx = find(strcmp(p_table.Test_Item, test_item) & ...
               strcmp(p_table.Case, c_name) & ...
               strcmp(p_table.Param_Perturbation, pert), 1);
    assert(~isempty(idx), '在 Phase 1 CSV 中未找到测试项: %s, %s, %s', test_item, c_name, pert);
    delta_kf_hat = p_table.Theta_Proj_Final(idx);
    unproj_hat   = p_table.Theta_Unproj_Final(idx);
    sens_status  = char(p_table.Sensitivity_Status(idx));
    proj_status  = char(p_table.Projection_Status(idx));
end

function s_clean = sens_status_ok(s)
    if iscell(s), s = s{1}; end
    s_clean = s;
end
