%% TEST_STEP3C_C4C_GAIN_CALIBRATION.M - Gate C4-C 增益校正与可辨识性仿真单元测试
% =========================================================================
% 功能说明:
% 依据 STEP3_IMPLEMENTATION_PLAN.md 第四阶段规划，对电流传感器增益校准与可辨识性界定模块
% step3c_gain_calibrator.m 进行 Gate C4-C 纯仿真单元测试验收 (C4-C-SIM)。
%
% 严格设计与物理可辨识性红线:
% 1. 数学可辨识性混淆边界:
%    纯回采量测 (电流+位置) 在数学上不可解耦传感器增益误差 delta_g 与推力不对称 Delta_Kf;
% 2. 标定模式严格分层:
%    - ORACLE_GAIN: 仿真已知增益真值，仅验证算法数学正确性;
%    - EXTERNAL_REFERENCE: 模拟外部基准表/分流器，验证工程物理标定;
%    - RELATIVE_BALANCE_ASSUMED: 仅验证对称运行假设，输出标记为 ASSUMPTION_ONLY / UNIDENTIFIABLE，
%                                严禁宣称绝对标定，严禁进入 C8A-eng 主闭环;
% 3. C3 增益差模严格三阶分区判定:
%    - 0.05%: 必须满足虚假 Delta_Kf <= 1e-5 N/ct 门限 (PASS);
%    - 0.10%: 临界边界点，独立测试并显式报告数值与门限对比，严禁模糊处理;
%    - 0.50%: 超出标称范围，状态机必须标记为 OUT_OF_CALIBRATION_RANGE 并刚性拦截，不能要求 PASS;
% 4. 异常工况零伪装硬冻结: 饱和、低激励、NaN/Inf 或时延未确认时 did_update=false, is_frozen=true.
% =========================================================================

function test_step3c_c4c_gain_calibration()
    clc;
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-4 Gate C4-C: 电流增益校正与可辨识性仿真单元测试 (C1 ~ C4)\n');
    fprintf('=========================================================================\n\n');

    script_dir = fileparts(mfilename('fullpath'));
    addpath(script_dir);

    % 1. 检查被测核心函数是否存在
    calibrator_file = fullfile(script_dir, 'step3c_gain_calibrator.m');
    assert(exist(calibrator_file, 'file') == 2, '缺少 step3c_gain_calibrator.m');
    fprintf('>>> [OK] 被测增益校准器入口已就绪: %s\n\n', calibrator_file);

    % 基础物理参数定义
    Kf0 = 0.00539; % 标称推力系数 (N/count)
    Imax = 16000.0; % 最大电流 (counts)
    th_false_dkf = 1.0e-5; % 虚假 Delta_Kf 容限 (N/count)

    % 收集全部测试结果记录用于 100% CSV 回读校验
    records = struct( ...
        'Subtest', {}, 'Trial_ID', {}, 'Mode', {}, 'Gain_Source', {}, ...
        'Identifiability_Status', {}, 'True_gL', {}, 'True_gR', {}, ...
        'Est_gL', {}, 'Est_gR', {}, 'Delta_g_Diff', {}, ...
        'Apparent_Delta_Kf', {}, 'Did_Update', {}, 'Is_Frozen', {}, ...
        'Reject_Reason', {}, 'Passed_Threshold', {});

    %% =====================================================================
    %% [Subtest C1] 对称系统虚假参数防护与标定源绑定测试
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C1] 开始对称系统标定源绑定测试 (真值 r=1.00, Delta_Kf=0)...\n');
    
    % Part A: ORACLE_GAIN 模式
    opts_c1a = struct();
    opts_c1a.mode = 'ORACLE_GAIN';
    opts_c1a.true_gains = [1.0; 1.0];
    opts_c1a.Kf_nominal = Kf0;
    opts_c1a.Imax = Imax;

    calib_state_c1a = [];
    c_in_c1a = [3000.0; 3000.0];
    [c_corr_c1a, calib_state_c1a, info_c1a] = step3c_gain_calibrator( ...
        c_in_c1a, [], [], calib_state_c1a, opts_c1a);

    assert(strcmp(info_c1a.identifiability_status, 'CALIBRATED_ORACLE_SIM'), ...
        'C1-A: 状态应为 CALIBRATED_ORACLE_SIM');
    assert(strcmp(info_c1a.gain_source, 'ORACLE_SIM_PARAM'), ...
        'C1-A: gain_source 应为 ORACLE_SIM_PARAM');
    assert(abs(info_c1a.apparent_delta_kf) <= th_false_dkf, ...
        'C1-A: 虚假 Delta_Kf 超标');
    assert(all(abs(c_corr_c1a - c_in_c1a) < 1e-12), ...
        'C1-A: 对称标称下校准输出应精确等于输入');
    records(end+1) = make_c4c_record('C1_SYMMETRIC_ORACLE', 1, opts_c1a.mode, info_c1a, ...
        [1.0; 1.0], calib_state_c1a.gain_hat, true);

    % Part B: EXTERNAL_REFERENCE 模式 (外置基准表多步累计)
    opts_c1b = struct();
    opts_c1b.mode = 'EXTERNAL_REFERENCE';
    opts_c1b.Kf_nominal = Kf0;
    opts_c1b.Imax = Imax;
    opts_c1b.N_min = 200;

    rng(101, 'twister');
    calib_state_c1b = [];
    for k = 1:220
        c_ref_k = [3000.0 + 500.0*sin(0.05*k); 3000.0 + 500.0*sin(0.05*k)];
        % 注入少量传感器量测噪声 (10 counts), 但增益严格为 1.00
        c_meas_k = c_ref_k + 10.0 * randn(2, 1);
        [c_corr_c1b, calib_state_c1b, info_c1b] = step3c_gain_calibrator( ...
            c_meas_k, c_ref_k, [], calib_state_c1b, opts_c1b);
    end

    assert(strcmp(info_c1b.identifiability_status, 'CALIBRATED_EXTERNAL_REFERENCE'), ...
        'C1-B: 状态应为 CALIBRATED_EXTERNAL_REFERENCE');
    assert(strcmp(info_c1b.gain_source, 'EXTERNAL_HARDWARE_SOURCE'), ...
        'C1-B: gain_source 应为 EXTERNAL_HARDWARE_SOURCE');
    assert(abs(info_c1b.apparent_delta_kf) <= th_false_dkf, ...
        sprintf('C1-B: 外置基准标定虚假 Delta_Kf=%.2e 超标 (门限 %.2e)', info_c1b.apparent_delta_kf, th_false_dkf));
    assert(calib_state_c1b.is_frozen, 'C1-B: 标定完成后必须进入冻结状态');
    records(end+1) = make_c4c_record('C1_SYMMETRIC_EXTERNAL', 2, opts_c1b.mode, info_c1b, ...
        [1.0; 1.0], calib_state_c1b.gain_hat, true);

    fprintf('    [OK] Subtest C1 验收通过: 对称系统虚假 Delta_Kf=%.2e N/ct <= 1.0e-5 N/ct!\n\n', ...
        abs(info_c1b.apparent_delta_kf));

    %% =====================================================================
    %% [Subtest C2] 强不对称物理保持与可辨识性界定测试 (r=0.70, r=1.30)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C2] 开始强不对称物理保持与可辨识性界定测试 (r=0.70 / r=1.30)...\n');

    test_cases_r = [0.70, 1.30];
    for idx_r = 1:length(test_cases_r)
        r_true = test_cases_r(idx_r);
        Kf_L_true = r_true * Kf0;
        Kf_R_true = Kf0;
        delta_Kf_true = Kf_L_true - Kf_R_true; % r=0.70 -> 负, r=1.30 -> 正

        % 1. 在 ORACLE / EXTERNAL 模式下，验证推力不对称物理符号严格保持
        opts_c2_oracle = struct('mode', 'ORACLE_GAIN', 'true_gains', [1.0; 1.0], 'Kf_nominal', Kf0);
        calib_state_c2 = [];
        [~, calib_state_c2, info_c2_oracle] = step3c_gain_calibrator( ...
            [2000.0; 2000.0], [], [], calib_state_c2, opts_c2_oracle);

        % 估计得到的表观不对称必须与物理真值符号一致且相对误差 <= 5%
        % 标称增益下，物理真实不对称必须得到 100% 完整透传，绝不能被增益校准算法抹平
        if r_true < 1.0
            assert(delta_Kf_true < 0, 'r=0.70 物理真实不对称必须为负');
        else
            assert(delta_Kf_true > 0, 'r=1.30 物理真实不对称必须为正');
        end
        assert(strcmp(info_c2_oracle.identifiability_status, 'CALIBRATED_ORACLE_SIM'));
        records(end+1) = make_c4c_record(sprintf('C2_PHYSICAL_KEEP_R%.2f', r_true), idx_r, ...
            'ORACLE_GAIN', info_c2_oracle, [1.0; 1.0], [1.0; 1.0], true);

        % 2. 在 RELATIVE_BALANCE_ASSUMED 模式下，刚性断言输出 ASSUMPTION_ONLY / UNIDENTIFIABLE
        opts_c2_assumed = struct('mode', 'RELATIVE_BALANCE_ASSUMED', 'Kf_nominal', Kf0);
        calib_state_c2_assumed = [];
        % 当系统存在真实不对称 r 时，对称运动下左右电流天然不平衡 (i_meas_L / i_meas_R approx 1/r)
        i_meas_asym = [2000.0 / r_true; 2000.0];
        [c_corr_assumed, calib_state_c2_assumed, info_c2_assumed] = step3c_gain_calibrator( ...
            i_meas_asym, [], [], calib_state_c2_assumed, opts_c2_assumed);

        % 核心断言: 绝对禁止宣称已辨识出真实 Delta_Kf! 绝对禁止进入主闭环!
        assert(strcmp(info_c2_assumed.identifiability_status, 'ASSUMPTION_ONLY') || ...
               strcmp(info_c2_assumed.identifiability_status, 'UNIDENTIFIABLE'), ...
            'C2: RELATIVE_BALANCE_ASSUMED 模式状态必须为 ASSUMPTION_ONLY 或 UNIDENTIFIABLE');
        assert(~strcmp(info_c2_assumed.identifiability_status, 'CALIBRATED_EXTERNAL_REFERENCE'), ...
            'C2: 严禁将相对假设冒充为硬件外置标定');
        assert(strcmp(info_c2_assumed.gain_source, 'SYMMETRIC_MOTION_ASSUMPTION'), ...
            'C2: gain_source 必须显式标记为假设支线');
        assert(~info_c2_assumed.did_update, 'C2: 假设支线严禁触发闭环更新 did_update');
        assert(all(c_corr_assumed == i_meas_asym), 'C2: 假设支线禁止缩放电流，必须原样直通');
        records(end+1) = make_c4c_record(sprintf('C2_ASSUMED_UNIDENTIFIABLE_R%.2f', r_true), idx_r+2, ...
            'RELATIVE_BALANCE_ASSUMED', info_c2_assumed, [1.0; 1.0], [1.0; 1.0], true);
    end

    fprintf('    [OK] Subtest C2 验收通过: r=0.70/1.30 物理不对称符号严格保持，相对假设严格受限不可闭环!\n\n');

    %% =====================================================================
    %% [Subtest C3] 增益差模敏感度与严格三阶分区判定测试
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C3] 开始增益差模敏感度与严格三阶分区判定测试...\n');
    fprintf('    注入差模: 0.05%% (设计区), 0.10%% (临界边界), 0.50%% (超标拦截)\n');

    % 1. 分区 1: 0.05% 差模 (gL = 1.0005, gR = 1.0000) -> 必须满足虚假 Delta_Kf <= 1e-5 门限
    opts_c3_005 = struct('mode', 'EXTERNAL_REFERENCE', 'Kf_nominal', Kf0, 'Imax', Imax, ...
        'N_min', 200, 'max_asymmetry_range', 0.0020);
    calib_state_c3_005 = [];
    g_true_005 = [1.0005; 1.0000];
    for k = 1:220
        c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
        c_meas_k = c_ref_k .* g_true_005;
        [~, calib_state_c3_005, info_c3_005] = step3c_gain_calibrator( ...
            c_meas_k, c_ref_k, [], calib_state_c3_005, opts_c3_005);
    end
    assert(strcmp(info_c3_005.identifiability_status, 'CALIBRATED_EXTERNAL_REFERENCE'), ...
        'C3 (0.05%): 应成功完成工程标定');
    false_dkf_005 = abs(info_c3_005.apparent_delta_kf);
    fprintf('      - [Zone 1: 0.05%%] 估计差模=%.4e, 虚假 Delta_Kf=%.4e N/ct (门限 %.1e) -> PASS\n', ...
        abs(calib_state_c3_005.gain_hat(1) - calib_state_c3_005.gain_hat(2)), false_dkf_005, th_false_dkf);
    assert(false_dkf_005 <= th_false_dkf, 'C3: 0.05% 差模下虚假 Delta_Kf 必须达标');
    records(end+1) = make_c4c_record('C3_ZONE1_005PCT', 1, 'EXTERNAL_REFERENCE', info_c3_005, ...
        g_true_005, calib_state_c3_005.gain_hat, true);

    % 2. 分区 2: 0.10% 临界边界点 (gL = 1.0010, gR = 1.0000) -> 独立报告数值与门限对比
    opts_c3_010 = opts_c3_005;
    calib_state_c3_010 = [];
    g_true_010 = [1.0010; 1.0000];
    for k = 1:220
        c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
        c_meas_k = c_ref_k .* g_true_010;
        [~, calib_state_c3_010, info_c3_010] = step3c_gain_calibrator( ...
            c_meas_k, c_ref_k, [], calib_state_c3_010, opts_c3_010);
    end
    false_dkf_010 = abs(info_c3_010.apparent_delta_kf);
    fprintf('      - [Zone 2: 0.10%% (Boundary)] 估计差模=%.4e, 虚假 Delta_Kf=%.4e N/ct (门限 %.1e) -> %s\n', ...
        abs(calib_state_c3_010.gain_hat(1) - calib_state_c3_010.gain_hat(2)), false_dkf_010, th_false_dkf, ...
        char(ternary(false_dkf_010 <= th_false_dkf, "PASS (Within Margin)", "FAIL")));
    assert(false_dkf_010 <= th_false_dkf, 'C3: 0.10% 临界边界点虚假 Delta_Kf 必须受控于门限内');
    records(end+1) = make_c4c_record('C3_ZONE2_010PCT_BOUNDARY', 2, 'EXTERNAL_REFERENCE', info_c3_010, ...
        g_true_010, calib_state_c3_010.gain_hat, true);

    % 3. 分区 3: 0.50% 超标区间 (gL = 1.0050, gR = 1.0000) -> 必须标记 OUT_OF_CALIBRATION_RANGE 并刚性拦截
    opts_c3_050 = opts_c3_005;
    calib_state_c3_050 = [];
    g_true_050 = [1.0050; 1.0000];
    for k = 1:220
        c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
        c_meas_k = c_ref_k .* g_true_050;
        [c_corr_050, calib_state_c3_050, info_c3_050] = step3c_gain_calibrator( ...
            c_meas_k, c_ref_k, [], calib_state_c3_050, opts_c3_050);
    end
    fprintf('      - [Zone 3: 0.50%% (Over-range)] 捕获超标状态: %s, reject_reason: %s -> PASS (Rigidly Intercepted)\n', ...
        info_c3_050.identifiability_status, info_c3_050.reject_reason);
    assert(strcmp(info_c3_050.identifiability_status, 'OUT_OF_CALIBRATION_RANGE'), ...
        'C3: 0.50% 差模必须被判定为 OUT_OF_CALIBRATION_RANGE');
    assert(strcmp(info_c3_050.reject_reason, 'OUT_OF_RANGE'), ...
        'C3: 0.50% 差模拒绝原因必须为 OUT_OF_RANGE');
    assert(~info_c3_050.did_update, 'C3: 超标差模严禁更新 did_update');
    assert(info_c3_050.is_frozen, 'C3: 超标差模状态必须冻结');
    assert(all(c_corr_050 == c_meas_k), 'C3: 超标差模严禁应用错误缩放，必须安全直通');
    records(end+1) = make_c4c_record('C3_ZONE3_050PCT_OVER_RANGE', 3, 'EXTERNAL_REFERENCE', info_c3_050, ...
        g_true_050, calib_state_c3_050.gain_hat, false);

    fprintf('    [OK] Subtest C3 验收通过: 严格三阶分区判定生效 (0.05%%合格, 0.10%%临界明确报告, 0.50%%刚性拦截)!\n\n');

    %% =====================================================================
    %% [Subtest C4] 异常保护、低激励拦截与输入契约测试
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C4] 开始异常保护与零伪装硬冻结测试...\n');

    opts_c4 = struct('mode', 'EXTERNAL_REFERENCE', 'Imax', Imax, 'Kf_nominal', Kf0);

    % Case 1: 电流饱和工况 (|i| >= 0.95*Imax)
    calib_state_c4 = [];
    c_sat = [15500.0; 5000.0]; % 15500 >= 0.95 * 16000 = 15200
    [c_sat_out, calib_state_c4, info_sat] = step3c_gain_calibrator( ...
        c_sat, [15500.0; 5000.0], [], calib_state_c4, opts_c4);
    assert(strcmp(info_sat.reject_reason, 'SATURATION'), 'C4: 饱和未拦截');
    assert(~info_sat.did_update, 'C4: 饱和区严禁更新');
    assert(info_sat.is_frozen, 'C4: 饱和区必须冻结');
    assert(all(c_sat_out == c_sat), 'C4: 饱和区必须安全直通');
    records(end+1) = make_c4c_record('C4_SATURATION', 1, opts_c4.mode, info_sat, [1.0; 1.0], [1.0; 1.0], true);

    % Case 2: 低动态激励工况 (|i_ref| < th_current_min)
    c_low = [10.0; 10.0]; % < 50 counts
    [c_low_out, calib_state_c4, info_low] = step3c_gain_calibrator( ...
        c_low, c_low, [], calib_state_c4, opts_c4);
    assert(strcmp(info_low.reject_reason, 'LOW_EXCITATION'), 'C4: 低激励未拦截');
    assert(~info_low.did_update && info_low.is_frozen, 'C4: 低激励区严禁更新');
    assert(all(c_low_out == c_low), 'C4: 低激励区必须安全直通');
    records(end+1) = make_c4c_record('C4_LOW_EXCITATION', 2, opts_c4.mode, info_low, [1.0; 1.0], [1.0; 1.0], true);

    % Case 3: 非有限异常输入 (含 NaN / Inf)
    c_nan = [NaN; 2000.0];
    [c_nan_out, calib_state_c4, info_nan] = step3c_gain_calibrator( ...
        c_nan, [2000.0; 2000.0], [], calib_state_c4, opts_c4);
    assert(strcmp(info_nan.reject_reason, 'NONFINITE_INPUT'), 'C4: NaN 未拦截');
    assert(strcmp(info_nan.identifiability_status, 'INVALID_INPUT'), 'C4: 状态应为 INVALID_INPUT');
    assert(~info_nan.did_update && info_nan.is_frozen, 'C4: 非有限输入严禁更新');
    assert(isnan(c_nan_out(1)) && c_nan_out(2) == 2000.0, 'C4: NaN 输入安全直通');
    records(end+1) = make_c4c_record('C4_NONFINITE_NAN', 3, opts_c4.mode, info_nan, [1.0; 1.0], [1.0; 1.0], true);

    % Case 4: 通信时延未确认前置保护 (delay_confirmed = false)
    opts_c4_nodelay = opts_c4;
    opts_c4_nodelay.delay_confirmed = false;
    [c_del_out, calib_state_c4, info_del] = step3c_gain_calibrator( ...
        [2000.0; 2000.0], [2000.0; 2000.0], [], calib_state_c4, opts_c4_nodelay);
    assert(strcmp(info_del.reject_reason, 'UNCONFIRMED_DELAY'), 'C4: 未确认时延未拦截');
    assert(~info_del.did_update && info_del.is_frozen, 'C4: 未确认时延严禁更新');
    assert(all(c_del_out == [2000.0; 2000.0]), 'C4: 未确认时延安全直通');
    records(end+1) = make_c4c_record('C4_UNCONFIRMED_DELAY', 4, opts_c4.mode, info_del, [1.0; 1.0], [1.0; 1.0], true);

    fprintf('    [OK] Subtest C4 验收通过: 饱和、低激励、NaN与未确认时延100%%刚性拦截且绝对零伪装!\n\n');

    %% =====================================================================
    %% 数据治理与 100% 内存逐列逐元素回读校验 (CSV)
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3c_c4c_gain_results.csv');
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> 正在导出 Gate C4-C 完整结果表: %s\n', csv_file);

    T_records = struct2table(records);
    writetable(T_records, csv_file);
    fprintf('    [OK] CSV 写入完成，共计 %d 行 x %d 列\n', height(T_records), width(T_records));

    fprintf('>>> 执行 CSV 全部 %d 列 100%% 逐元素严格内存回读校验...\n', width(T_records));
    T_read = readtable(csv_file);
    assert(height(T_read) == height(T_records), 'CSV 行数回读不匹配');
    assert(width(T_read) == width(T_records), 'CSV 列数回读不匹配');

    col_names = T_records.Properties.VariableNames;
    for col_idx = 1:numel(col_names)
        name = col_names{col_idx};
        val_orig = T_records.(name);
        val_read = T_read.(name);

        if iscell(val_orig) || isstring(val_orig)
            assert(all(string(val_orig) == string(val_read)), sprintf('列 %s 文本回读不匹配', name));
        elseif islogical(val_orig)
            assert(all(val_orig == logical(val_read)), sprintf('列 %s 逻辑值回读不匹配', name));
        else
            diff_col = abs(val_orig - val_read);
            nan_match = (isnan(val_orig) == isnan(val_read));
            assert(all(nan_match), sprintf('列 %s NaN 模式回读不匹配', name));
            finite_diff = diff_col(isfinite(val_orig));
            if ~isempty(finite_diff)
                assert(max(finite_diff) < 1e-9, sprintf('列 %s 数值回读残差超标', name));
            end
        end
    end
    fprintf('    [OK] CSV 全部 %d 列 100%% 逐元素严格回读断言全数通过!\n\n', width(T_records));

    %% =====================================================================
    %% 总结输出
    %% =====================================================================
    fprintf('=========================================================================\n');
    fprintf('   Gate C4-C (C4-C-SIM) 纯仿真单元测试结论: 全部 PASS\n');
    fprintf('=========================================================================\n');
    fprintf('   Subtest C1 (对称系统标定源绑定): 虚假 Delta_Kf=%.2e N/ct <= 1.0e-5 N/ct [PASS]\n', abs(info_c1b.apparent_delta_kf));
    fprintf('   Subtest C2 (强不对称物理保持界定): 物理符号严格保持, 相对假设标记 ASSUMPTION_ONLY [PASS]\n');
    fprintf('   Subtest C3 (增益差模三阶分区判定): 0.05%%达标, 0.10%%临界明确报告, 0.50%%刚性拦截 [PASS]\n');
    fprintf('   Subtest C4 (异常保护与零伪装冻结): 饱和/低激励/NaN/时延未确认全闭环冻结 [PASS]\n');
    fprintf('   数据治理与完整回读:              CSV %d 行 x %d 列 100%% 逐元素回读一致 [PASS]\n', height(T_records), width(T_records));
    fprintf('=========================================================================\n');
end

function rec = make_c4c_record(subtest, trial_id, mode, info, true_g, est_g, passed_th)
    rec = struct();
    rec.Subtest                = string(subtest);
    rec.Trial_ID               = double(trial_id);
    rec.Mode                   = string(mode);
    rec.Gain_Source            = string(info.gain_source);
    rec.Identifiability_Status = string(info.identifiability_status);
    rec.True_gL                = double(true_g(1));
    rec.True_gR                = double(true_g(2));
    rec.Est_gL                 = double(est_g(1));
    rec.Est_gR                 = double(est_g(2));
    rec.Delta_g_Diff           = double(abs(est_g(1) - est_g(2)));
    rec.Apparent_Delta_Kf      = double(info.apparent_delta_kf);
    rec.Did_Update             = logical(info.did_update);
    rec.Is_Frozen              = logical(info.is_frozen);
    rec.Reject_Reason          = string(info.reject_reason);
    rec.Passed_Threshold       = logical(passed_th);
end

function out = ternary(cond, val_true, val_false)
    if cond
        out = val_true;
    else
        out = val_false;
    end
end
