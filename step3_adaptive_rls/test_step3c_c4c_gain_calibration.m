%% TEST_STEP3C_C4C_GAIN_CALIBRATION.M - Gate C4-C 增益校正与可辨识性仿真单元测试
% =========================================================================
% 功能说明:
% 依据 STEP3_IMPLEMENTATION_PLAN.md 及最新技术审查意见，对电流传感器增益校准与可辨识性界定模块
% step3c_gain_calibrator.m 进行 Gate C4-C 纯仿真单元测试验收 (C4-C-SIM)。
%
% 严格设计与物理可辨识性红线:
% 1. 数学可辨识性混淆边界:
%    纯回采量测 (电流+位置) 在数学上不可解耦传感器增益误差 delta_g 与推力不对称 Delta_Kf;
% 2. 标定模式严格分层:
%    - ORACLE_GAIN: 仿真已知增益真值，验证算法数学正确性;
%    - EXTERNAL_REFERENCE: 模拟外部基准表/分流器，验证工程物理标定;
%    - RELATIVE_BALANCE_ASSUMED: 仅验证对称运行假设，输出标记为 ASSUMPTION_ONLY / UNIDENTIFIABLE，
%                                严禁宣称绝对标定，严禁进入 C8A-eng 主闭环;
% 3. C2 真实推力模型物理不对称辨识:
%    接入 F_true = Kf * i_true 与增益误差 g_true，通过独立最小二乘验证 Delta_Kf 符号保持与相对误差 <= 5%;
% 4. C3 增益差模严格三阶分区判定与 100 次 Monte Carlo 带噪评估:
%    - 0.05%: 必须满足 P95 虚假 Delta_Kf <= 1e-5 N/ct 门限 (PASS);
%    - 0.10%: 临界边界点，独立测试并显式报告数值与门限对比，严禁模糊处理;
%    - 0.50%: 超出标称范围，状态机必须标记为 OUT_OF_CALIBRATION_RANGE 并刚性拦截 (100 次全部拦截);
% 5. C4 异常工况零伪装硬冻结、故障锁存与显式 reset 机制:
%    饱和、NaN、未确认时延与超范围故障必须置位 freeze_latched，后续正常输入禁止恢复，
%    仅能通过 opts.reset = true 解除。低激励不锁存。
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
        'Reject_Reason', {}, 'Passed_Threshold', {}, ...
        'True_Delta_Kf', {}, 'Estimated_Delta_Kf', {}, ...
        'Relative_Delta_Kf_Error', {}, 'Physical_Sign_Preserved', {}, ...
        'MC_Trial', {}, 'P95_Gain_Error', {}, 'P95_Apparent_Delta_Kf', {});

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
    assert(~calib_state_c1a.freeze_latched, 'C1-A: 正常校准不应锁存');

    extra_c1a = struct('True_Delta_Kf', 0.0, 'Estimated_Delta_Kf', info_c1a.apparent_delta_kf, ...
        'Relative_Delta_Kf_Error', 0.0, 'Physical_Sign_Preserved', true);
    records(end+1) = make_c4c_record('C1_SYMMETRIC_ORACLE', 1, opts_c1a.mode, info_c1a, ...
        [1.0; 1.0], calib_state_c1a.gain_hat, true, extra_c1a);

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
    assert(~calib_state_c1b.freeze_latched, 'C1-B: 正常标定不应锁存');

    extra_c1b = struct('True_Delta_Kf', 0.0, 'Estimated_Delta_Kf', info_c1b.apparent_delta_kf, ...
        'Relative_Delta_Kf_Error', 0.0, 'Physical_Sign_Preserved', true);
    records(end+1) = make_c4c_record('C1_SYMMETRIC_EXTERNAL', 2, opts_c1b.mode, info_c1b, ...
        [1.0; 1.0], calib_state_c1b.gain_hat, true, extra_c1b);

    fprintf('    [OK] Subtest C1 验收通过: 对称系统虚假 Delta_Kf=%.2e N/ct <= 1.0e-5 N/ct!\n\n', ...
        abs(info_c1b.apparent_delta_kf));

    %% =====================================================================
    %% [Subtest C2] 强不对称物理保持与真实推力系数辨识检验 (r=0.70 / r=1.30)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C2] 开始强不对称物理保持与真实推力系数辨识检验 (r=0.70 / r=1.30)...\n');

    test_cases_r = [0.70, 1.30];
    for idx_r = 1:length(test_cases_r)
        r_true = test_cases_r(idx_r);
        Kf_L_true = r_true * Kf0;
        Kf_R_true = Kf0;
        delta_Kf_true = Kf_L_true - Kf_R_true;

        N_c2 = 400;
        rng(200 + idx_r, 'twister');

        i_true_L = 2500 + 900 * sin((1:N_c2)' * 0.04);
        i_true_R = 2300 + 700 * cos((1:N_c2)' * 0.035);

        F_true_L = Kf_L_true .* i_true_L;
        F_true_R = Kf_R_true .* i_true_R;

        g_true = [1.001; 0.999];

        i_meas = [
            g_true(1) .* i_true_L, ...
            g_true(2) .* i_true_R
        ];

        % 1. ORACLE_GAIN 模式
        opts_c2_oracle = struct();
        opts_c2_oracle.mode       = 'ORACLE_GAIN';
        opts_c2_oracle.true_gains = g_true;
        opts_c2_oracle.Kf_nominal = Kf0;

        state_oracle = [];
        i_corr_oracle = zeros(N_c2, 2);
        for k = 1:N_c2
            [i_corr_k, state_oracle, info_oracle] = step3c_gain_calibrator( ...
                [i_meas(k,1); i_meas(k,2)], [], [], state_oracle, opts_c2_oracle);
            i_corr_oracle(k, :) = i_corr_k';
        end

        Kf_L_hat_ora = sum(F_true_L .* i_corr_oracle(:,1)) / sum(i_corr_oracle(:,1).^2);
        Kf_R_hat_ora = sum(F_true_R .* i_corr_oracle(:,2)) / sum(i_corr_oracle(:,2).^2);
        delta_Kf_hat_ora = Kf_L_hat_ora - Kf_R_hat_ora;
        rel_err_ora = abs(delta_Kf_hat_ora - delta_Kf_true) / abs(delta_Kf_true);

        assert(sign(delta_Kf_hat_ora) == sign(delta_Kf_true), ...
            sprintf('C2 ORACLE r=%.2f: 真实 Delta_Kf 符号未保持', r_true));
        assert(rel_err_ora <= 0.05, ...
            sprintf('C2 ORACLE r=%.2f: Delta_Kf 相对误差超过 5%% (实际 %.2f%%)', r_true, rel_err_ora * 100));

        extra_ora = struct( ...
            'True_Delta_Kf', delta_Kf_true, ...
            'Estimated_Delta_Kf', delta_Kf_hat_ora, ...
            'Relative_Delta_Kf_Error', rel_err_ora, ...
            'Physical_Sign_Preserved', true);
        records(end+1) = make_c4c_record(sprintf('C2_ORACLE_R%.2f', r_true), idx_r, ...
            'ORACLE_GAIN', info_oracle, g_true, state_oracle.gain_hat, true, extra_ora);

        fprintf('      - [ORACLE r=%.2f] Delta_Kf真值=%+.4e, 估计值=%+.4e, 相对误差=%.4f%% -> PASS\n', ...
            r_true, delta_Kf_true, delta_Kf_hat_ora, rel_err_ora * 100);

        % 2. EXTERNAL_REFERENCE 模式
        opts_c2_ext = struct();
        opts_c2_ext.mode       = 'EXTERNAL_REFERENCE';
        opts_c2_ext.N_min      = 200;
        opts_c2_ext.Kf_nominal = Kf0;

        state_ext = [];
        i_corr_ext = zeros(N_c2, 2);
        for k = 1:N_c2
            i_ref_k = [i_true_L(k); i_true_R(k)];
            [i_corr_k, state_ext, info_ext] = step3c_gain_calibrator( ...
                [i_meas(k,1); i_meas(k,2)], i_ref_k, [], state_ext, opts_c2_ext);
            i_corr_ext(k, :) = i_corr_k';
        end

        Kf_L_hat_ext = sum(F_true_L .* i_corr_ext(:,1)) / sum(i_corr_ext(:,1).^2);
        Kf_R_hat_ext = sum(F_true_R .* i_corr_ext(:,2)) / sum(i_corr_ext(:,2).^2);
        delta_Kf_hat_ext = Kf_L_hat_ext - Kf_R_hat_ext;
        rel_err_ext = abs(delta_Kf_hat_ext - delta_Kf_true) / abs(delta_Kf_true);

        assert(sign(delta_Kf_hat_ext) == sign(delta_Kf_true), ...
            sprintf('C2 EXTERNAL r=%.2f: 真实 Delta_Kf 符号未保持', r_true));
        assert(rel_err_ext <= 0.05, ...
            sprintf('C2 EXTERNAL r=%.2f: Delta_Kf 相对误差超过 5%% (实际 %.2f%%)', r_true, rel_err_ext * 100));

        extra_ext = struct( ...
            'True_Delta_Kf', delta_Kf_true, ...
            'Estimated_Delta_Kf', delta_Kf_hat_ext, ...
            'Relative_Delta_Kf_Error', rel_err_ext, ...
            'Physical_Sign_Preserved', true);
        records(end+1) = make_c4c_record(sprintf('C2_EXTERNAL_R%.2f', r_true), idx_r + 2, ...
            'EXTERNAL_REFERENCE', info_ext, g_true, state_ext.gain_hat, true, extra_ext);

        fprintf('      - [EXTERNAL r=%.2f] Delta_Kf真值=%+.4e, 估计值=%+.4e, 相对误差=%.4f%% -> PASS\n', ...
            r_true, delta_Kf_true, delta_Kf_hat_ext, rel_err_ext * 100);

        % 3. RELATIVE_BALANCE_ASSUMED 模式 (仅作为不可辨识性/假设支线检验)
        opts_c2_assumed = struct('mode', 'RELATIVE_BALANCE_ASSUMED', 'Kf_nominal', Kf0);
        state_assumed = [];
        i_meas_k = [i_meas(1,1); i_meas(1,2)];
        [i_corr_assumed, state_assumed, info_assumed] = step3c_gain_calibrator( ...
            i_meas_k, [], [], state_assumed, opts_c2_assumed);

        assert(strcmp(info_assumed.identifiability_status, 'ASSUMPTION_ONLY') || ...
               strcmp(info_assumed.identifiability_status, 'UNIDENTIFIABLE'), ...
            'C2: RELATIVE_BALANCE_ASSUMED 必须标记为 ASSUMPTION_ONLY 或 UNIDENTIFIABLE');
        assert(~info_assumed.did_update, 'C2: 假设模式严禁 did_update');
        assert(all(i_corr_assumed == i_meas_k), 'C2: 假设模式必须安全直通原量测');
        assert(strcmp(info_assumed.gain_source, 'SYMMETRIC_MOTION_ASSUMPTION'), ...
            'C2: gain_source 必须为 SYMMETRIC_MOTION_ASSUMPTION');

        extra_assumed = struct( ...
            'True_Delta_Kf', delta_Kf_true, ...
            'Estimated_Delta_Kf', info_assumed.apparent_delta_kf, ...
            'Relative_Delta_Kf_Error', NaN, ...
            'Physical_Sign_Preserved', false);
        records(end+1) = make_c4c_record(sprintf('C2_ASSUMED_R%.2f', r_true), idx_r + 4, ...
            'RELATIVE_BALANCE_ASSUMED', info_assumed, g_true, [1.0; 1.0], true, extra_assumed);
        fprintf('      - [ASSUMPTION_ONLY r=%.2f] 状态: %s, 严禁用于主闭环 -> PASS\n', ...
            r_true, info_assumed.identifiability_status);
    end

    fprintf('    [OK] Subtest C2 验收通过: ORACLE/EXTERNAL 真实 Delta_Kf 符号与误差通过, RELATIVE_BALANCE_ASSUMED 严格限定为不可辨识假设支线!\n\n');

    %% =====================================================================
    %% [Subtest C3] 增益差模敏感度与严格三阶分区判定测试 (100 次 Monte Carlo 带噪)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C3] 开始增益差模敏感度与严格三阶分区判定测试 (MC N=100, sigma_i=5.0 counts)...\n');
    fprintf('    分区定义: 0.05%% (设计区), 0.10%% (临界边界), 0.50%% (超标拦截)\n');

    N_mc_c3 = 100;
    sigma_i_c3 = 5.0; % 电流测量高斯白噪声标准差 (counts)

    % 1. 分区 1: 0.05% 差模 (gL = 1.0005, gR = 1.0000) -> 必须满足 P95 虚假 Delta_Kf <= 1e-5 门限
    opts_c3_005 = struct('mode', 'EXTERNAL_REFERENCE', 'Kf_nominal', Kf0, 'Imax', Imax, ...
        'N_min', 200, 'max_asymmetry_range', 0.0020);
    g_true_005 = [1.0005; 1.0000];

    gain_error_mc_005 = zeros(N_mc_c3, 1);
    dkf_error_mc_005  = zeros(N_mc_c3, 1);
    did_update_005    = false(N_mc_c3, 1);
    frozen_005        = false(N_mc_c3, 1);

    for mc = 1:N_mc_c3
        rng(1000 + mc, 'twister');
        state_mc = [];
        info_mc = struct();
        for k = 1:220
            c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
            c_meas_k = c_ref_k .* g_true_005 + sigma_i_c3 * randn(2, 1);
            [~, state_mc, info_mc] = step3c_gain_calibrator( ...
                c_meas_k, c_ref_k, [], state_mc, opts_c3_005);
        end
        gain_error_mc_005(mc) = max(abs(state_mc.gain_hat - g_true_005));
        dkf_error_mc_005(mc)  = abs(info_mc.apparent_delta_kf);
        did_update_005(mc)    = state_mc.is_calibrated;
        frozen_005(mc)        = info_mc.is_frozen;
    end

    p95_gain_err_005 = prctile(gain_error_mc_005, 95);
    p95_dkf_005      = prctile(dkf_error_mc_005, 95);

    fprintf('      - [Zone 1: 0.05%% MC100] P95增益误差=%.4e, P95虚假Delta_Kf=%.4e N/ct (门限 %.1e) -> PASS\n', ...
        p95_gain_err_005, p95_dkf_005, th_false_dkf);
    assert(p95_dkf_005 <= th_false_dkf, 'C3: 0.05% 差模下 P95 虚假 Delta_Kf 必须达标');
    assert(all(did_update_005), 'C3: 0.05% 差模下全部 100 次 MC 必须成功标定');
    assert(all(frozen_005), 'C3: 0.05% 差模下全部 100 次 MC 必须冻结');

    extra_c3_005 = struct('MC_Trial', N_mc_c3, 'P95_Gain_Error', p95_gain_err_005, ...
        'P95_Apparent_Delta_Kf', p95_dkf_005);
    records(end+1) = make_c4c_record('C3_ZONE1_005PCT_MC100', 1, 'EXTERNAL_REFERENCE', info_mc, ...
        g_true_005, state_mc.gain_hat, true, extra_c3_005);

    % 2. 分区 2: 0.10% 临界边界点 (gL = 1.0010, gR = 1.0000) -> 独立测试并显式报告 P95 与边界对比
    opts_c3_010 = opts_c3_005;
    g_true_010 = [1.0010; 1.0000];

    gain_error_mc_010 = zeros(N_mc_c3, 1);
    dkf_error_mc_010  = zeros(N_mc_c3, 1);
    did_update_010    = false(N_mc_c3, 1);
    frozen_010        = false(N_mc_c3, 1);

    for mc = 1:N_mc_c3
        rng(2000 + mc, 'twister');
        state_mc = [];
        info_mc = struct();
        for k = 1:220
            c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
            c_meas_k = c_ref_k .* g_true_010 + sigma_i_c3 * randn(2, 1);
            [~, state_mc, info_mc] = step3c_gain_calibrator( ...
                c_meas_k, c_ref_k, [], state_mc, opts_c3_010);
        end
        gain_error_mc_010(mc) = max(abs(state_mc.gain_hat - g_true_010));
        dkf_error_mc_010(mc)  = abs(info_mc.apparent_delta_kf);
        did_update_010(mc)    = state_mc.is_calibrated;
        frozen_010(mc)        = info_mc.is_frozen;
    end

    p95_gain_err_010 = prctile(gain_error_mc_010, 95);
    p95_dkf_010      = prctile(dkf_error_mc_010, 95);

    fprintf('      - [Zone 2: 0.10%% (Boundary MC100)] P95增益误差=%.4e, P95虚假Delta_Kf=%.4e N/ct (门限 %.1e) -> %s\n', ...
        p95_gain_err_010, p95_dkf_010, th_false_dkf, char(ternary(p95_dkf_010 <= th_false_dkf, "PASS (Within Margin)", "FAIL")));
    assert(p95_dkf_010 <= th_false_dkf, 'C3: 0.10% 临界边界点 P95 虚假 Delta_Kf 必须受控于门限内');
    assert(all(did_update_010), 'C3: 0.10% 临界边界点全部 100 次 MC 必须成功标定');

    extra_c3_010 = struct('MC_Trial', N_mc_c3, 'P95_Gain_Error', p95_gain_err_010, ...
        'P95_Apparent_Delta_Kf', p95_dkf_010);
    records(end+1) = make_c4c_record('C3_ZONE2_010PCT_BOUNDARY_MC100', 2, 'EXTERNAL_REFERENCE', info_mc, ...
        g_true_010, state_mc.gain_hat, true, extra_c3_010);

    % 3. 分区 3: 0.50% 超标区间 (gL = 1.0050, gR = 1.0000) -> 必须全部 100 次标记 OUT_OF_CALIBRATION_RANGE 并刚性拦截
    opts_c3_050 = opts_c3_005;
    g_true_050 = [1.0050; 1.0000];

    over_range_detected = false(N_mc_c3, 1);
    mc_did_update_050   = false(N_mc_c3, 1);
    mc_frozen_050       = false(N_mc_c3, 1);
    mc_latched_050      = false(N_mc_c3, 1);

    for mc = 1:N_mc_c3
        rng(3000 + mc, 'twister');
        state_mc = [];
        info_mc = struct();
        for k = 1:220
            c_ref_k = [4000.0 + 1000.0*cos(0.04*k); 4000.0 + 1000.0*cos(0.04*k)];
            c_meas_k = c_ref_k .* g_true_050 + sigma_i_c3 * randn(2, 1);
            [c_corr_k, state_mc, info_mc] = step3c_gain_calibrator( ...
                c_meas_k, c_ref_k, [], state_mc, opts_c3_050);
        end
        over_range_detected(mc) = strcmp(info_mc.identifiability_status, 'OUT_OF_CALIBRATION_RANGE') && ...
                                  strcmp(info_mc.reject_reason, 'OUT_OF_RANGE');
        mc_did_update_050(mc)   = info_mc.did_update;
        mc_frozen_050(mc)       = info_mc.is_frozen;
        mc_latched_050(mc)      = state_mc.freeze_latched;
    end

    fprintf('      - [Zone 3: 0.50%% (Over-range MC100)] 超标捕获率: %d/%d, did_update=0: %d/%d, 锁存率: %d/%d -> PASS (Rigidly Intercepted)\n', ...
        sum(over_range_detected), N_mc_c3, sum(~mc_did_update_050), N_mc_c3, sum(mc_latched_050), N_mc_c3);
    assert(all(over_range_detected), 'C3: 0.50% 差模 100 次 MC 全部必须报告 OUT_OF_CALIBRATION_RANGE');
    assert(all(~mc_did_update_050), 'C3: 0.50% 差模 100 次 MC did_update 全部必须为 false');
    assert(all(mc_frozen_050), 'C3: 0.50% 差模 100 次 MC 全部必须冻结');
    assert(all(mc_latched_050), 'C3: 0.50% 差模 100 次 MC freeze_latched 全部必须为 true');

    extra_c3_050 = struct('MC_Trial', N_mc_c3, 'P95_Gain_Error', NaN, ...
        'P95_Apparent_Delta_Kf', abs(info_mc.apparent_delta_kf));
    records(end+1) = make_c4c_record('C3_ZONE3_050PCT_OVER_RANGE_MC100', 3, 'EXTERNAL_REFERENCE', info_mc, ...
        g_true_050, state_mc.gain_hat, false, extra_c3_050);

    fprintf('    [OK] Subtest C3 验收通过: 严格三阶分区判定生效 (0.05%%合格, 0.10%%临界明确报告, 0.50%%刚性拦截)!\n\n');

    %% =====================================================================
    %% [Subtest C4] 异常保护、低激励拦截、故障锁存与显式 reset 机制测试
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest C4] 开始异常保护、低激励拦截、故障锁存与显式 reset 测试...\n');

    opts_c4 = struct('mode', 'EXTERNAL_REFERENCE', 'Imax', Imax, 'Kf_nominal', Kf0);

    % Case 1: 电流饱和工况 (|i| >= 0.95*Imax) -> 独立初始化，必须锁存
    calib_state_sat = [];
    c_sat = [15500.0; 5000.0]; % 15500 >= 0.95 * 16000 = 15200
    [c_sat_out, calib_state_sat, info_sat] = step3c_gain_calibrator( ...
        c_sat, [15500.0; 5000.0], [], calib_state_sat, opts_c4);
    assert(strcmp(info_sat.reject_reason, 'SATURATION'), 'C4: 饱和未拦截');
    assert(~info_sat.did_update, 'C4: 饱和区严禁更新');
    assert(info_sat.is_frozen, 'C4: 饱和区必须冻结');
    assert(calib_state_sat.freeze_latched, 'C4: 饱和故障必须锁存 freeze_latched');
    assert(strcmp(calib_state_sat.freeze_reason, 'SATURATION'), 'C4: 锁存原因必须为 SATURATION');
    assert(all(c_sat_out == c_sat), 'C4: 饱和区必须安全直通');
    records(end+1) = make_c4c_record('C4_SATURATION', 1, opts_c4.mode, info_sat, [1.0; 1.0], [1.0; 1.0], true);

    % Case 2: 低动态激励工况 (|i_ref| < th_current_min) -> 独立初始化，不建议永久锁存
    calib_state_low = [];
    c_low = [10.0; 10.0]; % < 50 counts
    [c_low_out, calib_state_low, info_low] = step3c_gain_calibrator( ...
        c_low, c_low, [], calib_state_low, opts_c4);
    assert(strcmp(info_low.reject_reason, 'LOW_EXCITATION'), 'C4: 低激励未拦截');
    assert(~info_low.did_update && info_low.is_frozen, 'C4: 低激励区严禁更新');
    assert(~calib_state_low.freeze_latched, 'C4: 低激励为暂态，严禁永久锁存 freeze_latched');
    assert(all(c_low_out == c_low), 'C4: 低激励区必须安全直通');
    records(end+1) = make_c4c_record('C4_LOW_EXCITATION', 2, opts_c4.mode, info_low, [1.0; 1.0], [1.0; 1.0], true);

    % Case 3: 非有限异常输入 (含 NaN / Inf) -> 独立初始化，必须锁存
    calib_state_nan = [];
    c_nan = [NaN; 2000.0];
    [c_nan_out, calib_state_nan, info_nan] = step3c_gain_calibrator( ...
        c_nan, [2000.0; 2000.0], [], calib_state_nan, opts_c4);
    assert(strcmp(info_nan.reject_reason, 'NONFINITE_INPUT'), 'C4: NaN 未拦截');
    assert(strcmp(info_nan.identifiability_status, 'INVALID_INPUT'), 'C4: 状态应为 INVALID_INPUT');
    assert(~info_nan.did_update && info_nan.is_frozen, 'C4: 非有限输入严禁更新');
    assert(calib_state_nan.freeze_latched, 'C4: NaN 故障必须锁存 freeze_latched');
    assert(strcmp(calib_state_nan.freeze_reason, 'NONFINITE_INPUT'), 'C4: 锁存原因必须为 NONFINITE_INPUT');
    assert(isnan(c_nan_out(1)) && c_nan_out(2) == 2000.0, 'C4: NaN 输入安全直通');
    records(end+1) = make_c4c_record('C4_NONFINITE_NAN', 3, opts_c4.mode, info_nan, [1.0; 1.0], [1.0; 1.0], true);

    % Case 4: 通信时延未确认前置保护 (delay_confirmed = false) -> 独立初始化，必须锁存
    opts_c4_nodelay = opts_c4;
    opts_c4_nodelay.delay_confirmed = false;
    calib_state_del = [];
    [c_del_out, calib_state_del, info_del] = step3c_gain_calibrator( ...
        [2000.0; 2000.0], [2000.0; 2000.0], [], calib_state_del, opts_c4_nodelay);
    assert(strcmp(info_del.reject_reason, 'UNCONFIRMED_DELAY'), 'C4: 未确认时延未拦截');
    assert(~info_del.did_update && info_del.is_frozen, 'C4: 未确认时延严禁更新');
    assert(calib_state_del.freeze_latched, 'C4: 未确认时延故障必须锁存 freeze_latched');
    assert(strcmp(calib_state_del.freeze_reason, 'UNCONFIRMED_DELAY'), 'C4: 锁存原因必须为 UNCONFIRMED_DELAY');
    assert(all(c_del_out == [2000.0; 2000.0]), 'C4: 未确认时延安全直通');
    records(end+1) = make_c4c_record('C4_UNCONFIRMED_DELAY', 4, opts_c4.mode, info_del, [1.0; 1.0], [1.0; 1.0], true);

    % Case 5: 故障锁存防御测试 (后续正常输入不能偷偷恢复更新)
    state_latch = [];
    [~, state_latch, info_latch1] = step3c_gain_calibrator( ...
        [15500.0; 5000.0], [15500.0; 5000.0], [], state_latch, opts_c4);
    assert(strcmp(info_latch1.reject_reason, 'SATURATION'));
    assert(state_latch.freeze_latched, 'C4: 发生饱和后必须锁存');

    % 注入完全正常的输入，验证被锁存刚性拦截
    [normal_out, state_latch, info_latch2] = step3c_gain_calibrator( ...
        [3000.0; 3000.0], [3000.0; 3000.0], [], state_latch, opts_c4);
    assert(~info_latch2.did_update, 'C4: 锁存状态下正常输入严禁偷跑更新');
    assert(info_latch2.is_frozen, 'C4: 锁存状态下必须保持 is_frozen');
    assert(strcmp(info_latch2.reject_reason, 'SATURATION'), 'C4: 锁存原因必须维持为原始故障原因');
    assert(all(normal_out == [3000.0; 3000.0]), 'C4: 锁存拦截下输出必须安全直通');
    records(end+1) = make_c4c_record('C4_FAULT_LATCH_PROTECT', 5, opts_c4.mode, info_latch2, [1.0; 1.0], [1.0; 1.0], true);

    % Case 6: 显式 reset 机制验证 (仅在 opts.reset=true 时方可解除锁存)
    opts_reset = opts_c4;
    opts_reset.reset = true;
    [reset_out, state_latch, info_reset] = step3c_gain_calibrator( ...
        [3000.0; 3000.0], [3000.0; 3000.0], [], state_latch, opts_reset);
    assert(~state_latch.freeze_latched, 'C4: 显式 reset 后 freeze_latched 必须解除');
    assert(strcmp(state_latch.freeze_reason, 'NONE'), 'C4: 显式 reset 后 freeze_reason 必须重置为 NONE');
    records(end+1) = make_c4c_record('C4_EXPLICIT_RESET', 6, opts_c4.mode, info_reset, [1.0; 1.0], [1.0; 1.0], true);

    fprintf('    [OK] Subtest C4 验收通过: 饱和、低激励、NaN、未确认时延、故障锁存防御与显式 reset 机制全数达标!\n\n');

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
    fprintf('   Subtest C2 (强不对称物理保持界定): ORACLE/EXTERNAL 真实 Delta_Kf 符号正确, 误差<=5%% [PASS]\n');
    fprintf('                                  RELATIVE_BALANCE_ASSUMED 严格限定为不可辨识假设支线 [PASS]\n');
    fprintf('   Subtest C3 (增益差模三阶分区判定): MC100 带噪: 0.05%%达标, 0.10%%临界明确报告, 0.50%%刚性拦截 [PASS]\n');
    fprintf('   Subtest C4 (异常保护、锁存与reset): 饱和/NaN/时延未确认故障锁存, 正常输入偷跑拦截, 显式reset成功 [PASS]\n');
    fprintf('   数据治理与完整回读:              CSV %d 行 x %d 列 100%% 逐元素回读一致 [PASS]\n', height(T_records), width(T_records));
    fprintf('=========================================================================\n');
end

function rec = make_c4c_record(subtest, trial_id, mode, info, true_g, est_g, passed_th, extra)
    if nargin < 8 || isempty(extra), extra = struct(); end
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

    if isfield(extra, 'True_Delta_Kf'), rec.True_Delta_Kf = double(extra.True_Delta_Kf); else, rec.True_Delta_Kf = NaN; end
    if isfield(extra, 'Estimated_Delta_Kf'), rec.Estimated_Delta_Kf = double(extra.Estimated_Delta_Kf); else, rec.Estimated_Delta_Kf = NaN; end
    if isfield(extra, 'Relative_Delta_Kf_Error'), rec.Relative_Delta_Kf_Error = double(extra.Relative_Delta_Kf_Error); else, rec.Relative_Delta_Kf_Error = NaN; end
    if isfield(extra, 'Physical_Sign_Preserved'), rec.Physical_Sign_Preserved = logical(extra.Physical_Sign_Preserved); else, rec.Physical_Sign_Preserved = false; end
    if isfield(extra, 'MC_Trial'), rec.MC_Trial = double(extra.MC_Trial); else, rec.MC_Trial = NaN; end
    if isfield(extra, 'P95_Gain_Error'), rec.P95_Gain_Error = double(extra.P95_Gain_Error); else, rec.P95_Gain_Error = NaN; end
    if isfield(extra, 'P95_Apparent_Delta_Kf'), rec.P95_Apparent_Delta_Kf = double(extra.P95_Apparent_Delta_Kf); else, rec.P95_Apparent_Delta_Kf = NaN; end
end

function out = ternary(cond, val_true, val_false)
    if cond
        out = val_true;
    else
        out = val_false;
    end
end
