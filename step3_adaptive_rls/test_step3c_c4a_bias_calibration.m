%% TEST_STEP3C_C4A_BIAS_CALIBRATION.M - Gate C4-A 静态霍尔电流零偏标定单元测试
% =========================================================================
% 功能说明:
% 依据 STEP3_IMPLEMENTATION_PLAN.md 第四阶段规划，对独立电流量测通道校准器
% step3c_current_channel_calibrator.m 进行 Gate C4-A 单元测试验收。
%
% 严格红线边界:
% 1. 独立静态标定单元测试，严禁运行 RLS，严禁接入闭环控制器或 SyncAlloc；
% 2. 严禁在零偏模块暗含增益校正 (gain_scale 严格为 [1; 1])；
% 3. 严禁使用普通代数均值，强制执行 Hampel / MAD 异常剔除与稳健中位数估计；
% 4. 严苛准入判据: 驱动禁用、零电流指令、零速度、零角速度、零加速度。
%
% 六大子测试规划 (A1 ~ A6):
%   A1: 100 次蒙特卡洛标称高斯噪声精度测试 (P95(e_trial) <= 2.0 counts)
%   A2: 逐项破坏 6 大准入判据 + 非有限输入，验证 0 次更新
%   A3: 累计中途注入运动扰动，验证缓冲区清空并回退 IDLE
%   A4: 进入 FROZEN 后注入大运动电流，验证参数绝对锁定 (< 1e-15)
%   A5: 样本不足 (< N_min)，验证严禁进入 FROZEN
%   A6: 2% 脉冲异常点 (±100 counts) 与 NaN/Inf 保护，验证 Hampel 正确剔除
%       且 P95(e_trial) <= 2.0 counts 仍成立
%
% 数据导出:
%   step3_adaptive_rls/step3c_c4a_bias_results.csv (200 trials 逐条明细与 100% 回读断言)
% =========================================================================

function test_step3c_c4a_bias_calibration()
    clc;
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-4 Gate C4-A: 霍尔传感器静态零偏标定单元测试 (A1 ~ A6)\n');
    fprintf('=========================================================================\n\n');

    script_dir = fileparts(mfilename('fullpath'));
    addpath(script_dir);

    % 1. 检查被测核心函数是否存在
    calib_func_path = fullfile(script_dir, 'step3c_current_channel_calibrator.m');
    assert(exist(calib_func_path, 'file') == 2, '缺少 step3c_current_channel_calibrator.m');
    fprintf('>>> [OK] 被测校准器入口已就绪: %s\n\n', calib_func_path);

    % 2. 标定参数配置
    opts = struct();
    opts.calibration_duration = 0.5;   % 标定持续时间 0.5s
    opts.dt                   = 0.001; % 采样步长 1ms (1 kHz)
    opts.N_min                = ceil(opts.calibration_duration / opts.dt); % 500 步
    opts.min_retained_ratio   = 0.90;  % Hampel 最少保留 90% 样本 (450 步)
    opts.th_cmd               = 1e-4;  % 电流指令门限 counts
    opts.th_v                 = 1e-5;  % 速度门限 m/s
    opts.th_omega             = 1e-5;  % 角速度门限 rad/s
    opts.th_a                 = 1e-5;  % 加速度门限 m/s^2
    opts.hampel_nsigma        = 3.0;   % Hampel 3-sigma 门限
    opts.reset                = false;

    N_mc = 100;

    %% =====================================================================
    %% [Subtest A1] 100 次蒙特卡洛标称高斯噪声精度测试
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest A1] 开始 100 次蒙特卡洛标称高斯噪声精度测试 (MC 100)...\n');
    fprintf('    参数: 零偏真值 in [-30, +30] counts, sigma_i = 10.0 counts, N = 600 steps\n');

    a1_true_L   = zeros(N_mc, 1);
    a1_true_R   = zeros(N_mc, 1);
    a1_est_L    = zeros(N_mc, 1);
    a1_est_R    = zeros(N_mc, 1);
    a1_err_L    = zeros(N_mc, 1);
    a1_err_R    = zeros(N_mc, 1);
    a1_err_max  = zeros(N_mc, 1);
    a1_N_eff_L  = zeros(N_mc, 1);
    a1_N_eff_R  = zeros(N_mc, 1);
    a1_is_calib = false(N_mc, 1);
    a1_mode     = cell(N_mc, 1);

    cmd_zero = struct('iL', 0.0, 'iR', 0.0);
    motion_static = struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true);

    N_steps_a1 = 600; % 超过 N_min=500，验证进入 FROZEN 后的稳态

    for j = 1:N_mc
        seed_j = 20260927 + j;
        rng(seed_j, 'twister');

        bias_L_true = -30.0 + 60.0 * rand();
        bias_R_true = -30.0 + 60.0 * rand();

        a1_true_L(j) = bias_L_true;
        a1_true_R(j) = bias_R_true;

        % 生成 10 counts 高斯白噪声
        noise_L = 10.0 * randn(N_steps_a1, 1);
        noise_R = 10.0 * randn(N_steps_a1, 1);

        iL_meas = bias_L_true + noise_L;
        iR_meas = bias_R_true + noise_R;

        calib_state = [];
        for k = 1:N_steps_a1
            raw_k = [iL_meas(k); iR_meas(k)];
            [~, calib_state, info] = step3c_current_channel_calibrator( ...
                raw_k, cmd_zero, motion_static, calib_state, opts);
        end

        % 断言状态与增益红线
        assert(calib_state.is_calibrated, sprintf('A1 Trial %d: 未完成标定', j));
        assert(strcmp(calib_state.mode, 'FROZEN'), sprintf('A1 Trial %d: 状态未进入 FROZEN', j));
        assert(norm(calib_state.gain_scale - [1.0; 1.0]) < 1e-12, '增益比例必须严格为 [1; 1]');

        a1_est_L(j)    = calib_state.bias_hat(1);
        a1_est_R(j)    = calib_state.bias_hat(2);
        a1_err_L(j)    = abs(a1_est_L(j) - bias_L_true);
        a1_err_R(j)    = abs(a1_est_R(j) - bias_R_true);
        a1_err_max(j)  = max(a1_err_L(j), a1_err_R(j));
        a1_N_eff_L(j)  = info.N_eff_L;
        a1_N_eff_R(j)  = info.N_eff_R;
        a1_is_calib(j) = calib_state.is_calibrated;
        a1_mode{j}     = calib_state.mode;
    end

    p95_a1_L     = prctile(a1_err_L, 95);
    p95_a1_R     = prctile(a1_err_R, 95);
    p95_a1_trial = prctile(a1_err_max, 95);
    max_a1_trial = max(a1_err_max);
    med_a1_trial = median(a1_err_max);
    mean_a1_trial = mean(a1_err_max);

    fprintf('    [A1 统计指标]:\n');
    fprintf('      - P95(e_L)     = %.4f counts\n', p95_a1_L);
    fprintf('      - P95(e_R)     = %.4f counts\n', p95_a1_R);
    fprintf('      - P95(e_trial) = %.4f counts (验收阈值 <= 2.0 counts)\n', p95_a1_trial);
    fprintf('      - Max(e_trial) = %.4f counts\n', max_a1_trial);
    fprintf('      - Med(e_trial) = %.4f counts\n', med_a1_trial);
    fprintf('      - Mean(e_trial)= %.4f counts\n', mean_a1_trial);
    fprintf('      - 平均保留样本数: L = %.1f, R = %.1f (>= 450)\n', mean(a1_N_eff_L), mean(a1_N_eff_R));

    assert(p95_a1_trial <= 2.0, sprintf('A1 失败: P95(e_trial) = %.4f > 2.0 counts', p95_a1_trial));
    assert(all(a1_N_eff_L >= 450) && all(a1_N_eff_R >= 450), 'A1 失败: Hampel 保留样本不足 90%');
    fprintf('    [OK] Subtest A1 验收通过: P95(e_trial) = %.4f counts <= 2.0 counts\n\n', p95_a1_trial);

    %% =====================================================================
    %% [Subtest A2] 逐项破坏 6 大准入判据与非有限保护，验证 0 次更新
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest A2] 逐项破坏准入判据测试 (断言更新次数 == 0)...\n');

    test_cases_a2 = {
        % Name, cmd, motion, raw, expected_reject_reason
        'Case 1: iL_cmd 超标', ...
            struct('iL', 1e-3, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; -10.0], 'CMD_NONZERO';
        'Case 2: iR_cmd 超标', ...
            struct('iL', 0.0, 'iR', -1e-3), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; -10.0], 'CMD_NONZERO';
        'Case 3: vG 速度非零', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 1e-4, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; -10.0], 'MOTION_NONZERO';
        'Case 4: omega 角速度非零', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 1e-4, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; -10.0], 'MOTION_NONZERO';
        'Case 5: aG 加速度非零', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 1e-4, 'drive_torque_disabled', true), ...
            [10.0; -10.0], 'MOTION_NONZERO';
        'Case 6: 驱动使能 (drive_torque_disabled = false)', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', false), ...
            [10.0; -10.0], 'DRIVE_ENABLED';
        'Case 7: 电流输入非有限 (NaN)', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [NaN; 10.0], 'NONFINITE_INPUT';
        'Case 8: 电流输入非有限 (Inf)', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; Inf], 'NONFINITE_INPUT';
        'Case 9: 指令输入非有限 (NaN)', ...
            struct('iL', NaN, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; 10.0], 'NONFINITE_INPUT';
        'Case 10: 运动状态非有限 (Inf)', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', Inf, 'aG', 0.0, 'drive_torque_disabled', true), ...
            [10.0; 10.0], 'NONFINITE_INPUT';
        'Case 11: drive_torque_disabled 为 NaN', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', NaN), ...
            [10.0; -10.0], 'NONFINITE_INPUT';
        'Case 12: drive_torque_disabled 非布尔数值 (2)', ...
            struct('iL', 0.0, 'iR', 0.0), ...
            struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', 2), ...
            [10.0; -10.0], 'NONFINITE_INPUT'
    };

    update_count_when_inadmissible = 0;
    N_steps_check = 20;

    for c = 1:size(test_cases_a2, 1)
        c_name     = test_cases_a2{c, 1};
        c_cmd      = test_cases_a2{c, 2};
        c_motion   = test_cases_a2{c, 3};
        c_raw      = test_cases_a2{c, 4};
        c_expected = test_cases_a2{c, 5};

        calib_state = [];
        for step = 1:N_steps_check
            [cal_out, calib_state, info] = step3c_current_channel_calibrator( ...
                c_raw, c_cmd, c_motion, calib_state, opts);

            assert(strcmp(info.reject_reason, c_expected), ...
                sprintf('A2 %s: 预期原因码 %s，实际 %s', c_name, c_expected, info.reject_reason));
            assert(~info.is_admissible, sprintf('A2 %s: is_admissible 应为 false', c_name));
            assert(strcmp(calib_state.mode, 'IDLE'), sprintf('A2 %s: 状态应保持 IDLE', c_name));
            assert(~calib_state.is_calibrated, sprintf('A2 %s: is_calibrated 应为 false', c_name));
            assert(all(isfinite(cal_out)), sprintf('A2 %s: 输出校准值必须有限', c_name));
            assert(~info.did_update, sprintf('A2 %s: did_update 应为 false', c_name));

            if calib_state.valid_count > 0
                update_count_when_inadmissible = update_count_when_inadmissible + 1;
            end
        end
        fprintf('    [OK] %s -> 原因码: %s, 累计更新数 = 0\n', c_name, c_expected);
    end

    assert(update_count_when_inadmissible == 0, ...
        sprintf('A2 失败: 非法准入时累计更新数 %d > 0', update_count_when_inadmissible));

    % 输入契约边界硬错误检查 (必须触发 assert/error)
    fprintf('    [A2 契约检查]: 验证非法维度/虚数输入触发契约断言...\n');
    contract_err_3elem = false;
    try
        step3c_current_channel_calibrator([10.0; -10.0; 5.0], cmd_zero, motion_static, [], opts);
    catch
        contract_err_3elem = true;
    end
    assert(contract_err_3elem, 'A2 失败: 3 元素 current_raw 必须触发输入契约错误');

    contract_err_1elem = false;
    try
        step3c_current_channel_calibrator(10.0, cmd_zero, motion_static, [], opts);
    catch
        contract_err_1elem = true;
    end
    assert(contract_err_1elem, 'A2 失败: 1 元素 current_raw 必须触发输入契约错误');

    contract_err_complex = false;
    try
        step3c_current_channel_calibrator([10.0 + 1i; -10.0], cmd_zero, motion_static, [], opts);
    catch
        contract_err_complex = true;
    end
    assert(contract_err_complex, 'A2 失败: 虚数 current_raw 必须触发输入契约错误');
    fprintf('    [OK] 输入契约硬检验通过: 3元素/标量/虚数输入均严格触发契约拒绝\n');
    fprintf('    [OK] Subtest A2 验收通过: 非法准入下累计更新次数严格为 0!\n\n');

    %% =====================================================================
    %% [Subtest A3] 累计中途注入运动扰动，验证缓冲区清空并回退 IDLE
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest A3] 累计中途扰动中断测试 (中断前 200 步，中断清空并重置)...\n');

    calib_state = [];
    % 阶段 1: 正常累加 200 步
    for k = 1:200
        raw_k = [15.0 + randn(); -12.0 + randn()];
        [~, calib_state, info] = step3c_current_channel_calibrator( ...
            raw_k, cmd_zero, motion_static, calib_state, opts);
    end
    assert(calib_state.valid_count == 200, 'A3 阶段 1: 累计计数应为 200');
    assert(strcmp(calib_state.mode, 'ACCUMULATING'), 'A3 阶段 1: 模式应为 ACCUMULATING');
    assert(length(calib_state.buffer_L) == 200, 'A3 阶段 1: 缓冲区长度应为 200');
    assert(~calib_state.is_calibrated, 'A3 阶段 1: 尚未标定');

    % 阶段 2: 第 201 步注入运动扰动 (vG = 0.05 m/s)
    motion_disturbed = motion_static;
    motion_disturbed.vG = 0.05;
    [~, calib_state, info] = step3c_current_channel_calibrator( ...
        [15.0; -12.0], cmd_zero, motion_disturbed, calib_state, opts);

    assert(strcmp(calib_state.mode, 'IDLE'), 'A3 阶段 2: 扰动发生后必须立即回退 IDLE');
    assert(calib_state.valid_count == 0, 'A3 阶段 2: 扰动发生后有效计数必须清零');
    assert(isempty(calib_state.buffer_L), 'A3 阶段 2: buffer_L 必须清空');
    assert(isempty(calib_state.buffer_R), 'A3 阶段 2: buffer_R 必须清空');
    assert(~calib_state.is_calibrated, 'A3 阶段 2: 必须保持未标定');
    assert(norm(calib_state.bias_hat) == 0.0, 'A3 阶段 2: bias_hat 必须为 0');
    assert(strcmp(info.reject_reason, 'MOTION_NONZERO'), 'A3 阶段 2: 原因码应为 MOTION_NONZERO');

    % 阶段 3: 恢复静态，重新累加 200 步，验证状态机正常重启
    for k = 1:200
        raw_k = [15.0 + randn(); -12.0 + randn()];
        [~, calib_state, info] = step3c_current_channel_calibrator( ...
            raw_k, cmd_zero, motion_static, calib_state, opts);
    end
    assert(calib_state.valid_count == 200, 'A3 阶段 3: 重启后有效计数应重新累加至 200');
    assert(strcmp(calib_state.mode, 'ACCUMULATING'), 'A3 阶段 3: 重启后模式应为 ACCUMULATING');
    assert(length(calib_state.buffer_L) == 200, 'A3 阶段 3: 重新累加长度应为 200');
    fprintf('    [OK] Subtest A3 验收通过: 中途扰动即刻清空缓冲区并复位至 IDLE!\n\n');

    %% =====================================================================
    %% [Subtest A4] 进入 FROZEN 后注入大运动电流，验证参数绝对锁定
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest A4] 进入 FROZEN 后大电流冲击锁定测试 (断言漂移 < 1e-15)...\n');

    % 阶段 1: 完成 500 步正常标定，真值 [25.0; -18.0]
    rng(20260927, 'twister');
    calib_state = [];
    for k = 1:500
        raw_k = [25.0 + 10.0 * randn(); -18.0 + 10.0 * randn()];
        [~, calib_state, ~] = step3c_current_channel_calibrator( ...
            raw_k, cmd_zero, motion_static, calib_state, opts);
    end
    assert(calib_state.is_calibrated, 'A4: 应已完成标定');
    assert(strcmp(calib_state.mode, 'FROZEN'), 'A4: 模式应为 FROZEN');

    bias_before_motion = calib_state.bias_hat;

    % 阶段 2: 注入 1000 步大工作电流与全要素运动状态 (8000 counts, 驱动使能, 加速度 2m/s^2)
    cmd_motion = struct('iL', 8000.0, 'iR', -8000.0);
    motion_running = struct('vG', 0.5, 'omega', 0.05, 'aG', 2.0, 'drive_torque_disabled', false);

    for k = 1:1000
        raw_k = [8000.0 + 25.0 + 10.0 * randn(); -8000.0 - 18.0 + 10.0 * randn()];
        [cal_out, calib_state, info] = step3c_current_channel_calibrator( ...
            raw_k, cmd_motion, motion_running, calib_state, opts);

        assert(strcmp(calib_state.mode, 'FROZEN'), 'A4 运行中: 状态应锁定为 FROZEN');
        assert(calib_state.is_calibrated, 'A4 运行中: is_calibrated 应保持 true');
        assert(info.is_calibrated, 'A4 运行中: info.is_calibrated 应保持 true');
        assert(~info.is_admissible, 'A4 运行中: 驱动使能且运动工况下 is_admissible 必须为 false');
        assert(strcmp(info.reject_reason, 'DRIVE_ENABLED'), 'A4 运行中: 原因码必须为 DRIVE_ENABLED');
        assert(~info.did_update, 'A4 运行中: did_update 必须为 false');

        % 验证输出精确扣除了冻结零偏
        expected_cal = (raw_k - bias_before_motion) ./ [1.0; 1.0];
        assert(norm(cal_out - expected_cal) < 1e-12, 'A4 运行中: 校准输出未精确扣减冻结零偏');
    end

    bias_after_motion = calib_state.bias_hat;
    bias_drift = max(abs(bias_after_motion - bias_before_motion));

    fprintf('    [A4 结果]:\n');
    fprintf('      - 标定冻结零偏: L = %.6f, R = %.6f counts\n', bias_before_motion(1), bias_before_motion(2));
    fprintf('      - 运动后零偏:   L = %.6f, R = %.6f counts\n', bias_after_motion(1), bias_after_motion(2));
    fprintf('      - 最大参数漂移: %.2e counts (断言 < 1e-15)\n', bias_drift);

    assert(bias_drift < 1e-15, sprintf('A4 失败: 冻结后参数漂移 %.2e >= 1e-15', bias_drift));
    fprintf('    [OK] Subtest A4 验收通过: 冻结状态下强激励运动参数漂移绝对为 0 (< 1e-15)!\n\n');

    %% =====================================================================
    %% [Subtest A5] 样本不足 (< N_min)，验证严禁进入 FROZEN
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest A5] 采样样本不足测试 (N = 300 < N_min = 500)...\n');

    calib_state = [];
    for k = 1:300
        raw_k = [10.0 + randn(); -10.0 + randn()];
        [cal_out, calib_state, info] = step3c_current_channel_calibrator( ...
            raw_k, cmd_zero, motion_static, calib_state, opts);
    end

    assert(calib_state.valid_count == 300, 'A5: valid_count 应为 300');
    assert(strcmp(calib_state.mode, 'ACCUMULATING'), 'A5: 状态必须仍为 ACCUMULATING');
    assert(~calib_state.is_calibrated, 'A5: 样本不足时严禁标定 (is_calibrated 必须为 false)');
    assert(norm(calib_state.bias_hat) == 0.0, 'A5: 未标定时 bias_hat 必须为 [0; 0]');
    assert(norm(cal_out - raw_k) < 1e-12, 'A5: 未标定时应直通原始电流');

    is_calibrated_when_insufficient = calib_state.is_calibrated;
    assert(~is_calibrated_when_insufficient, 'A5 失败: 样本不足却完成了标定');
    fprintf('    [OK] Subtest A5 验收通过: 样本不足时严格禁止进入 FROZEN，信号安全直通!\n\n');

    %% =====================================================================
    %% [Subtest A6] 2% 脉冲异常点 (±100 counts) 与 NaN/Inf 保护
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest A6] 2%% 脉冲异常点 (±100 counts) 稳健性与 NaN/Inf 保护测试...\n');

    % 1. 非有限输入防护验证
    calib_state = [];
    [cal_out_nan, calib_state, info_nan] = step3c_current_channel_calibrator( ...
        [NaN; 20.0], cmd_zero, motion_static, calib_state, opts);
    assert(all(isfinite(cal_out_nan)), 'A6: NaN 输入时输出必须有限');
    assert(strcmp(info_nan.reject_reason, 'NONFINITE_INPUT'), 'A6: 原因码应为 NONFINITE_INPUT');

    [cal_out_inf, calib_state, info_inf] = step3c_current_channel_calibrator( ...
        [Inf; -Inf], cmd_zero, motion_static, calib_state, opts);
    assert(all(isfinite(cal_out_inf)), 'A6: Inf 输入时输出必须有限');
    assert(strcmp(info_inf.reject_reason, 'NONFINITE_INPUT'), 'A6: 原因码应为 NONFINITE_INPUT');

    % 2. 100 次蒙特卡洛脉冲异常点测试 (每组 500 样本中注入 10 个 ±100 counts 脉冲)
    a6_true_L   = zeros(N_mc, 1);
    a6_true_R   = zeros(N_mc, 1);
    a6_est_L    = zeros(N_mc, 1);
    a6_est_R    = zeros(N_mc, 1);
    a6_err_L    = zeros(N_mc, 1);
    a6_err_R    = zeros(N_mc, 1);
    a6_err_max  = zeros(N_mc, 1);
    a6_N_eff_L  = zeros(N_mc, 1);
    a6_N_eff_R  = zeros(N_mc, 1);
    a6_is_calib = false(N_mc, 1);
    a6_mode     = cell(N_mc, 1);

    N_steps_a6 = 500;
    N_spikes   = 10; % 500 * 2% = 10 个脉冲

    for j = 1:N_mc
        seed_j = 20261027 + j;
        rng(seed_j, 'twister');

        bias_L_true = -30.0 + 60.0 * rand();
        bias_R_true = -30.0 + 60.0 * rand();

        a6_true_L(j) = bias_L_true;
        a6_true_R(j) = bias_R_true;

        % 标称高斯白噪声
        noise_L = 10.0 * randn(N_steps_a6, 1);
        noise_R = 10.0 * randn(N_steps_a6, 1);

        % 随机注入 2% 强脉冲离群点 (±100 counts)
        idx_spikes_L = randperm(N_steps_a6, N_spikes);
        idx_spikes_R = randperm(N_steps_a6, N_spikes);
        spike_val_L  = (sign(randn(N_spikes, 1)) + (randn(N_spikes, 1) == 0)) * 100.0;
        spike_val_R  = (sign(randn(N_spikes, 1)) + (randn(N_spikes, 1) == 0)) * 100.0;

        noise_L(idx_spikes_L) = noise_L(idx_spikes_L) + spike_val_L;
        noise_R(idx_spikes_R) = noise_R(idx_spikes_R) + spike_val_R;

        iL_meas = bias_L_true + noise_L;
        iR_meas = bias_R_true + noise_R;

        calib_state = [];
        for k = 1:N_steps_a6
            raw_k = [iL_meas(k); iR_meas(k)];
            [cal_k, calib_state, info] = step3c_current_channel_calibrator( ...
                raw_k, cmd_zero, motion_static, calib_state, opts);
            assert(all(isfinite(cal_k)), sprintf('A6 Trial %d Step %d: 输出非有限', j, k));
        end

        assert(calib_state.is_calibrated, sprintf('A6 Trial %d: 未完成标定', j));
        assert(strcmp(calib_state.mode, 'FROZEN'), sprintf('A6 Trial %d: 未进入 FROZEN', j));
        assert(info.N_eff_L >= 450, sprintf('A6 Trial %d: L 有效样本不足 90%%', j));
        assert(info.N_eff_R >= 450, sprintf('A6 Trial %d: R 有效样本不足 90%%', j));

        a6_est_L(j)    = calib_state.bias_hat(1);
        a6_est_R(j)    = calib_state.bias_hat(2);
        a6_err_L(j)    = abs(a6_est_L(j) - bias_L_true);
        a6_err_R(j)    = abs(a6_est_R(j) - bias_R_true);
        a6_err_max(j)  = max(a6_err_L(j), a6_err_R(j));
        a6_N_eff_L(j)  = info.N_eff_L;
        a6_N_eff_R(j)  = info.N_eff_R;
        a6_is_calib(j) = calib_state.is_calibrated;
        a6_mode{j}     = calib_state.mode;
    end

    p95_a6_L     = prctile(a6_err_L, 95);
    p95_a6_R     = prctile(a6_err_R, 95);
    p95_a6_trial = prctile(a6_err_max, 95);
    max_a6_trial = max(a6_err_max);
    med_a6_trial = median(a6_err_max);
    mean_a6_trial = mean(a6_err_max);

    fprintf('    [A6 脉冲异常点统计指标]:\n');
    fprintf('      - P95(e_L)     = %.4f counts\n', p95_a6_L);
    fprintf('      - P95(e_R)     = %.4f counts\n', p95_a6_R);
    fprintf('      - P95(e_trial) = %.4f counts (验收阈值 <= 2.0 counts)\n', p95_a6_trial);
    fprintf('      - Max(e_trial) = %.4f counts\n', max_a6_trial);
    fprintf('      - Med(e_trial) = %.4f counts\n', med_a6_trial);
    fprintf('      - Mean(e_trial)= %.4f counts\n', mean_a6_trial);
    fprintf('      - 平均保留有效样本: L = %.1f, R = %.1f / 500\n', mean(a6_N_eff_L), mean(a6_N_eff_R));

    assert(p95_a6_trial <= 2.0, sprintf('A6 失败: 脉冲污染下 P95(e_trial) = %.4f > 2.0 counts', p95_a6_trial));
    fprintf('    [OK] Subtest A6 验收通过: 2%% 脉冲污染下 Hampel 稳健过滤保持 P95(e_trial) <= 2.0 counts!\n\n');

    %% =====================================================================
    %% 数据导出与 100% 内存回读校验 (CSV)
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3c_c4a_bias_results.csv');
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> 正在导出 Gate C4-A 蒙特卡洛结果表: %s\n', csv_file);

    % 构造完整表格: 100 行 A1 标称测试 + 100 行 A6 脉冲异常测试 = 200 行
    trial_id_all = [ (1:N_mc)'; (1:N_mc)' ];
    subtest_all  = [ repmat({'A1_Nominal_Gaussian'}, N_mc, 1); repmat({'A6_2pct_Impulse'}, N_mc, 1) ];
    seed_all     = [ (20260927 + (1:N_mc))'; (20261027 + (1:N_mc))' ];

    true_L_all   = [ a1_true_L; a6_true_L ];
    true_R_all   = [ a1_true_R; a6_true_R ];
    est_L_all    = [ a1_est_L;  a6_est_L ];
    est_R_all    = [ a1_est_R;  a6_est_R ];
    err_L_all    = [ a1_err_L;  a6_err_L ];
    err_R_all    = [ a1_err_R;  a6_err_R ];
    err_max_all  = [ a1_err_max; a6_err_max ];
    is_calib_all = [ a1_is_calib; a6_is_calib ];
    mode_all     = [ a1_mode; a6_mode ];
    N_eff_L_all  = [ a1_N_eff_L; a6_N_eff_L ];
    N_eff_R_all  = [ a1_N_eff_R; a6_N_eff_R ];
    pass_p95_all = (err_max_all <= 2.0);

    results_table = table( ...
        trial_id_all, subtest_all, seed_all, ...
        true_L_all, true_R_all, est_L_all, est_R_all, ...
        err_L_all, err_R_all, err_max_all, ...
        is_calib_all, mode_all, N_eff_L_all, N_eff_R_all, pass_p95_all, ...
        'VariableNames', { ...
            'Trial_ID', 'Subtest', 'RNG_Seed', ...
            'True_Bias_L_count', 'True_Bias_R_count', ...
            'Est_Bias_L_count', 'Est_Bias_R_count', ...
            'Err_L_count', 'Err_R_count', 'Err_Trial_Max_count', ...
            'Is_Calibrated', 'Calib_Mode', ...
            'N_Eff_L', 'N_Eff_R', 'Within_P95_Threshold' ...
        });

    writetable(results_table, csv_file);
    fprintf('    [OK] CSV 写入完成，共计 %d 行 x %d 列\n', height(results_table), width(results_table));

    % 100% 内存回读校验 (全 15 列严格逐列逐元素)
    fprintf('>>> 执行 CSV 全部 15 列 100%% 逐元素严格内存回读校验...\n');
    T_read = readtable(csv_file);
    assert(height(T_read) == 200, '回读行数必须为 200 行');
    assert(width(T_read) == 15, '回读列数必须为 15 列');

    % 1. Trial_ID
    assert(isequal(T_read.Trial_ID, trial_id_all), 'Col 1 Trial_ID 回读不匹配');
    % 2. Subtest
    assert(all(strcmp(T_read.Subtest, subtest_all)), 'Col 2 Subtest 字符串回读不匹配');
    % 3. RNG_Seed
    assert(isequal(T_read.RNG_Seed, seed_all), 'Col 3 RNG_Seed 回读不匹配');
    % 4. True_Bias_L_count
    assert(max(abs(T_read.True_Bias_L_count - true_L_all)) < 1e-9, 'Col 4 True_Bias_L 回读不匹配');
    % 5. True_Bias_R_count
    assert(max(abs(T_read.True_Bias_R_count - true_R_all)) < 1e-9, 'Col 5 True_Bias_R 回读不匹配');
    % 6. Est_Bias_L_count
    assert(max(abs(T_read.Est_Bias_L_count - est_L_all)) < 1e-9, 'Col 6 Est_Bias_L 回读不匹配');
    % 7. Est_Bias_R_count
    assert(max(abs(T_read.Est_Bias_R_count - est_R_all)) < 1e-9, 'Col 7 Est_Bias_R 回读不匹配');
    % 8. Err_L_count
    assert(max(abs(T_read.Err_L_count - err_L_all)) < 1e-9, 'Col 8 Err_L 回读不匹配');
    % 9. Err_R_count
    assert(max(abs(T_read.Err_R_count - err_R_all)) < 1e-9, 'Col 9 Err_R 回读不匹配');
    % 10. Err_Trial_Max_count
    assert(max(abs(T_read.Err_Trial_Max_count - err_max_all)) < 1e-9, 'Col 10 Err_Trial_Max 回读不匹配');
    % 11. Is_Calibrated
    assert(all(T_read.Is_Calibrated == is_calib_all), 'Col 11 Is_Calibrated 回读不匹配');
    % 12. Calib_Mode
    assert(all(strcmp(T_read.Calib_Mode, mode_all)), 'Col 12 Calib_Mode 字符串回读不匹配');
    % 13. N_Eff_L
    assert(isequal(T_read.N_Eff_L, N_eff_L_all), 'Col 13 N_Eff_L 回读不匹配');
    % 14. N_Eff_R
    assert(isequal(T_read.N_Eff_R, N_eff_R_all), 'Col 14 N_Eff_R 回读不匹配');
    % 15. Within_P95_Threshold
    assert(isequal(T_read.Within_P95_Threshold, pass_p95_all), 'Col 15 Within_P95_Threshold 回读不匹配');
    fprintf('    [OK] CSV 全部 15 列 100%% 逐元素严格回读断言全数通过!\n\n');

    %% =====================================================================
    %% 总结输出
    %% =====================================================================
    fprintf('=========================================================================\n');
    fprintf('   Gate C4-A 单元测试验收结论: 全部 PASS\n');
    fprintf('=========================================================================\n');
    fprintf('   Subtest A1 (MC 100 标称):  P95(e_trial) = %.4f counts <= 2.0 counts [PASS]\n', p95_a1_trial);
    fprintf('   Subtest A2 (准入破坏 10 例): 非法准入累计更新次数 = 0             [PASS]\n');
    fprintf('   Subtest A3 (中途扰动):     有效计数清零，状态复位回退 IDLE       [PASS]\n');
    fprintf('   Subtest A4 (FROZEN锁定):   强运动工况参数漂移 < 1e-15 counts    [PASS]\n');
    fprintf('   Subtest A5 (样本不足):     有效计数 300 < 500 禁止进入 FROZEN    [PASS]\n');
    fprintf('   Subtest A6 (2%% 脉冲异常): P95(e_trial) = %.4f counts <= 2.0 counts [PASS]\n', p95_a6_trial);
    fprintf('=========================================================================\n');
end
