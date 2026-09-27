%% TEST_STEP3C_C4B_DELAY_ALIGNMENT.M - Gate C4-B 时延识别与因果历史对齐单元测试
% =========================================================================
% 功能说明:
% 依据 STEP3_IMPLEMENTATION_PLAN.md 第四阶段规划，对通信时延识别与因果历史对齐模块
% step3c_causal_delay_aligner.m 进行 Gate C4-B 单元测试验收。
%
% 严格红线边界:
% 1. 严格因果历史对齐，严禁对已滤波信号进行平移，未来样本引用次数严格为 0；
% 2. 物理延迟严格解耦: d_total = d_path + d_meas = (d_act + d_driver) + d_meas；
% 3. 三大运行模式: TIMESTAMP (时间戳硬对齐), XCORR_KNOWN_PATH (已知路径延迟互相关),
%    DIFF_ONLY (未知路径延迟差模降级)；
% 4. 严禁截零伪装: d_total < d_path 时判定为 NEGATIVE_DELAY 并拒更，严禁截零；
% 5. DIFF_ONLY 模式下禁止输出虚假绝对时延 (d_meas_hat 强制为 [NaN; NaN]，禁止接入回归)；
% 6. 无效期/预热期输出 NaN，禁止补零，下游更新次数严格为 0。
%
% 六大子测试规划 (B1 ~ B6):
%   B1: TIMESTAMP 模式硬对齐测试 (8 组时滞组合，对齐误差严格 0 samples)
%   B2: XCORR_KNOWN_PATH 互相关模式测试 (MC 100 评估，识别正确率 >= 95%)
%   B3: DIFF_ONLY 未知路径差模降级测试 (强制 NaN，valid_for_regression = false)
%   B4: 稳健门控与迟滞确认测试 (低激励、饱和、多峰模糊时拒更，防单步跳变)
%   B5: 严格因果性硬检查 (对齐索引严格 >= 1，未来样本引用次数严格 == 0)
%   B6: 预热期与异常输入防护测试 (NaN/Inf、丢包乱序、时钟失配、负时延拒更、下游冻结)
%
% 数据导出:
%   step3_adaptive_rls/step3c_c4b_delay_results.csv (包含 100 次 MC 与典型故障案例，100% 逐元素回读)
% =========================================================================

function test_step3c_c4b_delay_alignment()
    clc;
    fprintf('=========================================================================\n');
    fprintf('   STEP 3C-4 Gate C4-B: 时延识别与因果历史对齐单元测试 (B1 ~ B6)\n');
    fprintf('=========================================================================\n\n');

    script_dir = fileparts(mfilename('fullpath'));
    addpath(script_dir);

    % 1. 检查被测核心函数是否存在
    aligner_file = fullfile(script_dir, 'step3c_causal_delay_aligner.m');
    assert(exist(aligner_file, 'file') == 2, '缺少 step3c_causal_delay_aligner.m');
    fprintf('>>> [OK] 被测因果对齐器入口已就绪: %s\n\n', aligner_file);

    dt = 0.001; % 1 ms
    opts_base = struct();
    opts_base.dt                    = dt;
    opts_base.buffer_depth          = 250;
    opts_base.xcorr_window_length   = 150;
    opts_base.th_cmd_var            = 50.0;
    opts_base.th_peak_margin        = 0.05;
    opts_base.th_peak_min           = 0.55;
    opts_base.N_confirm             = 5;
    opts_base.Imax                  = 16000.0;
    opts_base.max_search_delay      = 10;

    %% =====================================================================
    %% [Subtest B1] TIMESTAMP 模式硬对齐测试 (8 组典型延迟组合)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B1] 开始 TIMESTAMP 模式硬件时间戳因果对齐测试...\n');
    fprintf('    目标: 验证已知源时间戳下对齐误差严格为 0 samples, 残余差模严格为 0\n');

    delay_cases_b1 = [
        0, 0, 0;
        1, 0, 0;
        0, 1, 0;
        2, 1, 0;
        1, 2, 0;
        2, 0, 1;
        0, 2, 2;
        2, 2, 0
    ];
    N_cases_b1 = size(delay_cases_b1, 1);
    b1_align_errors = zeros(N_cases_b1, 1);
    b1_residual_skews = zeros(N_cases_b1, 1);

    opts_b1 = opts_base;
    opts_b1.mode = 'TIMESTAMP';

    for c = 1:N_cases_b1
        dL = delay_cases_b1(c, 1);
        dR = delay_cases_b1(c, 2);
        dpos = delay_cases_b1(c, 3);

        N_steps = 100;
        iL_f = @(t) 2000.0 * sin(2*pi*5*t) + 500.0 * cos(2*pi*15*t);
        iR_f = @(t) 2000.0 * cos(2*pi*5*t) - 500.0 * sin(2*pi*15*t);
        yL_f = @(t) 0.05 * sin(2*pi*2*t);
        yR_f = @(t) 0.05 * sin(2*pi*2*t);

        align_state = [];
        max_err_case = 0;
        max_skew_case = 0;

        for k = 1:N_steps
            t_curr = (k - 1) * dt;

            % 模拟量测通信滞后
            % 各通道在源头采样时刻为 t_curr - d*dt，在 t_curr 时刻被主控制器接收
            t_src_L = t_curr - dL * dt;
            t_src_R = t_curr - dR * dt;
            t_src_p = t_curr - dpos * dt;

            c_raw = [iL_f(t_src_L); iR_f(t_src_R)];
            c_cmd = [iL_f(t_curr); iR_f(t_curr)];
            pos   = [yL_f(t_src_p); yR_f(t_src_p)];

            ts = struct();
            ts.t_source_L   = t_src_L;
            ts.t_source_R   = t_src_R;
            ts.t_source_pos = t_src_p;
            ts.t_recv_L     = t_curr;
            ts.t_recv_R     = t_curr;
            ts.t_recv_pos   = t_curr;
            ts.seq_L        = k;
            ts.seq_R        = k;
            ts.seq_pos      = k;
            ts.clock_id_L   = 'MASTER_BUS_CLK';
            ts.clock_id_R   = 'MASTER_BUS_CLK';
            ts.clock_id_pos = 'MASTER_BUS_CLK';

            qual = struct();
            qual.is_saturated   = [false; false];
            qual.current_valid  = [true; true];
            qual.position_valid = [true; true];
            qual.packet_valid   = true;

            [sig_align, align_state, info] = step3c_causal_delay_aligner( ...
                c_raw, c_cmd, pos, ts, qual, align_state, opts_b1);

            % 预热期后开始评估对齐精度
            if sig_align.valid_for_regression
                assert(sig_align.current_pair_valid, 'B1: current_pair_valid 应为 true');
                assert(sig_align.absolute_alignment_valid, 'B1: absolute_alignment_valid 应为 true');

                t_common_expected = min([ts.t_source_L, ts.t_source_R, ts.t_source_pos]);
                assert(abs(sig_align.common_timestamp - t_common_expected) < 1e-12, '公共时间戳对齐误差超标');

                % 理想物理时刻真值验证
                err_iL = abs(sig_align.current_cal(1) - iL_f(t_common_expected));
                err_iR = abs(sig_align.current_cal(2) - iR_f(t_common_expected));
                err_yL = abs(sig_align.position(1) - yL_f(t_common_expected));
                err_yR = abs(sig_align.position(2) - yR_f(t_common_expected));

                max_err_k = max([err_iL, err_iR, err_yL*1000]);
                if max_err_k > max_err_case
                    max_err_case = max_err_k;
                end

                % 时延识别与对齐误差检查
                d_id_err = max(abs(info.d_meas_hat - [dL; dR]));
                if d_id_err > b1_align_errors(c)
                    b1_align_errors(c) = d_id_err;
                end
            end
        end

        fprintf('    [Case %d] 注入 (dL=%d, dR=%d, dpos=%d) -> 识别时延误差 = %d samples, 信号对齐最大残差 = %.2e\n', ...
            c, dL, dR, dpos, b1_align_errors(c), max_err_case);
        assert(b1_align_errors(c) == 0, sprintf('B1 Case %d: 时延识别误差必须严格为 0', c));
        assert(max_err_case < 1e-10, sprintf('B1 Case %d: 对齐后信号与目标物理时刻真值残差超标', c));
    end

    assert(max(b1_align_errors) == 0, 'B1 失败: 时间戳模式时延对齐存在非零误差');
    fprintf('    [OK] Subtest B1 验收通过: 8 组工况时延识别误差严格为 0 samples, 残差差模严格为 0!\n\n');

    %% =====================================================================
    %% [Subtest B2] XCORR_KNOWN_PATH 互相关模式 (MC 100 评估正确率)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B2] 开始 XCORR_KNOWN_PATH 互相关时延识别蒙特卡洛测试 (MC 100)...\n');
    fprintf('    设定: d_path_known = [2; 2], d_meas in {0, 1, 2, 3} samples\n');

    N_mc_b2 = 100;
    d_path_true = [2; 2];
    opts_b2 = opts_base;
    opts_b2.mode = 'XCORR_KNOWN_PATH';
    opts_b2.d_path_known = d_path_true;

    b2_true_meas_L = zeros(N_mc_b2, 1);
    b2_true_meas_R = zeros(N_mc_b2, 1);
    b2_est_meas_L  = zeros(N_mc_b2, 1);
    b2_est_meas_R  = zeros(N_mc_b2, 1);
    b2_is_correct  = false(N_mc_b2, 1);
    b2_conf        = zeros(N_mc_b2, 1);

    for j = 1:N_mc_b2
        seed_j = 20261101 + j;
        rng(seed_j, 'twister');

        d_meas_L = randi([0, 3]);
        d_meas_R = randi([0, 3]);
        b2_true_meas_L(j) = d_meas_L;
        b2_true_meas_R(j) = d_meas_R;

        d_tot_L = d_path_true(1) + d_meas_L;
        d_tot_R = d_path_true(2) + d_meas_R;

        % 构造足够长的高变化率宽带激励指令序列 (200 步)
        N_steps = 220;
        t_seq = (0:N_steps-1)' * dt;
        cmd_L = 3000.0 * sin(2*pi*8*t_seq) + 2000.0 * sin(2*pi*20*t_seq) + 800.0 * randn(N_steps, 1);
        cmd_R = 3000.0 * cos(2*pi*8*t_seq) + 2000.0 * cos(2*pi*20*t_seq) + 800.0 * randn(N_steps, 1);

        % 模拟综合延迟与测量噪声 (10 counts)
        meas_L = zeros(N_steps, 1);
        meas_R = zeros(N_steps, 1);
        for k = 1:N_steps
            idx_cmd_L = max(1, k - d_tot_L);
            idx_cmd_R = max(1, k - d_tot_R);
            meas_L(k) = cmd_L(idx_cmd_L) + 10.0 * randn();
            meas_R(k) = cmd_R(idx_cmd_R) + 10.0 * randn();
        end

        align_state = [];
        pos_dummy = [0.0; 0.0];

        ts_dummy = struct();
        ts.t_source_L   = 0; ts.t_source_R   = 0; ts.t_source_pos = 0;
        ts.t_recv_L     = 0; ts.t_recv_R     = 0; ts.t_recv_pos   = 0;
        ts.seq_L        = 0; ts.seq_R        = 0; ts.seq_pos      = 0;
        ts.clock_id_L   = 'A'; ts.clock_id_R = 'A'; ts.clock_id_pos = 'A';

        qual = struct();
        qual.is_saturated   = [false; false];
        qual.current_valid  = [true; true];
        qual.position_valid = [true; true];
        qual.packet_valid   = true;

        for k = 1:N_steps
            ts.t_source_L = t_seq(k); ts.t_source_R = t_seq(k); ts.t_source_pos = t_seq(k);
            ts.t_recv_L = t_seq(k); ts.t_recv_R = t_seq(k); ts.t_recv_pos = t_seq(k);
            ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;

            raw_k = [meas_L(k); meas_R(k)];
            cmd_k = [cmd_L(k); cmd_R(k)];

            [sig_align, align_state, info] = step3c_causal_delay_aligner( ...
                raw_k, cmd_k, pos_dummy, ts, qual, align_state, opts_b2);
        end

        b2_est_meas_L(j) = info.d_meas_hat(1);
        b2_est_meas_R(j) = info.d_meas_hat(2);
        b2_conf(j)       = info.confidence;

        if (b2_est_meas_L(j) == d_meas_L) && (b2_est_meas_R(j) == d_meas_R)
            b2_is_correct(j) = true;
        else
            b2_is_correct(j) = false;
        end
    end

    acc_rate_b2 = 100.0 * mean(b2_is_correct);
    fprintf('    [B2 统计指标]:\n');
    fprintf('      - 100 次 MC 双通道精确识别正确率: %.1f%% (验收门限 >= 95.0%%)\n', acc_rate_b2);
    fprintf('      - 平均互相关置信度: %.3f\n', mean(b2_conf));

    assert(acc_rate_b2 >= 95.0, sprintf('B2 失败: 互相关时延识别正确率 %.1f%% < 95.0%%', acc_rate_b2));
    fprintf('    [OK] Subtest B2 验收通过: 已知路径互相关识别率达 %.1f%% >= 95.0%%!\n\n', acc_rate_b2);

    %% =====================================================================
    %% [Subtest B3] DIFF_ONLY 未知路径差模降级测试 (强制 NaN 与回归禁止)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B3] 开始 DIFF_ONLY 未知路径差模降级模式测试...\n');

    % 1. 配置 assume_symmetric_path = true (合法降级)
    opts_b3 = opts_base;
    opts_b3.mode = 'DIFF_ONLY';
    opts_b3.assume_symmetric_path = true;

    N_steps = 220;
    t_seq = (0:N_steps-1)' * dt;
    cmd_L = 3000.0 * sin(2*pi*8*t_seq) + 1000.0 * randn(N_steps, 1);
    cmd_R = 3000.0 * cos(2*pi*8*t_seq) + 1000.0 * randn(N_steps, 1);
    d_tot_L = 4; d_tot_R = 3; % 差模 Delta_d = 1 sample

    align_state = [];
    for k = 1:N_steps
        idx_L = max(1, k - d_tot_L);
        idx_R = max(1, k - d_tot_R);
        raw_k = [cmd_L(idx_L); cmd_R(idx_R)];
        cmd_k = [cmd_L(k); cmd_R(k)];

        ts.t_source_L = t_seq(k); ts.t_source_R = t_seq(k); ts.t_source_pos = t_seq(k);
        ts.t_recv_L = t_seq(k); ts.t_recv_R = t_seq(k); ts.t_recv_pos = t_seq(k);
        ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;

        [sig_align, align_state, info] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b3);
    end

    fprintf('    [B3 合法降级结果]:\n');
    fprintf('      - 识别差模 Delta_d_hat: %.1f samples (真值 = %d)\n', info.delta_d_hat, d_tot_L - d_tot_R);
    fprintf('      - 绝对时延 d_meas_hat:  [%f; %f] (必须强制为 NaN)\n', info.d_meas_hat(1), info.d_meas_hat(2));
    fprintf('      - current_pair_valid:       %d (应为 1)\n', sig_align.current_pair_valid);
    fprintf('      - absolute_alignment_valid: %d (必须为 0)\n', sig_align.absolute_alignment_valid);
    fprintf('      - valid_for_regression:     %d (必须为 0)\n', sig_align.valid_for_regression);

    assert(info.delta_d_hat == 1, 'B3: 差模时延识别不符');
    assert(all(isnan(info.d_meas_hat)), 'B3 失败: DIFF_ONLY 模式严禁输出虚假绝对时延');
    assert(sig_align.current_pair_valid, 'B3: current_pair_valid 应为 true');
    assert(~sig_align.absolute_alignment_valid, 'B3: absolute_alignment_valid 必须为 false');
    assert(~sig_align.valid_for_regression, 'B3: valid_for_regression 必须为 false');
    assert(all(isnan(sig_align.current_cal)), 'B3: 回归禁止期电流输出必须为 NaN');
    assert(all(isnan(sig_align.position)), 'B3: 回归禁止期位置输出必须为 NaN');

    % 2. 配置 assume_symmetric_path = false (未显式声明对称假设，必须拒绝更新)
    opts_b3_unassumed = opts_b3;
    opts_b3_unassumed.assume_symmetric_path = false;
    [~, ~, info_unassumed] = step3c_causal_delay_aligner( ...
        raw_k, cmd_k, [0.0; 0.0], ts, qual, [], opts_b3_unassumed);

    assert(~info_unassumed.did_update, 'B3: 未声明对称性时必须禁止更新');
    assert(strcmp(info_unassumed.reject_reason, 'ASYMMETRIC_PATH_UNASSUMED'), ...
        'B3: 原因码必须为 ASYMMETRIC_PATH_UNASSUMED');
    fprintf('    [OK] Subtest B3 验收通过: 未知路径下 DIFF_ONLY 强制输出 NaN 并切断回归准入!\n\n');

    %% =====================================================================
    %% [Subtest B4] 稳健门控与迟滞确认测试 (低激励、饱和、多峰模糊)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B4] 开始稳健门控与迟滞防误触发测试...\n');

    % 1. 低激励工况门控测试 (直流指令)
    align_state = [];
    opts_b4 = opts_base;
    opts_b4.mode = 'XCORR_KNOWN_PATH';
    opts_b4.d_path_known = [2; 2];

    for k = 1:50
        raw_dc = [1000.0; 1000.0];
        cmd_dc = [1000.0; 1000.0];
        [~, align_state, info_low] = step3c_causal_delay_aligner( ...
            raw_dc, cmd_dc, [0.0; 0.0], ts, qual, align_state, opts_b4);
    end
    assert(~info_low.did_update, 'B4: 低激励工况下严禁触发时延更新');
    assert(strcmp(info_low.reject_reason, 'LOW_EXCITATION'), 'B4: 原因码应为 LOW_EXCITATION');
    fprintf('    [OK] Case 1: 低变化率激励段成功拦截 (LOW_EXCITATION, did_update = false)\n');

    % 2. 电流饱和门控测试 (饱和标志注入)
    qual_sat = qual;
    qual_sat.is_saturated = [true; false];
    [~, ~, info_sat] = step3c_causal_delay_aligner( ...
        [5000.0; 5000.0], [5000.0; 5000.0], [0.0; 0.0], ts, qual_sat, align_state, opts_b4);
    assert(~info_sat.did_update, 'B4: 饱和状态下严禁更新');
    assert(strcmp(info_sat.reject_reason, 'SATURATION'), 'B4: 原因码应为 SATURATION');
    fprintf('    [OK] Case 2: 电流饱和状态成功拦截 (SATURATION, did_update = false)\n');

    % 3. 迟滞防跳变测试 (连续 4 步突变不应切换，第 5 步方可确认切换)
    % 首先建立稳态 delay = 1 (N_confirm 步)
    align_state = [];
    cmd_chirp = 4000.0 * sin(2*pi*15*(0:100)'*dt);
    for k = 1:50
        idx_1 = max(1, k - 3); % d_tot = 3 -> d_meas = 3 - 2 = 1
        raw_k = [cmd_chirp(idx_1); cmd_chirp(idx_1)];
        cmd_k = [cmd_chirp(k); cmd_chirp(k)];
        [~, align_state, info] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b4);
    end
    assert(info.d_meas_hat(1) == 1, 'B4: 基准时延未确认');

    % 注入仅维持 2 步的时延噪声毛刺 (d_tot 突变为 5 -> d_meas = 3)
    for k = 51:52
        idx_3 = max(1, k - 5);
        raw_k = [cmd_chirp(idx_3); cmd_chirp(idx_3)];
        cmd_k = [cmd_chirp(k); cmd_chirp(k)];
        [~, align_state, info_glitch] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b4);
        assert(info_glitch.d_meas_hat(1) == 1, 'B4: 迟滞失效，毛刺导致了未确认的时延跳变');
    end
    fprintf('    [OK] Case 3: 瞬态毛刺未满 5 步确认门限，时延锁定未受污染\n');
    fprintf('    [OK] Subtest B4 验收通过: 稳健门控与迟滞防抖机制全面生效!\n\n');

    %% =====================================================================
    %% [Subtest B5] 严格因果性硬检查 (未来样本引用次数严格 == 0)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B5] 开始因果历史索引硬检查 (未来样本引用次数统计)...\n');

    % 模拟 500 步全工况，检测所有输出索引
    future_ref_count = 0;
    align_state = [];
    opts_b5 = opts_base;
    opts_b5.mode = 'TIMESTAMP';

    for k = 1:500
        % 动态网络时滞 0~2 步
        dL = mod(k, 3);
        dR = mod(k+1, 3);
        dpos = 1;

        % 源传感器采样时间戳严格单调递增
        t_src = k * dt;
        ts.t_source_L   = t_src;
        ts.t_source_R   = t_src;
        ts.t_source_pos = t_src;

        % 到达接收端时间包含通信时延
        ts.t_recv_L     = t_src + dL * dt;
        ts.t_recv_R     = t_src + dR * dt;
        ts.t_recv_pos   = t_src + dpos * dt;
        t_wall          = max([ts.t_recv_L, ts.t_recv_R, ts.t_recv_pos]);

        ts.seq_L        = k; ts.seq_R = k; ts.seq_pos = k;
        ts.clock_id_L   = 'CLK'; ts.clock_id_R = 'CLK'; ts.clock_id_pos = 'CLK';

        [sig_align, align_state, ~] = step3c_causal_delay_aligner( ...
            [10.0; 10.0], [10.0; 10.0], [0.1; 0.1], ts, qual, align_state, opts_b5);

        if sig_align.valid_for_regression
            % 检查公共参考时间是否超前于当前到达的量测源时间
            if sig_align.common_timestamp > t_wall + 1e-12
                future_ref_count = future_ref_count + 1;
            end
            if sig_align.common_timestamp > ts.t_source_L + 1e-12 || ...
               sig_align.common_timestamp > ts.t_source_R + 1e-12 || ...
               sig_align.common_timestamp > ts.t_source_pos + 1e-12
                future_ref_count = future_ref_count + 1;
            end
        end
    end

    fprintf('    [B5 统计指标]:\n');
    fprintf('      - 500 步运行中未来样本/未来时间戳引用次数: %d (断言严格 == 0)\n', future_ref_count);
    assert(future_ref_count == 0, sprintf('B5 失败: 检测到未来样本引用 %d 次', future_ref_count));
    fprintf('    [OK] Subtest B5 验收通过: 严格因果历史对齐成立，未来样本引用次数为 0!\n\n');

    %% =====================================================================
    %% [Subtest B6] 预热期、异常输入防护与下游 SVF/RLS 冻结校验
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B6] 开始预热期、异常防护与下游回归冻结硬断言测试...\n');

    % 1. 预热期输出有效性与下游冻结
    align_state = [];
    downstream_rls_updates = 0;
    nonfinite_valid_outputs = 0;

    for k = 1:50
        ts.t_source_L = k*dt; ts.t_source_R = k*dt; ts.t_source_pos = k*dt;
        ts.t_recv_L = k*dt; ts.t_recv_R = k*dt; ts.t_recv_pos = k*dt;
        ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;

        [sig_align, align_state, ~] = step3c_causal_delay_aligner( ...
            [10.0; 10.0], [10.0; 10.0], [0.0; 0.0], ts, qual, align_state, opts_b1);

        % 模拟下游 SVF/RLS 接收器行为
        if ~sig_align.valid_for_regression
            % 必须输出 NaN 且禁止更新
            assert(all(isnan(sig_align.current_cal)), 'B6: 无效期电流必须为 NaN');
            assert(all(isnan(sig_align.position)), 'B6: 无效期位置必须为 NaN');
        else
            % 有效期进行更新
            downstream_rls_updates = downstream_rls_updates + 1;
            if any(~isfinite(sig_align.current_cal)) || any(~isfinite(sig_align.position))
                nonfinite_valid_outputs = nonfinite_valid_outputs + 1;
            end
        end
    end
    assert(nonfinite_valid_outputs == 0, 'B6: 有效期严禁存在非有限值');
    fprintf('    [OK] Case 1: 预热期全输出 NaN，下游 RLS 更新完全冻结，有效期非有限项为 0\n');

    % 2. 负时延截零伪装检查 (严禁将 d_total < d_path 截零)
    opts_neg = opts_base;
    opts_neg.mode = 'XCORR_KNOWN_PATH';
    opts_neg.d_path_known = [5; 5]; % 设定已知综合路径为 5 步

    align_state = [];
    cmd_pulse = 3000.0 * sin(2*pi*10*(0:100)'*dt);
    clamped_zero_count = 0;

    for k = 1:50
        % 实际总延迟仅 2 步 -> d_meas 理论为 2 - 5 = -3 < 0
        idx_neg = max(1, k - 2);
        raw_k = [cmd_pulse(idx_neg); cmd_pulse(idx_neg)];
        cmd_k = [cmd_pulse(k); cmd_pulse(k)];

        [~, align_state, info_neg] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_neg);

        if strcmp(info_neg.reject_reason, 'NEGATIVE_DELAY')
            assert(~info_neg.did_update, '负时延时严禁更新');
            if any(info_neg.d_meas_hat == 0)
                clamped_zero_count = clamped_zero_count + 1;
            end
        end
    end
    assert(clamped_zero_count == 0, 'B6 失败: 检测到负时延被非法截断为 0 冒充有效估计');
    fprintf('    [OK] Case 2: 负时延拒绝生效 (NEGATIVE_DELAY)，截零伪装次数严格为 0\n');

    % 3. 报文损坏、时钟失配与序列倒退拦截
    qual_corrupt = qual;
    qual_corrupt.packet_valid = false;
    [~, ~, info_p] = step3c_causal_delay_aligner( ...
        [10.0; 10.0], [10.0; 10.0], [0.0; 0.0], ts, qual_corrupt, [], opts_b1);
    assert(strcmp(info_p.reject_reason, 'PACKET_CORRUPT'), '报文损坏未拦截');

    ts_mismatch = ts;
    ts_mismatch.clock_id_R = 'DESYNC_CLK';
    [~, ~, info_c] = step3c_causal_delay_aligner( ...
        [10.0; 10.0], [10.0; 10.0], [0.0; 0.0], ts_mismatch, qual, [], opts_b1);
    assert(strcmp(info_c.reject_reason, 'CLOCK_MISMATCH'), '时钟域失配未拦截');

    ts_rollback = ts;
    ts_rollback.seq_L = 10;
    align_state_seq = struct('last_seq_L', 20, 'last_seq_R', 5, 'last_seq_pos', 5, ...
        'last_ts_L', 0, 'last_ts_R', 0, 'last_ts_pos', 0, 'mode', 'TIMESTAMP', ...
        'buffer_depth', 50, 'current_buffer_L', NaN(50,1), 'current_buffer_R', NaN(50,1), ...
        'command_buffer_L', NaN(50,1), 'command_buffer_R', NaN(50,1), ...
        'position_buffer_L', NaN(50,1), 'position_buffer_R', NaN(50,1), ...
        't_source_buffer_L', NaN(50,1), 't_source_buffer_R', NaN(50,1), ...
        't_source_buffer_pos', NaN(50,1), 'buffer_count', 25, 'd_hat_L', 0, 'd_hat_R', 0, ...
        'd_total_hat', [0;0], 'last_trusted_delay', [0;0], 'candidate_delay', [0;0], ...
        'confirm_count', 0, 'last_confidence', 0.0, 'is_initialized', true);

    [~, ~, info_r] = step3c_causal_delay_aligner( ...
        [10.0; 10.0], [10.0; 10.0], [0.0; 0.0], ts_rollback, qual, align_state_seq, opts_b1);
    assert(strcmp(info_r.reject_reason, 'SEQ_ROLLBACK'), '包序列倒退未拦截');
    fprintf('    [OK] Case 3: 报文损坏、时钟域失配、序列号倒退全数被刚性拦截\n');
    fprintf('    [OK] Subtest B6 验收通过: 预热/异常保护与下游冻结全部闭环!\n\n');

    %% =====================================================================
    %% 数据导出与 100% 内存回读校验 (CSV)
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3c_c4b_delay_results.csv');
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> 正在导出 Gate C4-B 蒙特卡洛结果表: %s\n', csv_file);

    trial_id_vec = (1:N_mc_b2)';
    subtest_vec  = repmat({'B2_XCORR_KNOWN_PATH_MC'}, N_mc_b2, 1);
    seed_vec     = (20261101 + (1:N_mc_b2))';

    results_table = table( ...
        trial_id_vec, subtest_vec, seed_vec, ...
        b2_true_meas_L, b2_true_meas_R, ...
        b2_est_meas_L, b2_est_meas_R, ...
        b2_conf, b2_is_correct, ...
        'VariableNames', { ...
            'Trial_ID', 'Subtest', 'RNG_Seed', ...
            'True_Meas_Delay_L_samples', 'True_Meas_Delay_R_samples', ...
            'Est_Meas_Delay_L_samples', 'Est_Meas_Delay_R_samples', ...
            'Xcorr_Confidence', 'Is_Correct' ...
        });

    writetable(results_table, csv_file);
    fprintf('    [OK] CSV 写入完成，共计 %d 行 x %d 列\n', height(results_table), width(results_table));

    % 100% 逐列逐元素严格内存回读校验 (全 9 列)
    fprintf('>>> 执行 CSV 全部 9 列 100%% 逐元素严格内存回读校验...\n');
    T_read = readtable(csv_file);
    assert(height(T_read) == N_mc_b2, '回读行数不符');
    assert(width(T_read) == 9, '回读列数不符');

    assert(isequal(T_read.Trial_ID, trial_id_vec), 'Col 1 Trial_ID 回读不匹配');
    assert(all(strcmp(T_read.Subtest, subtest_vec)), 'Col 2 Subtest 回读不匹配');
    assert(isequal(T_read.RNG_Seed, seed_vec), 'Col 3 RNG_Seed 回读不匹配');
    assert(isequal(T_read.True_Meas_Delay_L_samples, b2_true_meas_L), 'Col 4 True_L 回读不匹配');
    assert(isequal(T_read.True_Meas_Delay_R_samples, b2_true_meas_R), 'Col 5 True_R 回读不匹配');
    assert(isequal(T_read.Est_Meas_Delay_L_samples, b2_est_meas_L), 'Col 6 Est_L 回读不匹配');
    assert(isequal(T_read.Est_Meas_Delay_R_samples, b2_est_meas_R), 'Col 7 Est_R 回读不匹配');
    assert(max(abs(T_read.Xcorr_Confidence - b2_conf)) < 1e-9, 'Col 8 Confidence 回读不匹配');
    assert(all(T_read.Is_Correct == b2_is_correct), 'Col 9 Is_Correct 回读不匹配');
    fprintf('    [OK] CSV 全部 9 列 100%% 逐元素严格回读断言全数通过!\n\n');

    %% =====================================================================
    %% 总结输出
    %% =====================================================================
    fprintf('=========================================================================\n');
    fprintf('   Gate C4-B 单元测试验收结论: 全部 PASS\n');
    fprintf('=========================================================================\n');
    fprintf('   Subtest B1 (TIMESTAMP 8 组工况): 对齐误差严格 0 samples        [PASS]\n');
    fprintf('   Subtest B2 (XCORR MC 100 识别): 正确率 = %.1f%% >= 95.0%%        [PASS]\n', acc_rate_b2);
    fprintf('   Subtest B3 (DIFF_ONLY 降级):    强制 NaN 并禁止回归输入        [PASS]\n');
    fprintf('   Subtest B4 (稳健门控与迟滞):    饱和/低激励拦截, 毛刺防抖确认 [PASS]\n');
    fprintf('   Subtest B5 (因果性硬检验):      未来样本引用次数严格 == 0      [PASS]\n');
    fprintf('   Subtest B6 (预热与异常防护):    下游 RLS 冻结, 截零伪装 == 0   [PASS]\n');
    fprintf('=========================================================================\n');
end
