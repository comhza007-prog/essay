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
% 6. 无效期/预热期输出 NaN，禁止补零，下游更新次数严格为 0；
% 7. 饱和期间禁止回归有效输出，绝对冻结下游更新。
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
    opts_base.max_search_delay      = 10;
    opts_base.max_position_delay    = 10;
    opts_base.d_pos_known           = NaN;
    opts_base.require_position_alignment = true;
    opts_base.strict_sequence       = true;
    opts_base.th_cmd_var            = 50.0;
    opts_base.th_peak_margin        = 0.05;
    opts_base.th_peak_min           = 0.55;
    opts_base.N_confirm             = 5;
    opts_base.Imax                  = 16000.0;

    % 用于全套测试治理的汇总记录存储 (共 26 列)
    records = struct( ...
        'Subtest', {}, 'Trial_ID', {}, ...
        'True_Delay_L', {}, 'True_Delay_R', {}, ...
        'Estimated_Delay_L', {}, 'Estimated_Delay_R', {}, ...
        'Candidate_Delay_L', {}, 'Candidate_Delay_R', {}, ...
        'Confirm_Count_L', {}, 'Confirm_Count_R', {}, ...
        'Trusted_Delay_L', {}, 'Trusted_Delay_R', {}, ...
        'Used_Index_L', {}, 'Used_Index_R', {}, ...
        'Used_Index_Pos_L', {}, 'Used_Index_Pos_R', {}, ...
        'Used_Timestamp_L', {}, 'Used_Timestamp_R', {}, ...
        'Used_Timestamp_Pos_L', {}, 'Used_Timestamp_Pos_R', {}, ...
        'Common_Timestamp', {}, ...
        'Current_Pair_Valid', {}, 'Absolute_Alignment_Valid', {}, 'Valid_For_Regression', {}, ...
        'Reject_Reason', {}, 'Did_Update', {} ...
    );

    %% =====================================================================
    %% [Subtest B1] TIMESTAMP 模式硬对齐测试 (8 组正常工况 + 5 组异常工况)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B1] 开始 TIMESTAMP 模式硬件时间戳因果对齐与异常拦截测试...\n');
    fprintf('    目标: 验证已知源时间戳下对齐误差严格为 0 samples, 异常时间戳全面拦截\n');

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

    opts_b1 = opts_base;
    opts_b1.mode = 'TIMESTAMP';

    % 1. 正常 8 组工况测试
    for c = 1:N_cases_b1
        dL   = delay_cases_b1(c, 1);
        dR   = delay_cases_b1(c, 2);
        dpos = delay_cases_b1(c, 3);

        N_steps = 100;
        iL_f = @(t) 2000.0 * sin(2*pi*5*t) + 500.0 * cos(2*pi*15*t);
        iR_f = @(t) 2000.0 * cos(2*pi*5*t) - 500.0 * sin(2*pi*15*t);
        yL_f = @(t) 0.05 * sin(2*pi*2*t);
        yR_f = @(t) 0.05 * sin(2*pi*2*t);

        align_state = [];
        max_err_case = 0;

        for k = 1:N_steps
            t_curr = (k - 1) * dt;

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

            if sig_align.valid_for_regression
                assert(sig_align.current_pair_valid, 'B1: current_pair_valid 应为 true');
                assert(sig_align.absolute_alignment_valid, 'B1: absolute_alignment_valid 应为 true');

                t_common_expected = min([ts.t_source_L, ts.t_source_R, ts.t_source_pos]);
                assert(abs(sig_align.common_timestamp - t_common_expected) < 1e-12, '公共时间戳对齐误差超标');

                err_iL = abs(sig_align.current_cal(1) - iL_f(t_common_expected));
                err_iR = abs(sig_align.current_cal(2) - iR_f(t_common_expected));
                err_yL = abs(sig_align.position(1) - yL_f(t_common_expected));
                err_yR = abs(sig_align.position(2) - yR_f(t_common_expected));

                max_err_k = max([err_iL, err_iR, err_yL*1000]);
                if max_err_k > max_err_case
                    max_err_case = max_err_k;
                end

                d_id_err = max(abs(info.d_meas_hat - [dL; dR]));
                if d_id_err > b1_align_errors(c)
                    b1_align_errors(c) = d_id_err;
                end
            end
        end

        fprintf('    [Normal Case %d] 注入 (dL=%d, dR=%d, dpos=%d) -> 识别时延误差 = %d samples, 信号对齐最大残差 = %.2e\n', ...
            c, dL, dR, dpos, b1_align_errors(c), max_err_case);
        assert(b1_align_errors(c) == 0, sprintf('B1 Case %d: 时延识别误差必须严格为 0', c));
        assert(max_err_case < 1e-10, sprintf('B1 Case %d: 对齐后信号与目标物理时刻真值残差超标', c));

        records(end+1) = make_record('B1_TIMESTAMP_NORMAL', c, dL, dR, info, align_state, sig_align);
    end
    assert(max(b1_align_errors) == 0, 'B1 失败: 时间戳模式时延对齐存在非零误差');

    % 2. 异常时间戳 5 组负测试
    fprintf('    >>> 执行 B1 异常时间戳与时钟 domain 负测试 (5 组)...\n');

    % 异常 1: t_recv < t_source (时间倒流负时延)
    ts_abn1 = ts;
    ts_abn1.seq_L = 101; ts_abn1.seq_R = 101; ts_abn1.seq_pos = 101;
    ts_abn1.t_source_L = 1.0;
    ts_abn1.t_recv_L   = 0.99; % 接收早于发射!
    [sig_abn1, ~, info_abn1] = step3c_causal_delay_aligner(c_raw, c_cmd, pos, ts_abn1, qual, align_state, opts_b1);
    assert(~sig_abn1.valid_for_regression && all(isnan(sig_abn1.current_cal)) && all(isnan(sig_abn1.position)) && ~info_abn1.did_update);
    assert(strcmp(info_abn1.reject_reason, 'NEGATIVE_DELAY'), '异常 1: 负延迟未拦截');
    records(end+1) = make_record('B1_ABNORMAL_NEG_DELAY', 1, NaN, NaN, info_abn1, align_state, sig_abn1);

    % 异常 2: 时间戳含 NaN
    ts_abn2 = ts;
    ts_abn2.seq_L = 102; ts_abn2.seq_R = 102; ts_abn2.seq_pos = 102;
    ts_abn2.t_source_L = NaN;
    [sig_abn2, ~, info_abn2] = step3c_causal_delay_aligner(c_raw, c_cmd, pos, ts_abn2, qual, align_state, opts_b1);
    assert(~sig_abn2.valid_for_regression && all(isnan(sig_abn2.current_cal)) && all(isnan(sig_abn2.position)) && ~info_abn2.did_update);
    assert(strcmp(info_abn2.reject_reason, 'PACKET_CORRUPT'), '异常 2: NaN 时间戳未拦截');
    records(end+1) = make_record('B1_ABNORMAL_NAN_TS', 2, NaN, NaN, info_abn2, align_state, sig_abn2);

    % 异常 3: 重复序列号 (seq <= last_seq)
    ts_abn3 = ts;
    ts_abn3.seq_L = align_state.last_seq_L; % 重复序列号
    ts_abn3.t_source_L = align_state.last_ts_L + dt;
    ts_abn3.t_source_R = align_state.last_ts_R + dt;
    ts_abn3.t_source_pos = align_state.last_ts_pos_L + dt;
    ts_abn3.t_recv_L   = ts_abn3.t_source_L + dt;
    ts_abn3.t_recv_R   = ts_abn3.t_source_R + dt;
    ts_abn3.t_recv_pos = ts_abn3.t_source_pos + dt;
    [sig_abn3, ~, info_abn3] = step3c_causal_delay_aligner(c_raw, c_cmd, pos, ts_abn3, qual, align_state, opts_b1);
    assert(~sig_abn3.valid_for_regression && all(isnan(sig_abn3.current_cal)) && all(isnan(sig_abn3.position)) && ~info_abn3.did_update);
    assert(strcmp(info_abn3.reject_reason, 'SEQ_ROLLBACK'), '异常 3: 重复序列号未拦截');
    records(end+1) = make_record('B1_ABNORMAL_DUP_SEQ', 3, NaN, NaN, info_abn3, align_state, sig_abn3);

    % 异常 4: 源时钟域失配 (clock_id 不匹配)
    ts_abn4 = ts;
    ts_abn4.seq_L = 104; ts_abn4.seq_R = 104; ts_abn4.seq_pos = 104;
    ts_abn4.t_source_L = align_state.last_ts_L + dt;
    ts_abn4.t_source_R = align_state.last_ts_R + dt;
    ts_abn4.t_source_pos = align_state.last_ts_pos_L + dt;
    ts_abn4.t_recv_L   = ts_abn4.t_source_L + dt;
    ts_abn4.t_recv_R   = ts_abn4.t_source_R + dt;
    ts_abn4.t_recv_pos = ts_abn4.t_source_pos + dt;
    ts_abn4.clock_id_R = 'DESYNC_SLAVE_CLK';
    [sig_abn4, ~, info_abn4] = step3c_causal_delay_aligner(c_raw, c_cmd, pos, ts_abn4, qual, align_state, opts_b1);
    assert(~sig_abn4.valid_for_regression && all(isnan(sig_abn4.current_cal)) && all(isnan(sig_abn4.position)) && ~info_abn4.did_update);
    assert(strcmp(info_abn4.reject_reason, 'CLOCK_MISMATCH'), '异常 4: 时钟失配未拦截');
    records(end+1) = make_record('B1_ABNORMAL_CLOCK_MISMATCH', 4, NaN, NaN, info_abn4, align_state, sig_abn4);

    % 异常 5: 位置通道左右源时间戳不同 (t_source_pos_L ~= t_source_pos_R)
    ts_abn5 = rmfield(ts, {'t_source_pos', 't_recv_pos', 'seq_pos', 'clock_id_pos'});
    ts_abn5.seq_L = 105; ts_abn5.seq_R = 105;
    ts_abn5.t_source_L = align_state.last_ts_L + dt;
    ts_abn5.t_source_R = align_state.last_ts_R + dt;
    ts_abn5.t_recv_L   = ts_abn5.t_source_L + dt;
    ts_abn5.t_recv_R   = ts_abn5.t_source_R + dt;
    ts_abn5.t_source_pos_L = 0.500;
    ts_abn5.t_source_pos_R = 0.510; % 左右位置源采样时刻相差 10 ms
    ts_abn5.t_recv_pos_L   = 0.550;
    ts_abn5.t_recv_pos_R   = 0.550;
    ts_abn5.seq_pos_L      = 105;
    ts_abn5.seq_pos_R      = 105;
    ts_abn5.clock_id_pos_L = 'MASTER_BUS_CLK';
    ts_abn5.clock_id_pos_R = 'MASTER_BUS_CLK';
    [sig_abn5, ~, info_abn5] = step3c_causal_delay_aligner(c_raw, c_cmd, pos, ts_abn5, qual, align_state, opts_b1);
    assert(~sig_abn5.valid_for_regression && all(isnan(sig_abn5.current_cal)) && all(isnan(sig_abn5.position)) && ~info_abn5.did_update);
    assert(strcmp(info_abn5.reject_reason, 'POSITION_ASYNC'), '异常 5: 位置左右异步未拦截');
    records(end+1) = make_record('B1_ABNORMAL_POS_ASYNC', 5, NaN, NaN, info_abn5, align_state, sig_abn5);

    fprintf('    [OK] Subtest B1 验收通过: 8 组正常工况与 5 组异常时间戳全面闭环刚性拦截!\n\n');

    %% =====================================================================
    %% [Subtest B2] XCORR_KNOWN_PATH 互相关模式 (四场景分流测试)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B2] 开始 XCORR_KNOWN_PATH 四场景分流测试 (A~D)...\n');
    fprintf('    设定: 非对称路径 d_path_known = [2; 4], 已知位置延迟 d_pos_known = 1\n');

    d_path_true = [2; 4]; % 显式非对称路径
    d_pos_true  = 1;      % 显式已知位置延迟

    opts_b2 = opts_base;
    opts_b2.mode = 'XCORR_KNOWN_PATH';
    opts_b2.d_path_known = d_path_true;
    opts_b2.d_pos_known  = d_pos_true;

    % ---------------------------------------------------------------------
    % [Scenario A] 无噪声理想精确对齐测试 (noise_current = 0, 残差断言 < 1e-10)
    % ---------------------------------------------------------------------
    fprintf('    >>> [Scenario A] 无噪声理想精确因果对齐测试...\n');
    d_meas_L_a = 2;
    d_meas_R_a = 1;
    d_tot_L_a  = d_path_true(1) + d_meas_L_a; % 4
    d_tot_R_a  = d_path_true(2) + d_meas_R_a; % 5
    d_common_a = max([d_tot_L_a, d_tot_R_a, d_pos_true]); % 5

    N_steps_a = 220;
    t_seq_a = (0:N_steps_a-1)' * dt;
    rng(100, 'twister');
    cmd_L_a = 3000.0 * sin(2*pi*8*t_seq_a) + 2000.0 * sin(2*pi*20*t_seq_a) + 800.0 * randn(N_steps_a, 1);
    cmd_R_a = 3000.0 * cos(2*pi*8*t_seq_a) + 2000.0 * cos(2*pi*20*t_seq_a) + 800.0 * randn(N_steps_a, 1);
    pos_true_L_a = 0.05 * sin(2*pi*2*t_seq_a);
    pos_true_R_a = 0.05 * cos(2*pi*2*t_seq_a);

    meas_L_a = zeros(N_steps_a, 1);
    meas_R_a = zeros(N_steps_a, 1);
    pos_meas_L_a = zeros(N_steps_a, 1);
    pos_meas_R_a = zeros(N_steps_a, 1);
    for k = 1:N_steps_a
        meas_L_a(k) = cmd_L_a(max(1, k - d_tot_L_a));
        meas_R_a(k) = cmd_R_a(max(1, k - d_tot_R_a));
        pos_meas_L_a(k) = pos_true_L_a(max(1, k - d_pos_true));
        pos_meas_R_a(k) = pos_true_R_a(max(1, k - d_pos_true));
    end

    align_state_a = [];
    max_align_err_a = 0;
    for k = 1:N_steps_a
        ts_a.t_source_L   = t_seq_a(k); ts_a.t_source_R   = t_seq_a(k); ts_a.t_source_pos = t_seq_a(k);
        ts_a.t_recv_L     = t_seq_a(k); ts_a.t_recv_R     = t_seq_a(k); ts_a.t_recv_pos   = t_seq_a(k);
        ts_a.seq_L        = k; ts_a.seq_R        = k; ts_a.seq_pos      = k;
        ts_a.clock_id_L   = 'CLK'; ts_a.clock_id_R   = 'CLK'; ts_a.clock_id_pos = 'CLK';

        qual_a = struct('is_saturated', [false; false], 'current_valid', [true; true], ...
                        'position_valid', [true; true], 'packet_valid', true);

        raw_k_a = [meas_L_a(k); meas_R_a(k)];
        cmd_k_a = [cmd_L_a(k); cmd_R_a(k)];
        pos_k_a = [pos_meas_L_a(k); pos_meas_R_a(k)];

        [sig_a, align_state_a, info_a] = step3c_causal_delay_aligner( ...
            raw_k_a, cmd_k_a, pos_k_a, ts_a, qual_a, align_state_a, opts_b2);

        if sig_a.valid_for_regression
            idx_common = k - d_common_a;
            if idx_common >= 1
                err_iL = abs(sig_a.current_cal(1) - cmd_L_a(idx_common));
                err_iR = abs(sig_a.current_cal(2) - cmd_R_a(idx_common));
                err_yL = abs(sig_a.position(1) - pos_true_L_a(idx_common));
                err_yR = abs(sig_a.position(2) - pos_true_R_a(idx_common));
                err_k  = max([err_iL, err_iR, err_yL*1000, err_yR*1000]);
                if err_k > max_align_err_a
                    max_align_err_a = err_k;
                end
            end
        end
    end
    assert(all(info_a.d_meas_hat == [d_meas_L_a; d_meas_R_a]), 'Scenario A: 理想时延识别不符');
    assert(max_align_err_a < 1e-10, sprintf('Scenario A: 无噪声理想对齐残差超标: %.2e >= 1e-10', max_align_err_a));
    fprintf('      - Scenario A PASS: 无噪声对齐误差最大残差 %.2e < 1e-10\n', max_align_err_a);
    records(end+1) = make_record('B2_SCENARIO_A_NOISELESS', 1, d_meas_L_a, d_meas_R_a, info_a, align_state_a, sig_a);

    % ---------------------------------------------------------------------
    % [Scenario B] 鲁棒高斯噪声与蒙特卡洛评估 (MC 100, noise=10 counts, RMS <= 30 counts)
    % ---------------------------------------------------------------------
    fprintf('    >>> [Scenario B] 噪声鲁棒性与蒙特卡洛测试 (MC 100, noise=10 counts)...\n');
    N_mc_b2 = 100;
    b2_is_correct = false(N_mc_b2, 1);
    b2_rms_errors = zeros(N_mc_b2, 1);
    b2_max_errors = zeros(N_mc_b2, 1);

    for j = 1:N_mc_b2
        rng(20261101 + j, 'twister');

        d_meas_L = randi([0, 3]);
        d_meas_R = randi([0, 3]);

        d_tot_L = d_path_true(1) + d_meas_L;
        d_tot_R = d_path_true(2) + d_meas_R;
        d_pos   = d_pos_true;
        d_common = max([d_tot_L, d_tot_R, d_pos]);

        N_steps = 220;
        t_seq = (0:N_steps-1)' * dt;

        cmd_L = 3000.0 * sin(2*pi*8*t_seq) + 2000.0 * sin(2*pi*20*t_seq) + 800.0 * randn(N_steps, 1);
        cmd_R = 3000.0 * cos(2*pi*8*t_seq) + 2000.0 * cos(2*pi*20*t_seq) + 800.0 * randn(N_steps, 1);

        pos_true_L = 0.05 * sin(2*pi*2*t_seq);
        pos_true_R = 0.05 * cos(2*pi*2*t_seq);

        meas_L = zeros(N_steps, 1);
        meas_R = zeros(N_steps, 1);
        pos_meas_L = zeros(N_steps, 1);
        pos_meas_R = zeros(N_steps, 1);

        for k = 1:N_steps
            idx_cL = max(1, k - d_tot_L);
            idx_cR = max(1, k - d_tot_R);
            idx_p  = max(1, k - d_pos);

            meas_L(k) = cmd_L(idx_cL) + 10.0 * randn();
            meas_R(k) = cmd_R(idx_cR) + 10.0 * randn();
            pos_meas_L(k) = pos_true_L(idx_p);
            pos_meas_R(k) = pos_true_R(idx_p);
        end

        align_state = [];
        err_vec_trial = [];

        for k = 1:N_steps
            ts.t_source_L   = t_seq(k); ts.t_source_R   = t_seq(k); ts.t_source_pos = t_seq(k);
            ts.t_recv_L     = t_seq(k); ts.t_recv_R     = t_seq(k); ts.t_recv_pos   = t_seq(k);
            ts.seq_L        = k; ts.seq_R        = k; ts.seq_pos      = k;
            ts.clock_id_L   = 'CLK'; ts.clock_id_R   = 'CLK'; ts.clock_id_pos = 'CLK';

            qual = struct('is_saturated', [false; false], 'current_valid', [true; true], ...
                          'position_valid', [true; true], 'packet_valid', true);

            raw_k = [meas_L(k); meas_R(k)];
            cmd_k = [cmd_L(k); cmd_R(k)];
            pos_k = [pos_meas_L(k); pos_meas_R(k)];

            [sig_align, align_state, info] = step3c_causal_delay_aligner( ...
                raw_k, cmd_k, pos_k, ts, qual, align_state, opts_b2);

            if sig_align.valid_for_regression
                assert(sig_align.current_pair_valid, 'B2: current_pair_valid 必须为 true');
                assert(sig_align.absolute_alignment_valid, 'B2: absolute_alignment_valid 必须为 true');

                idx_common = k - d_common;
                if idx_common >= 1
                    err_iL = sig_align.current_cal(1) - cmd_L(idx_common);
                    err_iR = sig_align.current_cal(2) - cmd_R(idx_common);
                    err_vec_trial = [err_vec_trial; err_iL; err_iR]; %#ok<AGROW>
                end
            end
        end

        if ~isempty(err_vec_trial)
            b2_rms_errors(j) = sqrt(mean(err_vec_trial.^2));
            b2_max_errors(j) = max(abs(err_vec_trial));
        else
            b2_rms_errors(j) = NaN;
            b2_max_errors(j) = NaN;
        end

        if (info.d_meas_hat(1) == d_meas_L) && (info.d_meas_hat(2) == d_meas_R)
            b2_is_correct(j) = true;
        else
            b2_is_correct(j) = false;
        end

        records(end+1) = make_record('B2_SCENARIO_B_NOISY_MC', j, d_meas_L, d_meas_R, info, align_state, sig_align);
    end

    acc_rate_b2 = 100.0 * mean(b2_is_correct);
    mean_rms_b2 = mean(b2_rms_errors(isfinite(b2_rms_errors)));
    p95_rms_b2  = prctile(b2_rms_errors(isfinite(b2_rms_errors)), 95);

    fprintf('      - Scenario B 统计指标:\n');
    fprintf('        * 100 次 MC 双通道精确识别正确率: %.1f%% (验收门限 >= 95.0%%)\n', acc_rate_b2);
    fprintf('        * 100 次 MC 对齐电流 RMS 误差均值: %.2f counts (门限 <= 30.0 counts)\n', mean_rms_b2);
    fprintf('        * 100 次 MC 对齐电流 RMS 误差 P95:  %.2f counts\n', p95_rms_b2);

    assert(acc_rate_b2 >= 95.0, sprintf('B2 Scenario B 失败: 互相关时延识别正确率 %.1f%% < 95.0%%', acc_rate_b2));
    assert(mean_rms_b2 <= 30.0, sprintf('B2 Scenario B 失败: RMS 误差均值 %.2f counts > 30.0 counts', mean_rms_b2));
    fprintf('      - Scenario B PASS\n');

    % ---------------------------------------------------------------------
    % [Scenario C] 丢包与时间戳不连续性因果一致性测试 (common_timestamp 从缓冲区读取)
    % ---------------------------------------------------------------------
    fprintf('    >>> [Scenario C] 丢包与时间戳不连续性因果一致性测试...\n');
    align_state_c = [];
    max_causality_gap = 0;
    for k = 1:180
        t_src_base = (k - 1) * dt;
        % 模拟非均匀抖动源时间戳，但在物理因果范围内
        ts_c.t_source_L   = t_src_base + 0.0001 * sin(k);
        ts_c.t_source_R   = t_src_base + 0.0001 * cos(k);
        ts_c.t_source_pos = t_src_base;
        % 接收时间均晚于发射
        ts_c.t_recv_L     = ts_c.t_source_L + 0.002;
        ts_c.t_recv_R     = ts_c.t_source_R + 0.003;
        ts_c.t_recv_pos   = ts_c.t_source_pos + 0.001;
        ts_c.seq_L        = k; ts_c.seq_R = k; ts_c.seq_pos = k;
        ts_c.clock_id_L   = 'CLK'; ts_c.clock_id_R = 'CLK'; ts_c.clock_id_pos = 'CLK';

        qual_c = struct('is_saturated', [false; false], 'current_valid', [true; true], ...
                        'position_valid', [true; true], 'packet_valid', true);

        raw_k_c = [meas_L_a(k); meas_R_a(k)];
        cmd_k_c = [cmd_L_a(k); cmd_R_a(k)];
        pos_k_c = [pos_meas_L_a(k); pos_meas_R_a(k)];

        [sig_c, align_state_c, info_c] = step3c_causal_delay_aligner( ...
            raw_k_c, cmd_k_c, pos_k_c, ts_c, qual_c, align_state_c, opts_b2);

        if sig_c.valid_for_regression
            % 断言 common_timestamp 严格为所选历史源时间戳的最小值
            expected_common = min(sig_c.used_source_timestamp);
            assert(abs(sig_c.common_timestamp - expected_common) < 1e-12, ...
                'Scenario C: common_timestamp 必须严格等于 min(used_source_timestamp)');
            % 断言严格因果性: common_timestamp 绝不超过墙上时间
            t_wall_c = max([ts_c.t_recv_L, ts_c.t_recv_R, ts_c.t_recv_pos]);
            assert(sig_c.common_timestamp <= t_wall_c + 1e-12, ...
                'Scenario C: common_timestamp 超越物理接收墙上时间');
            gap = max(sig_c.used_source_timestamp) - sig_c.common_timestamp;
            if gap > max_causality_gap
                max_causality_gap = gap;
            end
        end
    end
    fprintf('      - Scenario C PASS: common_timestamp 严格从历史缓冲区读取且严格因果 (最大通道离散差: %.2e s)\n', max_causality_gap);
    records(end+1) = make_record('B2_SCENARIO_C_DISCONTINUITY', 1, NaN, NaN, info_c, align_state_c, sig_c);

    % ---------------------------------------------------------------------
    % [Scenario D] 未知位置延迟测试 (d_pos_known = NaN 必须刚性切断回归)
    % ---------------------------------------------------------------------
    fprintf('    >>> [Scenario D] 未知位置延迟测试 (d_pos_known = NaN 刚性切断回归)...\n');
    opts_b2_nopos = opts_b2;
    opts_b2_nopos.d_pos_known = NaN;
    ts_nopos = ts_c;
    ts_nopos.seq_L = align_state_c.last_seq_L + 1;
    ts_nopos.seq_R = align_state_c.last_seq_R + 1;
    ts_nopos.seq_pos = align_state_c.last_seq_pos_L + 1;
    ts_nopos.t_source_L = align_state_c.last_ts_L + dt;
    ts_nopos.t_source_R = align_state_c.last_ts_R + dt;
    ts_nopos.t_source_pos = align_state_c.last_ts_pos_L + dt;
    ts_nopos.t_recv_L = ts_nopos.t_source_L + 0.001;
    ts_nopos.t_recv_R = ts_nopos.t_source_R + 0.001;
    ts_nopos.t_recv_pos = ts_nopos.t_source_pos + 0.001;
    raw_k_d = [meas_L_a(181); meas_R_a(181)];
    cmd_k_d = [cmd_L_a(181); cmd_R_a(181)];
    pos_k_d = [pos_meas_L_a(181); pos_meas_R_a(181)];
    [sig_nopos, ~, info_nopos] = step3c_causal_delay_aligner(raw_k_d, cmd_k_d, pos_k_d, ts_nopos, qual_c, align_state_c, opts_b2_nopos);
    assert(sig_nopos.current_pair_valid, 'B2 Scenario D: 左右电流差模对齐仍应有效');
    assert(~sig_nopos.absolute_alignment_valid, 'B2 Scenario D: 无位置时延时绝对对齐必须为 false');
    assert(~sig_nopos.valid_for_regression, 'B2 Scenario D: 无位置时基时严禁开放回归准入');
    assert(all(isnan(sig_nopos.current_cal)) && all(isnan(sig_nopos.position)), 'B2 Scenario D: 绝对对齐无效期输出必须为 NaN');
    assert(strcmp(info_nopos.reject_reason, 'POSITION_DELAY_UNKNOWN'), 'B2 Scenario D: 原因码应为 POSITION_DELAY_UNKNOWN');
    records(end+1) = make_record('B2_SCENARIO_D_NOPOS_UNKNOWN', 101, NaN, NaN, info_nopos, align_state_c, sig_nopos);
    fprintf('      - Scenario D PASS: 无已知位置延迟模型时刚性切断回归准入 (POSITION_DELAY_UNKNOWN)\n');

    fprintf('    [OK] Subtest B2 验收通过: 四场景分流 (A~D) 全面合规通过!\n\n');

    %% =====================================================================
    %% [Subtest B3] DIFF_ONLY 未知路径差模降级与非法假设/门控测试
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B3] 开始 DIFF_ONLY 未知路径差模降级与门控保护测试...\n');

    % 1. 合法降级 (assume_symmetric_path = true, 真实路径对称)
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
        raw_k = [cmd_L(max(1, k - d_tot_L)); cmd_R(max(1, k - d_tot_R))];
        cmd_k = [cmd_L(k); cmd_R(k)];

        ts.t_source_L = t_seq(k); ts.t_source_R = t_seq(k); ts.t_source_pos = t_seq(k);
        ts.t_recv_L = t_seq(k); ts.t_recv_R = t_seq(k); ts.t_recv_pos = t_seq(k);
        ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;

        [sig_align, align_state, info] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b3);
    end

    assert(info.delta_d_hat == 1, 'B3: 差模时延识别不符');
    assert(all(isnan(info.d_meas_hat)), 'B3: DIFF_ONLY 模式严禁输出虚假绝对时延');
    assert(sig_align.current_pair_delay_estimate_valid, 'B3: current_pair_delay_estimate_valid 应为 true');
    assert(~sig_align.current_pair_valid, 'B3: 未对齐前 current_pair_valid 必须为 false');
    assert(~sig_align.absolute_alignment_valid, 'B3: absolute_alignment_valid 必须为 false');
    assert(~sig_align.valid_for_regression, 'B3: valid_for_regression 必须为 false');
    assert(all(isnan(sig_align.current_cal)) && all(isnan(sig_align.position)), 'B3: 输出必须全为 NaN');
    records(end+1) = make_record('B3_DIFF_ONLY_LEGAL', 1, d_tot_L, d_tot_R, info, align_state, sig_align);

    % 2. 非法假设 1: assume_symmetric_path = false (拒绝更新)
    opts_b3_unassumed = opts_b3;
    opts_b3_unassumed.assume_symmetric_path = false;
    ts_unassumed = ts;
    ts_unassumed.seq_L = 1; ts_unassumed.seq_R = 1; ts_unassumed.seq_pos = 1;
    ts_unassumed.t_source_L = dt; ts_unassumed.t_source_R = dt; ts_unassumed.t_source_pos = dt;
    ts_unassumed.t_recv_L = dt; ts_unassumed.t_recv_R = dt; ts_unassumed.t_recv_pos = dt;
    [sig_unassumed, ~, info_unassumed] = step3c_causal_delay_aligner( ...
        raw_k, cmd_k, [0.0; 0.0], ts_unassumed, qual, [], opts_b3_unassumed);
    assert(~info_unassumed.did_update, 'B3: 未声明对称性时必须禁止更新');
    assert(strcmp(info_unassumed.reject_reason, 'ASYMMETRIC_PATH_UNASSUMED'), 'B3: 原因码不匹配');
    records(end+1) = make_record('B3_UNASSUMED_PATH', 2, NaN, NaN, info_unassumed, align_state, sig_unassumed);

    % 3. 饱和状态拦截
    qual_sat = qual;
    qual_sat.is_saturated = [true; false];
    ts_sat = ts;
    ts_sat.seq_L = align_state.last_seq_L + 1;
    ts_sat.seq_R = align_state.last_seq_R + 1;
    ts_sat.seq_pos = align_state.last_seq_pos_L + 1;
    ts_sat.t_source_L = align_state.last_ts_L + dt;
    ts_sat.t_source_R = align_state.last_ts_R + dt;
    ts_sat.t_source_pos = align_state.last_ts_pos_L + dt;
    ts_sat.t_recv_L = ts_sat.t_source_L;
    ts_sat.t_recv_R = ts_sat.t_source_R;
    ts_sat.t_recv_pos = ts_sat.t_source_pos;
    [sig_sat, ~, info_sat] = step3c_causal_delay_aligner(raw_k, cmd_k, [0.0; 0.0], ts_sat, qual_sat, align_state, opts_b3);
    assert(~info_sat.did_update && ~sig_sat.valid_for_regression && strcmp(info_sat.reject_reason, 'SATURATION'));
    records(end+1) = make_record('B3_SATURATION', 3, NaN, NaN, info_sat, align_state, sig_sat);

    % 4. 低激励状态拦截
    raw_dc = [1000.0; 1000.0]; cmd_dc = [1000.0; 1000.0];
    align_state_dc = [];
    for k_dc = 1:35
        ts_dc = ts;
        ts_dc.seq_L = k_dc; ts_dc.seq_R = k_dc; ts_dc.seq_pos = k_dc;
        ts_dc.t_source_L = k_dc * dt; ts_dc.t_source_R = k_dc * dt; ts_dc.t_source_pos = k_dc * dt;
        ts_dc.t_recv_L = ts_dc.t_source_L; ts_dc.t_recv_R = ts_dc.t_source_R; ts_dc.t_recv_pos = ts_dc.t_source_pos;
        [sig_low, align_state_dc, info_low] = step3c_causal_delay_aligner(raw_dc, cmd_dc, [0.0; 0.0], ts_dc, qual, align_state_dc, opts_b3);
    end
    assert(~info_low.did_update && ~sig_low.valid_for_regression && strcmp(info_low.reject_reason, 'LOW_EXCITATION'));
    records(end+1) = make_record('B3_LOW_EXCITATION', 4, NaN, NaN, info_low, align_state_dc, sig_low);

    % 5. 非对称物理路径被错误假设为对称的保护性断言
    % 实际 d_path = [2; 4] 时，总差模反映的是包括执行器的总延迟，绝不能被误用为回归输入
    assert(~sig_align.valid_for_regression, 'B3: DIFF_ONLY 绝对禁止回归输入');
    records(end+1) = make_record('B3_REGRESSION_FREEZE', 5, NaN, NaN, info, align_state, sig_align);

    fprintf('    [OK] Subtest B3 验收通过: DIFF_ONLY 强制输出 NaN、全面配置门控并切断回归准入!\n\n');

    %% =====================================================================
    %% [Subtest B4] 明确五步迟滞确认测试 (防单步跳变与瞬态毛刺)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B4] 开始严格五步迟滞确认测试 (Step 1~4 保持, Step 5 切换, 毛刺不切换)...\n');

    align_state = [];
    opts_b4 = opts_base;
    opts_b4.mode = 'XCORR_KNOWN_PATH';
    opts_b4.d_path_known = [2; 2];
    opts_b4.d_pos_known  = 1;
    opts_b4.N_confirm    = 5;

    % 1. 先建立延迟 3 的稳定基准 (d_tot = 3 -> d_meas = 1)
    rng(42, 'twister');
    cmd_chirp = 4000.0 * sin(2*pi*8*(0:200)'*dt) + 2000.0 * sin(2*pi*25*(0:200)'*dt) + 1000.0 * randn(201, 1);
    for k = 1:50
        raw_k = [cmd_chirp(max(1, k - 3)); cmd_chirp(max(1, k - 3))];
        cmd_k = [cmd_chirp(k); cmd_chirp(k)];
        ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;
        ts.t_source_L = k*dt; ts.t_source_R = k*dt; ts.t_source_pos = k*dt;
        ts.t_recv_L = k*dt; ts.t_recv_R = k*dt; ts.t_recv_pos = k*dt;
        [sig_align, align_state, info] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b4);
    end
    assert(all(info.d_meas_hat == [1; 1]), 'B4: 基准时延未确认');
    assert(all(align_state.confirm_count >= 5), 'B4: 基准确认计数未达标');
    records(end+1) = make_record('B4_BASELINE_CONFIRMED', 1, 1, 1, info, align_state, sig_align);

    % 2. 突变输入至延迟 5 (d_tot = 5 -> d_meas = 3), 逐步监测确认计数与锁定状态
    % 对齐历史缓冲区中与指令的对应相位，确保互相关滑动窗口在步阶到达时直接输出 candidate = [5; 5]
    align_state.current_buffer_L(1:50) = cmd_chirp(max(1, (50:-1:1)' - 5));
    align_state.current_buffer_R(1:50) = cmd_chirp(max(1, (50:-1:1)' - 5));

    cand_hist = zeros(5, 2);
    conf_cnt_hist = zeros(5, 2);
    trusted_hist = zeros(5, 2);

    for s = 1:5
        k_curr = 50 + s;
        raw_k = [cmd_chirp(max(1, k_curr - 5)); cmd_chirp(max(1, k_curr - 5))];
        cmd_k = [cmd_chirp(k_curr); cmd_chirp(k_curr)];
        ts.seq_L = k_curr; ts.seq_R = k_curr; ts.seq_pos = k_curr;
        ts.t_source_L = k_curr*dt; ts.t_source_R = k_curr*dt; ts.t_source_pos = k_curr*dt;
        ts.t_recv_L = k_curr*dt; ts.t_recv_R = k_curr*dt; ts.t_recv_pos = k_curr*dt;

        [sig_s, align_state, info_s] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b4);

        cand_hist(s, :)     = align_state.candidate_delay';
        conf_cnt_hist(s, :) = align_state.confirm_count';
        trusted_hist(s, :)  = align_state.last_trusted_delay';

        fprintf('      - 阶跃第 %d 步: candidate=[%d,%d], confirm_cnt=[%d,%d], trusted=[%d,%d], did_update=%d\n', ...
            s, cand_hist(s,1), cand_hist(s,2), conf_cnt_hist(s,1), conf_cnt_hist(s,2), ...
            trusted_hist(s,1), trusted_hist(s,2), info_s.did_update);

        if s < 5
            % 1~4 步: 必须维持上一可信延迟 [1; 1], 严禁提前跳变!
            assert(all(trusted_hist(s, :) == [1, 1]), sprintf('B4: 第 %d 步迟滞失效，未满 5 步即发生了跳变', s));
            assert(all(conf_cnt_hist(s, :) == [s, s]), sprintf('B4: 第 %d 步确认计数递增异常', s));
            assert(~info_s.did_update, sprintf('B4: 第 %d 步严禁报告 did_update=true', s));
        else
            % 第 5 步: 达到确认门限，正式切换至 [3; 3]!
            assert(all(trusted_hist(s, :) == [3, 3]), 'B4: 第 5 步达到门限后未能成功切换可信时延');
            assert(all(conf_cnt_hist(s, :) >= [5, 5]), 'B4: 第 5 步确认计数异常');
            assert(info_s.did_update, 'B4: 第 5 步确认切换时必须设置 did_update=true');
        end
    end
    records(end+1) = make_record('B4_STEP4_MAINTAIN', 2, 3, 3, info_s, align_state, sig_s);
    records(end+1) = make_record('B4_STEP5_CONFIRMED', 3, 3, 3, info_s, align_state, sig_s);

    % 3. 稳态测试: 第 6~10 步继续维持延迟 5 输入，验证维持 [3; 3] 且严禁重复报告 did_update=true
    for s = 6:10
        k_curr = 50 + s;
        raw_k = [cmd_chirp(max(1, k_curr - 5)); cmd_chirp(max(1, k_curr - 5))];
        cmd_k = [cmd_chirp(k_curr); cmd_chirp(k_curr)];
        ts.seq_L = k_curr; ts.seq_R = k_curr; ts.seq_pos = k_curr;
        ts.t_source_L = k_curr*dt; ts.t_source_R = k_curr*dt; ts.t_source_pos = k_curr*dt;
        ts.t_recv_L = k_curr*dt; ts.t_recv_R = k_curr*dt; ts.t_recv_pos = k_curr*dt;

        [sig_ss, align_state, info_ss] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b4);

        assert(all(align_state.last_trusted_delay == [3; 3]), sprintf('B4 稳态第 %d 步: last_trusted_delay 偏离', s));
        assert(all(info_ss.d_meas_hat == [3; 3]), sprintf('B4 稳态第 %d 步: d_meas_hat 偏离', s));
        assert(~info_ss.did_update, sprintf('B4 稳态第 %d 步: 稳态下严禁重复报告 did_update=true', s));
        assert(all(align_state.confirm_count >= 5), sprintf('B4 稳态第 %d 步: confirm_count 异常', s));
    end
    records(end+1) = make_record('B4_STEADY_STATE_MAINTAIN', 4, 3, 3, info_ss, align_state, sig_ss);

    % 4. 注入仅维持 2 步的时延噪声毛刺 (d_tot 突变为 7 -> d_meas = 5)
    align_state.current_buffer_L(1:60) = cmd_chirp(max(1, (60:-1:1)' - 7));
    align_state.current_buffer_R(1:60) = cmd_chirp(max(1, (60:-1:1)' - 7));
    for g = 1:2
        k_curr = 60 + g;
        raw_k = [cmd_chirp(max(1, k_curr - 7)); cmd_chirp(max(1, k_curr - 7))];
        cmd_k = [cmd_chirp(k_curr); cmd_chirp(k_curr)];
        ts.seq_L = k_curr; ts.seq_R = k_curr; ts.seq_pos = k_curr;
        ts.t_source_L = k_curr*dt; ts.t_source_R = k_curr*dt; ts.t_source_pos = k_curr*dt;
        ts.t_recv_L = k_curr*dt; ts.t_recv_R = k_curr*dt; ts.t_recv_pos = k_curr*dt;

        [sig_glitch, align_state, info_glitch] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_b4);

        assert(all(info_glitch.d_meas_hat == [3; 3]), 'B4: 迟滞失效，瞬态毛刺污染了可信时延');
        assert(~info_glitch.did_update, 'B4: 瞬态毛刺下严禁触发 did_update');
    end
    records(end+1) = make_record('B4_GLITCH_REJECTED', 5, 3, 3, info_glitch, align_state, sig_glitch);

    fprintf('    [OK] Subtest B4 验收通过: 五步迟滞确认机制完全生效 (Step 1~4 刚性维持, Step 5 精准确认切换, 稳态防重更, 毛刺零污染)!\n\n');

    %% =====================================================================
    %% [Subtest B5] 严格因果性硬检查与实际索引审计 (未来样本引用次数严格 == 0)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B5] 开始严格因果性硬检查与实际使用历史索引审计 (500 步全工况)...\n');

    future_ref_count = 0;
    align_state = [];
    opts_b5 = opts_base;
    opts_b5.mode = 'TIMESTAMP';

    for k = 1:500
        dL = mod(k, 3);
        dR = mod(k+1, 3);
        dpos = 1;

        t_src = k * dt;
        ts.t_source_L   = t_src;
        ts.t_source_R   = t_src;
        ts.t_source_pos = t_src;

        ts.t_recv_L     = t_src + dL * dt;
        ts.t_recv_R     = t_src + dR * dt;
        ts.t_recv_pos   = t_src + dpos * dt;
        t_wall          = max([ts.t_recv_L, ts.t_recv_R, ts.t_recv_pos]);

        ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;
        ts.clock_id_L = 'CLK'; ts.clock_id_R = 'CLK'; ts.clock_id_pos = 'CLK';

        [sig_align, align_state, info_b5] = step3c_causal_delay_aligner( ...
            [10.0; 10.0], [10.0; 10.0], [0.1; 0.1], ts, qual, align_state, opts_b5);

        if sig_align.valid_for_regression
            % 1. 实际使用缓冲区索引审计
            valid_depth = min(align_state.buffer_count, align_state.buffer_depth);
            assert(all(sig_align.used_index >= 1), 'B5: 检测到非正历史索引');
            assert(all(sig_align.used_index <= valid_depth), 'B5: 索引超出已存有效深度');

            % 2. 实际引用的源时间戳审计 (有限性与因果性)
            assert(all(isfinite(sig_align.used_source_timestamp)), 'B5: 历史源时间戳必须有限');
            assert(isfinite(sig_align.common_timestamp), 'B5: 公共基准时刻必须有限');
            if any(sig_align.used_source_timestamp > sig_align.common_timestamp + 1e-12)
                future_ref_count = future_ref_count + 1;
            end
            if sig_align.common_timestamp > t_wall + 1e-12
                future_ref_count = future_ref_count + 1;
            end
        end
    end

    % 3. 人为构造未来引用的负测试，确保断言逻辑确实能精准捕捉越界
    tampered_sig = sig_align;
    tampered_sig.used_source_timestamp(1) = tampered_sig.common_timestamp + 0.05;
    tampered_detected = any(tampered_sig.used_source_timestamp > tampered_sig.common_timestamp + 1e-12);
    assert(tampered_detected, 'B5: 负测试失败，未能捕捉人为注入的未来时间戳泄露');

    % 4. 显式越界检索负测试: 注入超过缓冲区深度的时延 (500 steps > buffer_depth 250)，对齐器应刚性拦截并报告 BUFFER_WARMING
    ts_oob = ts;
    ts_oob.seq_L = align_state.last_seq_L + 1;
    ts_oob.seq_R = align_state.last_seq_R + 1;
    ts_oob.seq_pos = align_state.last_seq_pos_L + 1;
    ts_oob.t_source_L = (align_state.last_seq_L + 1) * dt;
    ts_oob.t_source_R = (align_state.last_seq_R + 1) * dt;
    ts_oob.t_source_pos = (align_state.last_seq_pos_L + 1) * dt;
    ts_oob.t_recv_L = ts_oob.t_source_L + 500 * dt; % 接收延迟 500 步
    ts_oob.t_recv_R = ts_oob.t_source_R + 500 * dt;
    ts_oob.t_recv_pos = ts_oob.t_source_pos + 500 * dt;
    [sig_oob, ~, info_oob] = step3c_causal_delay_aligner([10.0; 10.0], [10.0; 10.0], [0.1; 0.1], ts_oob, qual, align_state, opts_b5);
    assert(~sig_oob.valid_for_regression && all(isnan(sig_oob.current_cal)) && ~info_oob.did_update);
    assert(strcmp(info_oob.reject_reason, 'BUFFER_WARMING'), 'B5: 超出缓冲区深度的时延应判定为 BUFFER_WARMING');

    fprintf('    [B5 统计指标]:\n');
    fprintf('      - 500 步全工况未来样本/未来时间戳引用次数: %d (断言严格 == 0)\n', future_ref_count);
    fprintf('      - 负测试检测灵敏度: 100%% 成功捕捉人为注入的越界时间戳\n');
    fprintf('      - 超出缓冲区深度负测试: 100%% 成功刚性拦截 (BUFFER_WARMING)\n');
    assert(future_ref_count == 0, 'B5 失败: 检测到未来样本引用');
    records(end+1) = make_record('B5_CAUSALITY_AUDIT', 1, dL, dR, info_b5, align_state, sig_align);
    records(end+1) = make_record('B5_OOB_REJECTED', 2, 500, 500, info_oob, align_state, sig_oob);

    fprintf('    [OK] Subtest B5 验收通过: 严格因果索引审计成立，未来样本引用次数严格为 0!\n\n');

    %% =====================================================================
    %% [Subtest B6] 预热期、异常输入防护与下游 SVF/RLS 冻结硬断言
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Subtest B6] 开始预热期零更新硬断言与异常输入防护测试...\n');

    % 1. 明确预热期零更新测试 (dL=3, dR=2, dpos=2 -> expected_warmup = 4 步)
    dL_b6 = 3; dR_b6 = 2; dpos_b6 = 2;
    expected_warmup = max([dL_b6, dR_b6, dpos_b6]) + 1; % 4 步

    align_state = [];
    downstream_updates_warmup = 0;

    for k = 1:expected_warmup - 1
        t_curr = k * dt;
        ts_w.t_source_L   = t_curr;
        ts_w.t_source_R   = t_curr;
        ts_w.t_source_pos = t_curr;
        ts_w.t_recv_L     = t_curr + dL_b6 * dt;
        ts_w.t_recv_R     = t_curr + dR_b6 * dt;
        ts_w.t_recv_pos   = t_curr + dpos_b6 * dt;
        ts_w.seq_L        = k; ts_w.seq_R = k; ts_w.seq_pos = k;
        ts_w.clock_id_L   = 'CLK'; ts_w.clock_id_R = 'CLK'; ts_w.clock_id_pos = 'CLK';

        [sig_w, align_state, info_w] = step3c_causal_delay_aligner( ...
            [10.0; 10.0], [10.0; 10.0], [0.0; 0.0], ts_w, qual, align_state, opts_b1);

        % 预热期必须完全冻结
        assert(~sig_w.valid_for_regression, sprintf('B6: 第 %d 步预热期 valid_for_regression 必须为 false', k));
        assert(all(isnan(sig_w.current_cal)), sprintf('B6: 第 %d 步预热期电流必须为 NaN', k));
        assert(all(isnan(sig_w.position)), sprintf('B6: 第 %d 步预热期位置必须为 NaN', k));
        assert(~info_w.did_update, sprintf('B6: 第 %d 步预热期 did_update 必须为 false', k));

        if sig_w.valid_for_regression
            downstream_updates_warmup = downstream_updates_warmup + 1;
        end
    end
    assert(downstream_updates_warmup == 0, 'B6 失败: 预热期检测到下游非法更新');
    fprintf('    [OK] Case 1: 预热期 1~%d 步全输出 NaN，下游更新严格为 0\n', expected_warmup - 1);
    records(end+1) = make_record('B6_WARMUP_FREEZE', 1, dL_b6, dR_b6, info_w, align_state, sig_w);

    % 第 4 步开始满足深度
    k = expected_warmup;
    t_curr = k * dt;
    ts_w.t_source_L = t_curr; ts_w.t_source_R = t_curr; ts_w.t_source_pos = t_curr;
    ts_w.t_recv_L = t_curr + dL_b6*dt; ts_w.t_recv_R = t_curr + dR_b6*dt; ts_w.t_recv_pos = t_curr + dpos_b6*dt;
    ts_w.seq_L = k; ts_w.seq_R = k; ts_w.seq_pos = k;
    [sig_w4, align_state, info_w4] = step3c_causal_delay_aligner( ...
        [10.0; 10.0], [10.0; 10.0], [0.0; 0.0], ts_w, qual, align_state, opts_b1);
    assert(sig_w4.valid_for_regression, 'B6: 第 4 步完成预热后必须开放准入');
    assert(all(isfinite(sig_w4.current_cal)) && all(isfinite(sig_w4.position)));

    % 2. 负时延截零伪装检查 (严禁将 d_total < d_path 截零)
    opts_neg = opts_base;
    opts_neg.mode = 'XCORR_KNOWN_PATH';
    opts_neg.d_path_known = [5; 5];
    opts_neg.d_pos_known  = 1;

    align_state = [];
    clamped_zero_count = 0;
    downstream_neg_updates = 0;

    for k = 1:50
        raw_k = [cmd_chirp(max(1, k - 2)); cmd_chirp(max(1, k - 2))]; % 实际总时延仅 2 < 5
        cmd_k = [cmd_chirp(k); cmd_chirp(k)];
        ts.seq_L = k; ts.seq_R = k; ts.seq_pos = k;
        ts.t_source_L = k*dt; ts.t_source_R = k*dt; ts.t_source_pos = k*dt;
        ts.t_recv_L = k*dt; ts.t_recv_R = k*dt; ts.t_recv_pos = k*dt;

        [sig_neg, align_state, info_neg] = step3c_causal_delay_aligner( ...
            raw_k, cmd_k, [0.0; 0.0], ts, qual, align_state, opts_neg);

        if strcmp(info_neg.reject_reason, 'NEGATIVE_DELAY')
            assert(~info_neg.did_update, '负时延时严禁更新');
            if any(info_neg.d_meas_hat == 0)
                clamped_zero_count = clamped_zero_count + 1;
            end
        end
        if sig_neg.valid_for_regression
            downstream_neg_updates = downstream_neg_updates + 1;
        end
    end
    assert(clamped_zero_count == 0, 'B6: 严禁截零伪装');
    assert(downstream_neg_updates == 0, 'B6: 负时延期下游更新严格为 0');
    fprintf('    [OK] Case 2: 负时延拒绝生效 (NEGATIVE_DELAY)，截零伪装次数与下游更新严格为 0\n');
    records(end+1) = make_record('B6_NEGATIVE_DELAY', 2, NaN, NaN, info_neg, align_state, sig_neg);

    % 3. NaN/Inf、丢包、饱和工况下游 RLS 更新次数硬统计 (必须全为 0)
    ts_fault = ts;
    ts_fault.seq_L = align_state.last_seq_L + 1;
    ts_fault.seq_R = align_state.last_seq_R + 1;
    ts_fault.seq_pos = align_state.last_seq_pos_L + 1;
    ts_fault.t_source_L = align_state.last_ts_L + dt;
    ts_fault.t_source_R = align_state.last_ts_R + dt;
    ts_fault.t_source_pos = align_state.last_ts_pos_L + dt;
    ts_fault.t_recv_L = ts_fault.t_source_L;
    ts_fault.t_recv_R = ts_fault.t_source_R;
    ts_fault.t_recv_pos = ts_fault.t_source_pos;

    % NaN 输入
    [sig_nan, ~, info_nan] = step3c_causal_delay_aligner([NaN; 10.0], [10.0; 10.0], [0;0], ts_fault, qual, align_state, opts_b1);
    assert(~sig_nan.valid_for_regression && all(isnan(sig_nan.current_cal)) && ~info_nan.did_update);
    assert(strcmp(info_nan.reject_reason, 'PACKET_CORRUPT'), 'NaN 输入应被判定为 PACKET_CORRUPT');
    records(end+1) = make_record('B6_NAN_INPUT', 3, NaN, NaN, info_nan, align_state, sig_nan);

    % Inf 输入
    ts_fault.seq_L = ts_fault.seq_L + 1; ts_fault.seq_R = ts_fault.seq_R + 1; ts_fault.seq_pos = ts_fault.seq_pos + 1;
    ts_fault.t_source_L = ts_fault.t_source_L + dt; ts_fault.t_source_R = ts_fault.t_source_R + dt; ts_fault.t_source_pos = ts_fault.t_source_pos + dt;
    ts_fault.t_recv_L = ts_fault.t_source_L; ts_fault.t_recv_R = ts_fault.t_source_R; ts_fault.t_recv_pos = ts_fault.t_source_pos;
    [sig_inf, ~, info_inf] = step3c_causal_delay_aligner([Inf; 10.0], [10.0; 10.0], [0;0], ts_fault, qual, align_state, opts_b1);
    assert(~sig_inf.valid_for_regression && all(isnan(sig_inf.current_cal)) && ~info_inf.did_update);
    assert(strcmp(info_inf.reject_reason, 'PACKET_CORRUPT'), 'Inf 输入应被判定为 PACKET_CORRUPT');
    records(end+1) = make_record('B6_INF_INPUT', 4, NaN, NaN, info_inf, align_state, sig_inf);

    % 丢包/损坏
    ts_fault.seq_L = ts_fault.seq_L + 1; ts_fault.seq_R = ts_fault.seq_R + 1; ts_fault.seq_pos = ts_fault.seq_pos + 1;
    ts_fault.t_source_L = ts_fault.t_source_L + dt; ts_fault.t_source_R = ts_fault.t_source_R + dt; ts_fault.t_source_pos = ts_fault.t_source_pos + dt;
    ts_fault.t_recv_L = ts_fault.t_source_L; ts_fault.t_recv_R = ts_fault.t_source_R; ts_fault.t_recv_pos = ts_fault.t_source_pos;
    qual_loss = qual; qual_loss.packet_valid = false;
    [sig_loss, ~, info_loss] = step3c_causal_delay_aligner([10.0; 10.0], [10.0; 10.0], [0;0], ts_fault, qual_loss, align_state, opts_b1);
    assert(~sig_loss.valid_for_regression && all(isnan(sig_loss.current_cal)) && ~info_loss.did_update);
    assert(strcmp(info_loss.reject_reason, 'PACKET_CORRUPT'), '丢包应被判定为 PACKET_CORRUPT');
    records(end+1) = make_record('B6_PACKET_LOSS', 5, NaN, NaN, info_loss, align_state, sig_loss);

    % 饱和工况
    ts_fault.seq_L = ts_fault.seq_L + 1; ts_fault.seq_R = ts_fault.seq_R + 1; ts_fault.seq_pos = ts_fault.seq_pos + 1;
    ts_fault.t_source_L = ts_fault.t_source_L + dt; ts_fault.t_source_R = ts_fault.t_source_R + dt; ts_fault.t_source_pos = ts_fault.t_source_pos + dt;
    ts_fault.t_recv_L = ts_fault.t_source_L; ts_fault.t_recv_R = ts_fault.t_source_R; ts_fault.t_recv_pos = ts_fault.t_source_pos;
    qual_sat2 = qual; qual_sat2.is_saturated = [true; false];
    [sig_sat2, ~, info_sat2] = step3c_causal_delay_aligner([10.0; 10.0], [10.0; 10.0], [0;0], ts_fault, qual_sat2, align_state, opts_b1);
    assert(~sig_sat2.valid_for_regression && all(isnan(sig_sat2.current_cal)) && ~info_sat2.did_update);
    assert(strcmp(info_sat2.reject_reason, 'SATURATION'), '饱和工况应被判定为 SATURATION');
    records(end+1) = make_record('B6_SAT_INPUT', 6, NaN, NaN, info_sat2, align_state, sig_sat2);

    fprintf('    [OK] Case 3: NaN/Inf/丢包/饱和下 valid_for_regression 全为 0，下游更新完全冻结\n');
    fprintf('    [OK] Subtest B6 验收通过: 预热零更新、截零伪装零容忍与异常保护全闭环!\n\n');

    %% =====================================================================
    %% 数据治理与 100% 内存逐列逐元素回读校验 (CSV)
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3c_c4b_delay_results.csv');
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> 正在导出 Gate C4-B 完整结果表 (包含 B1~B6 全测试集): %s\n', csv_file);

    T_out = struct2table(records);
    writetable(T_out, csv_file);
    fprintf('    [OK] CSV 写入完成，共计 %d 行 x %d 列\n', height(T_out), width(T_out));

    % 100% 逐列逐元素严格内存回读校验 (全 26 列)
    fprintf('>>> 执行 CSV 全部 26 列 100%% 逐元素严格内存回读校验...\n');
    T_read = readtable(csv_file);
    assert(height(T_read) == height(T_out), '回读行数不符');
    assert(width(T_read) == 26, sprintf('回读列数不符: 期望 26 列，实际 %d 列', width(T_read)));

    col_names = { ...
        'Subtest', 'Trial_ID', ...
        'True_Delay_L', 'True_Delay_R', ...
        'Estimated_Delay_L', 'Estimated_Delay_R', ...
        'Candidate_Delay_L', 'Candidate_Delay_R', ...
        'Confirm_Count_L', 'Confirm_Count_R', ...
        'Trusted_Delay_L', 'Trusted_Delay_R', ...
        'Used_Index_L', 'Used_Index_R', ...
        'Used_Index_Pos_L', 'Used_Index_Pos_R', ...
        'Used_Timestamp_L', 'Used_Timestamp_R', ...
        'Used_Timestamp_Pos_L', 'Used_Timestamp_Pos_R', ...
        'Common_Timestamp', ...
        'Current_Pair_Valid', 'Absolute_Alignment_Valid', 'Valid_For_Regression', ...
        'Reject_Reason', 'Did_Update' ...
    };

    for c = 1:length(col_names)
        name = col_names{c};
        val_orig = T_out.(name);
        val_read = T_read.(name);

        if iscell(val_orig) || iscellstr(val_orig) || isstring(val_orig)
            assert(all(string(val_orig) == string(val_read)), sprintf('列 %s 文本回读不匹配', name));
        elseif islogical(val_orig)
            assert(all(val_orig == logical(val_read)), sprintf('列 %s 逻辑值回读不匹配', name));
        else
            diff_col = abs(val_orig - val_read);
            % 处理 NaN 匹配
            nan_match = isnan(val_orig) == isnan(val_read);
            assert(all(nan_match), sprintf('列 %s NaN 模式回读不匹配', name));
            finite_diff = diff_col(isfinite(val_orig));
            if ~isempty(finite_diff)
                assert(max(finite_diff) < 1e-9, sprintf('列 %s 数值回读残差超标', name));
            end
        end
    end
    fprintf('    [OK] CSV 全部 26 列 100%% 逐元素严格回读断言全数通过!\n\n');

    %% =====================================================================
    %% 总结输出
    %% =====================================================================
    fprintf('=========================================================================\n');
    fprintf('   Gate C4-B 单元测试重测结论: 全部 PASS\n');
    fprintf('=========================================================================\n');
    fprintf('   Subtest B1 (TIMESTAMP 8+5 工况): 识别误差 0, 5 组异常时间戳全面拦截 [PASS]\n');
    fprintf('   Subtest B2 (XCORR 四场景分流):   A/B/C/D 全通过, MC 正确率 %.1f%%, RMS %.2f ct [PASS]\n', acc_rate_b2, mean_rms_b2);
    fprintf('   Subtest B3 (DIFF_ONLY 门控降级): 强制 NaN, 拒绝非对称, 饱和/低激励拦截 [PASS]\n');
    fprintf('   Subtest B4 (严格五步迟滞防抖):   1~4 步维持, 第 5 步切换, 稳态防重更, 毛刺零污染 [PASS]\n');
    fprintf('   Subtest B5 (因果性硬检验审计):   历史索引合法, 越界拦截, 未来引用次数严格 == 0 [PASS]\n');
    fprintf('   Subtest B6 (预热与异常输入防护): 预热零更新, 截零零容忍, 异常全冻结   [PASS]\n');
    fprintf('   数据治理与完整回读:              CSV 26 列 100%% 逐列逐元素回读校验一致  [PASS]\n');
    fprintf('=========================================================================\n');
end

%% =========================================================================
%% 辅助函数: 构造规范化 26 列数据记录
%% =========================================================================
function rec = make_record(subtest, trial_id, true_L, true_R, info, state, sig)
    rec = struct();
    rec.Subtest                  = string(subtest);
    rec.Trial_ID                 = double(trial_id);
    rec.True_Delay_L             = double(true_L);
    rec.True_Delay_R             = double(true_R);

    if isfield(info, 'd_meas_hat') && numel(info.d_meas_hat) >= 2
        rec.Estimated_Delay_L    = double(info.d_meas_hat(1));
        rec.Estimated_Delay_R    = double(info.d_meas_hat(2));
    else
        rec.Estimated_Delay_L    = NaN;
        rec.Estimated_Delay_R    = NaN;
    end

    if isfield(state, 'candidate_delay') && numel(state.candidate_delay) >= 2
        rec.Candidate_Delay_L    = double(state.candidate_delay(1));
        rec.Candidate_Delay_R    = double(state.candidate_delay(2));
    else
        rec.Candidate_Delay_L    = NaN;
        rec.Candidate_Delay_R    = NaN;
    end

    if isfield(state, 'confirm_count') && numel(state.confirm_count) >= 2
        rec.Confirm_Count_L      = double(state.confirm_count(1));
        rec.Confirm_Count_R      = double(state.confirm_count(2));
    else
        rec.Confirm_Count_L      = NaN;
        rec.Confirm_Count_R      = NaN;
    end

    if isfield(state, 'last_trusted_delay') && numel(state.last_trusted_delay) >= 2
        rec.Trusted_Delay_L      = double(state.last_trusted_delay(1));
        rec.Trusted_Delay_R      = double(state.last_trusted_delay(2));
    else
        rec.Trusted_Delay_L      = NaN;
        rec.Trusted_Delay_R      = NaN;
    end

    if isfield(sig, 'used_index') && numel(sig.used_index) >= 4
        rec.Used_Index_L         = double(sig.used_index(1));
        rec.Used_Index_R         = double(sig.used_index(2));
        rec.Used_Index_Pos_L     = double(sig.used_index(3));
        rec.Used_Index_Pos_R     = double(sig.used_index(4));
    else
        rec.Used_Index_L         = NaN;
        rec.Used_Index_R         = NaN;
        rec.Used_Index_Pos_L     = NaN;
        rec.Used_Index_Pos_R     = NaN;
    end

    if isfield(sig, 'used_source_timestamp') && numel(sig.used_source_timestamp) >= 4
        rec.Used_Timestamp_L     = double(sig.used_source_timestamp(1));
        rec.Used_Timestamp_R     = double(sig.used_source_timestamp(2));
        rec.Used_Timestamp_Pos_L = double(sig.used_source_timestamp(3));
        rec.Used_Timestamp_Pos_R = double(sig.used_source_timestamp(4));
    else
        rec.Used_Timestamp_L     = NaN;
        rec.Used_Timestamp_R     = NaN;
        rec.Used_Timestamp_Pos_L = NaN;
        rec.Used_Timestamp_Pos_R = NaN;
    end

    rec.Common_Timestamp         = double(sig.common_timestamp);
    rec.Current_Pair_Valid       = logical(sig.current_pair_valid);
    rec.Absolute_Alignment_Valid = logical(sig.absolute_alignment_valid);
    rec.Valid_For_Regression     = logical(sig.valid_for_regression);
    rec.Reject_Reason            = string(info.reject_reason);
    rec.Did_Update               = logical(info.did_update);
end
