function [signals_aligned, align_state_next, delay_info] = ...
    step3c_causal_delay_aligner( ...
        current_cal, current_cmd, position, ...
        timestamp, quality, align_state, opts)
% =========================================================================
% STEP 3C-4 Gate C4-B: 严格因果历史对齐与量测时延估计器
%
% 物理建模与可辨识性设计规范:
% 1. 物理延迟数学分解:
%    d_path  = d_act + d_driver         (执行器与驱动综合前向延迟)
%    d_total = d_path + d_meas          (控制回路闭环总延迟)
%    互相关估计提取的是 d_total, 绝非纯量测时延 d_meas!
% 2. 三大对齐模式:
%    - 'TIMESTAMP': 统一硬件源采样时间戳硬对齐 (工程主路径)
%    - 'XCORR_KNOWN_PATH': 已知 d_path_known 互相关估计 (需已知 d_pos_known 方可绝对对齐)
%    - 'DIFF_ONLY': 未知路径差模降级估计 (强制输出 NaN 与禁止回归)
% =========================================================================

    %% 1. 参数缺省处理与输入契约校验
    if nargin < 7, opts = struct(); end
    if ~isfield(opts, 'dt'), opts.dt = 0.001; end
    if ~isfield(opts, 'xcorr_window_length'), opts.xcorr_window_length = 150; end
    if ~isfield(opts, 'max_search_delay'), opts.max_search_delay = 10; end
    if ~isfield(opts, 'max_position_delay'), opts.max_position_delay = opts.max_search_delay; end
    if ~isfield(opts, 'd_pos_known'), opts.d_pos_known = NaN; end
    if ~isfield(opts, 'require_position_alignment'), opts.require_position_alignment = true; end
    if ~isfield(opts, 'strict_sequence'), opts.strict_sequence = true; end
    if ~isfield(opts, 'timestamp_alignment_tolerance')
        opts.timestamp_alignment_tolerance = 0.5 * opts.dt;
    end

    required_depth = opts.xcorr_window_length + opts.max_search_delay + opts.max_position_delay + 1;
    if ~isfield(opts, 'buffer_depth')
        opts.buffer_depth = max(250, required_depth + 50);
    end
    assert(opts.buffer_depth >= required_depth, 'buffer_depth 不足以覆盖相关窗口和最大因果回溯');

    if ~isfield(opts, 'mode'), opts.mode = 'TIMESTAMP'; end
    if ~isfield(opts, 'd_path_known'), opts.d_path_known = [0.0; 0.0]; end
    if ~isfield(opts, 'assume_symmetric_path'), opts.assume_symmetric_path = false; end
    if ~isfield(opts, 'th_cmd_var'), opts.th_cmd_var = 50.0; end
    if ~isfield(opts, 'th_peak_margin'), opts.th_peak_margin = 0.05; end
    if ~isfield(opts, 'th_peak_min'), opts.th_peak_min = 0.55; end
    if ~isfield(opts, 'N_confirm'), opts.N_confirm = 5; end
    if ~isfield(opts, 'Imax'), opts.Imax = 16000.0; end
    if ~isfield(opts, 'reset'), opts.reset = false; end

    assert(isnumeric(current_cal) && isreal(current_cal) && numel(current_cal) == 2, ...
        'current_cal 必须是包含两个实数元素的向量');
    assert(isnumeric(current_cmd) && isreal(current_cmd) && numel(current_cmd) == 2, ...
        'current_cmd 必须是包含两个实数元素的向量');
    assert(isnumeric(position) && isreal(position) && numel(position) == 2, ...
        'position 必须是包含两个实数元素的向量');
    assert(isstruct(timestamp), 'timestamp 必须是包含时间戳字段的结构体');
    assert(isstruct(quality), 'quality 必须是包含质量标志的结构体');

    current_cal = current_cal(:);
    current_cmd = current_cmd(:);
    position    = position(:);
    opts.d_path_known = opts.d_path_known(:);

    assert(numel(opts.d_path_known) == 2, 'd_path_known 元素个数必须为 2');
    assert(numel(opts.d_pos_known) == 1 || numel(opts.d_pos_known) == 2, 'd_pos_known 元素个数必须为 1 或 2');

    assert(all(isfinite(opts.d_path_known)), 'd_path_known 必须为有限数');
    assert(all(opts.d_path_known >= 0), 'd_path_known 不能为负');
    assert(all(mod(opts.d_path_known, 1) == 0), 'd_path_known 必须为整数采样延迟');

    if all(isfinite(opts.d_pos_known))
        assert(all(opts.d_pos_known >= 0), 'd_pos_known 不能为负');
        assert(all(mod(opts.d_pos_known, 1) == 0), 'd_pos_known 必须为整数采样延迟');
    end

    %% 2. 状态机跨步初始化与显式重置
    if nargin < 6 || isempty(align_state) || opts.reset
        align_state = struct();
        align_state.mode                     = opts.mode;
        align_state.buffer_depth             = opts.buffer_depth;
        align_state.current_buffer_L         = NaN(opts.buffer_depth, 1);
        align_state.current_buffer_R         = NaN(opts.buffer_depth, 1);
        align_state.command_buffer_L         = NaN(opts.buffer_depth, 1);
        align_state.command_buffer_R         = NaN(opts.buffer_depth, 1);
        align_state.position_buffer_L        = NaN(opts.buffer_depth, 1);
        align_state.position_buffer_R        = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_L        = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_R        = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_pos_L    = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_pos_R    = NaN(opts.buffer_depth, 1);
        align_state.buffer_count             = 0;
        align_state.d_hat_L                  = 0;
        align_state.d_hat_R                  = 0;
        align_state.d_total_hat              = [0.0; 0.0];
        align_state.last_trusted_delay       = [NaN; NaN];
        align_state.last_trusted_total       = [NaN; NaN];
        align_state.candidate_delay          = [NaN; NaN];
        align_state.confirm_count            = [0; 0];
        align_state.last_seq_L               = -1;
        align_state.last_seq_R               = -1;
        align_state.last_seq_pos_L           = -1;
        align_state.last_seq_pos_R           = -1;
        align_state.last_ts_L                = -Inf;
        align_state.last_ts_R                = -Inf;
        align_state.last_ts_pos_L            = -Inf;
        align_state.last_ts_pos_R            = -Inf;
        align_state.last_confidence          = 0.0;
        align_state.is_initialized           = false;
    end

    align_state_next = align_state;
    align_state_next.mode = opts.mode;

    %% 3. 初始化诊断与输出结构体
    delay_info = struct();
    delay_info.did_update    = false;
    delay_info.method        = opts.mode;
    if isfield(align_state_next, 'last_confidence')
        delay_info.confidence = align_state_next.last_confidence;
    else
        delay_info.confidence = 0.0;
    end
    delay_info.reject_reason = 'NONE';
    delay_info.d_total_hat   = align_state_next.d_total_hat;
    delay_info.d_meas_hat    = [NaN; NaN];
    delay_info.delta_d_hat   = align_state_next.d_hat_L - align_state_next.d_hat_R;

    signals_aligned = struct();
    signals_aligned.current_pair_valid                = false;
    signals_aligned.current_pair_delay_estimate_valid = false;
    signals_aligned.absolute_alignment_valid          = false;
    signals_aligned.valid_for_regression              = false;
    signals_aligned.common_timestamp                  = NaN;
    signals_aligned.current_cal                       = [NaN; NaN];
    signals_aligned.position                          = [NaN; NaN];
    signals_aligned.used_index                        = [NaN; NaN; NaN; NaN];
    signals_aligned.used_source_timestamp             = [NaN; NaN; NaN; NaN];

    %% 4. 统一时间戳与输入完整性前置校验 (在写入缓冲区之前完成)
    required_ts_base = {'t_source_L','t_source_R','t_recv_L','t_recv_R','seq_L','seq_R','clock_id_L','clock_id_R'};
    assert(all(isfield(timestamp, required_ts_base)), 'timestamp 缺少必要字段');

    pos_has_single = isfield(timestamp, 't_source_pos') && isfield(timestamp, 't_recv_pos') && ...
                     isfield(timestamp, 'seq_pos') && isfield(timestamp, 'clock_id_pos');
    pos_has_dual   = isfield(timestamp, 't_source_pos_L') && isfield(timestamp, 't_source_pos_R') && ...
                     isfield(timestamp, 't_recv_pos_L') && isfield(timestamp, 't_recv_pos_R') && ...
                     isfield(timestamp, 'seq_pos_L') && isfield(timestamp, 'seq_pos_R') && ...
                     isfield(timestamp, 'clock_id_pos_L') && isfield(timestamp, 'clock_id_pos_R');
    assert(pos_has_single || pos_has_dual, 'timestamp 缺少必要位置通道时间戳字段');

    if pos_has_single
        t_src_pos_L = timestamp.t_source_pos;
        t_src_pos_R = timestamp.t_source_pos;
        t_rcv_pos_L = timestamp.t_recv_pos;
        t_rcv_pos_R = timestamp.t_recv_pos;
        seq_p_L     = timestamp.seq_pos;
        seq_p_R     = timestamp.seq_pos;
        clk_p_L     = timestamp.clock_id_pos;
        clk_p_R     = timestamp.clock_id_pos;
    else
        t_src_pos_L = timestamp.t_source_pos_L;
        t_src_pos_R = timestamp.t_source_pos_R;
        t_rcv_pos_L = timestamp.t_recv_pos_L;
        t_rcv_pos_R = timestamp.t_recv_pos_R;
        seq_p_L     = timestamp.seq_pos_L;
        seq_p_R     = timestamp.seq_pos_R;
        clk_p_L     = timestamp.clock_id_pos_L;
        clk_p_R     = timestamp.clock_id_pos_R;
    end

    % 检查左右位置源时间戳不同 (如果显式提供了左右位置源且不一致)
    if isfield(timestamp, 't_source_pos_L') && isfield(timestamp, 't_source_pos_R')
        if abs(timestamp.t_source_pos_L - timestamp.t_source_pos_R) > 1e-12
            delay_info.reject_reason = 'POSITION_ASYNC';
            delay_info.did_update    = false;
            return;
        end
    end

    % 数值有效性检查 (必须为有限实数)
    ts_num = [timestamp.t_source_L, timestamp.t_source_R, t_src_pos_L, t_src_pos_R, ...
              timestamp.t_recv_L, timestamp.t_recv_R, t_rcv_pos_L, t_rcv_pos_R];
    if any(~isfinite(ts_num))
        delay_info.reject_reason = 'PACKET_CORRUPT';
        delay_info.did_update    = false;
        return;
    end

    % 物理因果方向检查: 接收时间严禁超前于源发射时间 (通信传输延迟 >= 0)
    if (timestamp.t_recv_L < timestamp.t_source_L) || ...
       (timestamp.t_recv_R < timestamp.t_source_R) || ...
       (t_rcv_pos_L < t_src_pos_L) || ...
       (t_rcv_pos_R < t_src_pos_R)
        delay_info.reject_reason = 'NEGATIVE_DELAY';
        delay_info.did_update    = false;
        return;
    end

    % 时钟域一致性检查 (严禁跨未对齐时钟域)
    clock_ok = isequal(timestamp.clock_id_L, timestamp.clock_id_R) && ...
               isequal(timestamp.clock_id_L, clk_p_L) && ...
               isequal(timestamp.clock_id_L, clk_p_R);
    if ~clock_ok
        delay_info.reject_reason = 'CLOCK_MISMATCH';
        delay_info.did_update    = false;
        return;
    end

    % 序列号单调性与连续性检查
    % 1. 防包乱序、回滚与重复 (seq <= last_seq)
    if opts.strict_sequence
        if (align_state_next.last_seq_L >= 0 && timestamp.seq_L <= align_state_next.last_seq_L) || ...
           (align_state_next.last_seq_R >= 0 && timestamp.seq_R <= align_state_next.last_seq_R) || ...
           (align_state_next.last_seq_pos_L >= 0 && seq_p_L <= align_state_next.last_seq_pos_L) || ...
           (align_state_next.last_seq_pos_R >= 0 && seq_p_R <= align_state_next.last_seq_pos_R)
            delay_info.reject_reason = 'SEQ_ROLLBACK';
            delay_info.did_update    = false;
            return;
        end
    else
        if (align_state_next.last_seq_L >= 0 && timestamp.seq_L < align_state_next.last_seq_L) || ...
           (align_state_next.last_seq_R >= 0 && timestamp.seq_R < align_state_next.last_seq_R) || ...
           (align_state_next.last_seq_pos_L >= 0 && seq_p_L < align_state_next.last_seq_pos_L) || ...
           (align_state_next.last_seq_pos_R >= 0 && seq_p_R < align_state_next.last_seq_pos_R)
            delay_info.reject_reason = 'SEQ_ROLLBACK';
            delay_info.did_update    = false;
            return;
        end
    end

    % 2. 丢包与序列号跳变连续性检查 (seq ~= last_seq + 1)
    if (align_state_next.last_seq_L >= 0 && timestamp.seq_L ~= align_state_next.last_seq_L + 1) || ...
       (align_state_next.last_seq_R >= 0 && timestamp.seq_R ~= align_state_next.last_seq_R + 1) || ...
       (align_state_next.last_seq_pos_L >= 0 && seq_p_L ~= align_state_next.last_seq_pos_L + 1) || ...
       (align_state_next.last_seq_pos_R >= 0 && seq_p_R ~= align_state_next.last_seq_pos_R + 1)
        delay_info.reject_reason = 'SEQ_GAP';
        delay_info.did_update    = false;
        % 丢包意味着历史连续性被打断，必须清空缓冲区与重置锁定状态，防止使用丢包前的历史做错误回归
        align_state_next.buffer_count          = 0;
        align_state_next.current_buffer_L(:)   = NaN;
        align_state_next.current_buffer_R(:)   = NaN;
        align_state_next.command_buffer_L(:)   = NaN;
        align_state_next.command_buffer_R(:)   = NaN;
        align_state_next.position_buffer_L(:)  = NaN;
        align_state_next.position_buffer_R(:)  = NaN;
        align_state_next.t_source_buffer_L(:)     = NaN;
        align_state_next.t_source_buffer_R(:)     = NaN;
        align_state_next.t_source_buffer_pos_L(:) = NaN;
        align_state_next.t_source_buffer_pos_R(:) = NaN;
        align_state_next.is_initialized        = false;
        align_state_next.last_trusted_delay    = [NaN; NaN];
        align_state_next.last_trusted_total    = [NaN; NaN];
        align_state_next.candidate_delay       = [NaN; NaN];
        align_state_next.confirm_count         = [0; 0];
        % 更新最后序列号与时间戳至当前帧，以便后续报文可以连续递推重入
        align_state_next.last_seq_L            = timestamp.seq_L;
        align_state_next.last_seq_R            = timestamp.seq_R;
        align_state_next.last_seq_pos_L        = seq_p_L;
        align_state_next.last_seq_pos_R        = seq_p_R;
        align_state_next.last_ts_L             = timestamp.t_source_L;
        align_state_next.last_ts_R             = timestamp.t_source_R;
        align_state_next.last_ts_pos_L         = t_src_pos_L;
        align_state_next.last_ts_pos_R         = t_src_pos_R;
        return;
    end

    % 时间戳严格单调递增性检查
    if (timestamp.t_source_L <= align_state_next.last_ts_L) || ...
       (timestamp.t_source_R <= align_state_next.last_ts_R) || ...
       (t_src_pos_L <= align_state_next.last_ts_pos_L) || ...
       (t_src_pos_R <= align_state_next.last_ts_pos_R)
        delay_info.reject_reason = 'PACKET_CORRUPT';
        delay_info.did_update    = false;
        return;
    end

    % 量测健康度与报文完整性检验
    packet_ok = isfield(quality, 'packet_valid') && quality.packet_valid && ...
                isfield(quality, 'current_valid') && all(quality.current_valid) && ...
                isfield(quality, 'position_valid') && all(quality.position_valid) && ...
                all(isfinite(current_cal)) && all(isfinite(current_cmd)) && all(isfinite(position));
    if ~packet_ok
        delay_info.reject_reason = 'PACKET_CORRUPT';
        delay_info.did_update    = false;
        return;
    end

    % 饱和状态检查: 饱和期间禁止回归有效输出，绝对冻结
    is_sat = (isfield(quality, 'is_saturated') && any(quality.is_saturated)) || ...
             (abs(current_cmd(1)) >= 0.95 * opts.Imax) || ...
             (abs(current_cmd(2)) >= 0.95 * opts.Imax);
    if is_sat
        signals_aligned.current_pair_valid       = false;
        signals_aligned.absolute_alignment_valid = false;
        signals_aligned.valid_for_regression     = false;
        signals_aligned.current_cal              = [NaN; NaN];
        signals_aligned.position                 = [NaN; NaN];
        delay_info.did_update                    = false;
        delay_info.reject_reason                 = 'SATURATION';
        return;
    end

    %% 5. 校验通过后，更新序列/时间戳并写入因果历史滑动缓冲区
    align_state_next.last_seq_L     = timestamp.seq_L;
    align_state_next.last_seq_R     = timestamp.seq_R;
    align_state_next.last_seq_pos_L = seq_p_L;
    align_state_next.last_seq_pos_R = seq_p_R;
    align_state_next.last_ts_L      = timestamp.t_source_L;
    align_state_next.last_ts_R      = timestamp.t_source_R;
    align_state_next.last_ts_pos_L  = t_src_pos_L;
    align_state_next.last_ts_pos_R  = t_src_pos_R;

    align_state_next.current_buffer_L      = [current_cal(1); align_state_next.current_buffer_L(1:end-1)];
    align_state_next.current_buffer_R      = [current_cal(2); align_state_next.current_buffer_R(1:end-1)];
    align_state_next.command_buffer_L      = [current_cmd(1); align_state_next.command_buffer_L(1:end-1)];
    align_state_next.command_buffer_R      = [current_cmd(2); align_state_next.command_buffer_R(1:end-1)];
    align_state_next.position_buffer_L     = [position(1); align_state_next.position_buffer_L(1:end-1)];
    align_state_next.position_buffer_R     = [position(2); align_state_next.position_buffer_R(1:end-1)];
    align_state_next.t_source_buffer_L     = [timestamp.t_source_L; align_state_next.t_source_buffer_L(1:end-1)];
    align_state_next.t_source_buffer_R     = [timestamp.t_source_R; align_state_next.t_source_buffer_R(1:end-1)];
    align_state_next.t_source_buffer_pos_L = [t_src_pos_L; align_state_next.t_source_buffer_pos_L(1:end-1)];
    align_state_next.t_source_buffer_pos_R = [t_src_pos_R; align_state_next.t_source_buffer_pos_R(1:end-1)];
    align_state_next.buffer_count = align_state_next.buffer_count + 1;

    %% 6. 三大模式核心时延估计与因果对齐处理
    switch opts.mode

        %% =================================================================
        %% 模式 1: TIMESTAMP (硬件源采样时间戳硬对齐，工程主路径)
        %% =================================================================
        case 'TIMESTAMP'
            delay_info.method = 'TIMESTAMP';

            % 计算各通道离散步数延迟
            dL_meas_steps = round((timestamp.t_recv_L - timestamp.t_source_L) / opts.dt);
            dR_meas_steps = round((timestamp.t_recv_R - timestamp.t_source_R) / opts.dt);
            dposL_steps   = round((t_rcv_pos_L - t_src_pos_L) / opts.dt);
            dposR_steps   = round((t_rcv_pos_R - t_src_pos_R) / opts.dt);

            % 公共对齐参考时刻定义 (所有通道历史最新交集点)
            t_common = min([timestamp.t_source_L, timestamp.t_source_R, t_src_pos_L, t_src_pos_R]);

            % 检查缓冲区有效深度是否足以因果覆盖回溯 (且不超过物理缓冲区最大容量)
            req_depth = max([dL_meas_steps, dR_meas_steps, dposL_steps, dposR_steps, 0]) + 1;
            valid_depth = min(align_state_next.buffer_count, align_state_next.buffer_depth);
            if valid_depth < req_depth
                delay_info.reject_reason             = 'BUFFER_WARMING';
                delay_info.did_update                = false;
                signals_aligned.valid_for_regression = false;
                signals_aligned.current_cal          = [NaN; NaN];
                signals_aligned.position             = [NaN; NaN];
                return;
            end

            % 因果历史样本检索 (满足 t_source <= t_common 的最近历史点)
            idx_L    = find_causal_sample_index(align_state_next.t_source_buffer_L, t_common);
            idx_R    = find_causal_sample_index(align_state_next.t_source_buffer_R, t_common);
            idx_posL = find_causal_sample_index(align_state_next.t_source_buffer_pos_L, t_common);
            idx_posR = find_causal_sample_index(align_state_next.t_source_buffer_pos_R, t_common);

            if isempty(idx_L) || isempty(idx_R) || isempty(idx_posL) || isempty(idx_posR) || ...
               idx_L > valid_depth || idx_R > valid_depth || idx_posL > valid_depth || idx_posR > valid_depth
                delay_info.reject_reason             = 'BUFFER_WARMING';
                delay_info.did_update                = false;
                signals_aligned.valid_for_regression = false;
                signals_aligned.current_cal          = [NaN; NaN];
                signals_aligned.position             = [NaN; NaN];
                return;
            end

            assert(idx_L >= 1 && idx_R >= 1 && idx_posL >= 1 && idx_posR >= 1, '因果索引越界异常');

            % 满足深度后，更新时延状态与输出有效对齐信号
            old_delay = align_state_next.last_trusted_delay;
            new_delay = [dL_meas_steps; dR_meas_steps];

            delay_info.did_update = any(~isfinite(old_delay)) || ...
                                    any(old_delay ~= new_delay);

            align_state_next.d_hat_L            = dL_meas_steps;
            align_state_next.d_hat_R            = dR_meas_steps;
            align_state_next.last_trusted_delay = new_delay;
            align_state_next.is_initialized     = true;

            delay_info.d_meas_hat        = new_delay;
            delay_info.d_total_hat       = delay_info.d_meas_hat;
            delay_info.delta_d_hat       = dL_meas_steps - dR_meas_steps;
            delay_info.confidence        = 1.0;

            iL_align = align_state_next.current_buffer_L(idx_L);
            iR_align = align_state_next.current_buffer_R(idx_R);
            yL_align = align_state_next.position_buffer_L(idx_posL);
            yR_align = align_state_next.position_buffer_R(idx_posR);

            signals_aligned.current_pair_valid        = true;
            signals_aligned.absolute_alignment_valid  = true;
            signals_aligned.valid_for_regression      = true;
            signals_aligned.common_timestamp          = t_common;
            signals_aligned.current_cal               = [iL_align; iR_align];
            signals_aligned.position                  = [yL_align; yR_align];
            signals_aligned.used_index                = [idx_L; idx_R; idx_posL; idx_posR];
            signals_aligned.used_source_timestamp     = [align_state_next.t_source_buffer_L(idx_L); ...
                                                         align_state_next.t_source_buffer_R(idx_R); ...
                                                         align_state_next.t_source_buffer_pos_L(idx_posL); ...
                                                         align_state_next.t_source_buffer_pos_R(idx_posR)];

        %% =================================================================
        %% 模式 2: XCORR_KNOWN_PATH (已知前向路径延迟互相关模式)
        %% =================================================================
        case 'XCORR_KNOWN_PATH'
            delay_info.method = 'XCORR_KNOWN_PATH';

            % 激励充分性检查: 统计指令滑动变化率方差
            N_win = min([align_state_next.buffer_count, opts.xcorr_window_length, length(align_state_next.command_buffer_L)]);
            if N_win < 30
                delay_info.reject_reason = 'BUFFER_WARMING';
                delay_info.did_update    = false;
            else
                cmd_hist_L  = align_state_next.command_buffer_L(1:N_win);
                cmd_hist_R  = align_state_next.command_buffer_R(1:N_win);
                meas_hist_L = align_state_next.current_buffer_L(1:N_win);
                meas_hist_R = align_state_next.current_buffer_R(1:N_win);

                dcmd_L = diff(cmd_hist_L) / opts.dt;
                dcmd_R = diff(cmd_hist_R) / opts.dt;
                var_L  = var(dcmd_L);
                var_R  = var(dcmd_R);

                if (var_L < opts.th_cmd_var) || (var_R < opts.th_cmd_var)
                    delay_info.reject_reason = 'LOW_EXCITATION';
                    delay_info.did_update    = false;
                else
                    % 执行归一化互相关估计
                    [d_tot_L, conf_L, ok_L] = estimate_channel_delay_xcorr(...
                        cmd_hist_L, meas_hist_L, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);
                    [d_tot_R, conf_R, ok_R] = estimate_channel_delay_xcorr(...
                        cmd_hist_R, meas_hist_R, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);

                    if ~ok_L || ~ok_R
                        delay_info.reject_reason = 'PEAK_INSIGNIFICANT';
                        delay_info.did_update    = false;
                    else
                        d_total_cand = [d_tot_L; d_tot_R];
                        d_meas_cand  = d_total_cand - opts.d_path_known;

                        % 负时延拒绝硬断言 (严禁截零)
                        if any(d_meas_cand < 0)
                            delay_info.reject_reason = 'NEGATIVE_DELAY';
                            delay_info.did_update    = false;
                        else
                            delay_info.confidence = min(conf_L, conf_R);
                            align_state_next.last_confidence = delay_info.confidence;

                            % 独立通道确认计数维护
                            for ch = 1:2
                                if isfinite(align_state_next.candidate_delay(ch)) && (d_total_cand(ch) == align_state_next.candidate_delay(ch))
                                    align_state_next.confirm_count(ch) = align_state_next.confirm_count(ch) + 1;
                                else
                                    align_state_next.candidate_delay(ch) = d_total_cand(ch);
                                    align_state_next.confirm_count(ch)   = 1;
                                end
                            end

                            old_trusted_total = align_state_next.last_trusted_total;

                            if all(align_state_next.confirm_count >= opts.N_confirm)
                                first_lock    = any(~isfinite(old_trusted_total));
                                delay_changed = first_lock || any(d_total_cand ~= old_trusted_total);

                                align_state_next.last_trusted_total = d_total_cand;
                                align_state_next.last_trusted_delay = d_meas_cand;
                                align_state_next.d_total_hat        = d_total_cand;
                                align_state_next.d_hat_L            = d_meas_cand(1);
                                align_state_next.d_hat_R            = d_meas_cand(2);
                                align_state_next.is_initialized     = true;

                                delay_info.did_update = delay_changed;
                            end
                        end
                    end
                end
            end

            % 因果对齐执行
            if align_state_next.is_initialized
                d_meas_L = align_state_next.last_trusted_delay(1);
                d_meas_R = align_state_next.last_trusted_delay(2);
                delay_info.d_meas_hat  = [d_meas_L; d_meas_R];
                delay_info.d_total_hat = align_state_next.d_total_hat;
                delay_info.delta_d_hat = d_meas_L - d_meas_R;

                % 绝对对齐与回归准入: 必须存在已知位置延迟模型
                if ~all(isfinite(opts.d_pos_known)) || (opts.require_position_alignment && ~all(isfinite(opts.d_pos_known)))
                    signals_aligned.current_pair_valid       = true;
                    signals_aligned.absolute_alignment_valid = false;
                    signals_aligned.valid_for_regression     = false;
                    signals_aligned.current_cal              = [NaN; NaN];
                    signals_aligned.position                 = [NaN; NaN];
                    delay_info.reject_reason                 = 'POSITION_DELAY_UNKNOWN';
                    return;
                end

                d_tot_L = opts.d_path_known(1) + d_meas_L;
                d_tot_R = opts.d_path_known(2) + d_meas_R;
                d_pos = opts.d_pos_known;
                if numel(d_pos) == 1
                    d_pos_L = d_pos; d_pos_R = d_pos;
                else
                    d_pos_L = d_pos(1); d_pos_R = d_pos(2);
                end

                d_common = max([d_tot_L, d_tot_R, d_pos_L, d_pos_R]);
                req_depth = d_common + 1;
                valid_depth = min(align_state_next.buffer_count, align_state_next.buffer_depth);
                if valid_depth >= req_depth
                    idx_iL   = 1 + d_common - d_tot_L;
                    idx_iR   = 1 + d_common - d_tot_R;
                    idx_posL = 1 + d_common - d_pos_L;
                    idx_posR = 1 + d_common - d_pos_R;

                    assert(idx_iL >= 1 && idx_iL <= align_state_next.buffer_depth, 'idx_iL 越界');
                    assert(idx_iR >= 1 && idx_iR <= align_state_next.buffer_depth, 'idx_iR 越界');
                    assert(idx_posL >= 1 && idx_posL <= align_state_next.buffer_depth, 'idx_posL 越界');
                    assert(idx_posR >= 1 && idx_posR <= align_state_next.buffer_depth, 'idx_posR 越界');

                    iL_align = align_state_next.current_buffer_L(idx_iL);
                    iR_align = align_state_next.current_buffer_R(idx_iR);
                    yL_align = align_state_next.position_buffer_L(idx_posL);
                    yR_align = align_state_next.position_buffer_R(idx_posR);

                    signals_aligned.current_pair_valid        = true;
                    signals_aligned.absolute_alignment_valid  = true;
                    signals_aligned.valid_for_regression      = true;
                    t_iL = align_state_next.t_source_buffer_L(idx_iL);
                    t_iR = align_state_next.t_source_buffer_R(idx_iR);
                    t_pL = align_state_next.t_source_buffer_pos_L(idx_posL);
                    t_pR = align_state_next.t_source_buffer_pos_R(idx_posR);

                    used_timestamps = [t_iL; t_iR; t_pL; t_pR];
                    effective_timestamps = [ ...
                        t_iL - d_tot_L * opts.dt; ...
                        t_iR - d_tot_R * opts.dt; ...
                        t_pL - d_pos_L * opts.dt; ...
                        t_pR - d_pos_R * opts.dt];

                    if all(isfinite([used_timestamps; effective_timestamps]))
                        timestamp_spread = max(effective_timestamps) - min(effective_timestamps);
                        if timestamp_spread > opts.timestamp_alignment_tolerance
                            signals_aligned.current_pair_valid       = false;
                            signals_aligned.absolute_alignment_valid = false;
                            signals_aligned.valid_for_regression     = false;
                            signals_aligned.current_cal              = [NaN; NaN];
                            signals_aligned.position                 = [NaN; NaN];
                            signals_aligned.common_timestamp         = NaN;
                            signals_aligned.used_source_timestamp    = used_timestamps;
                            delay_info.reject_reason                  = 'TIMESTAMP_ALIGNMENT_MISMATCH';
                            return;
                        end

                        signals_aligned.common_timestamp      = min(effective_timestamps);
                        signals_aligned.used_source_timestamp = used_timestamps;
                    else
                        signals_aligned.current_pair_valid       = false;
                        signals_aligned.absolute_alignment_valid = false;
                        signals_aligned.valid_for_regression     = false;
                        signals_aligned.current_cal              = [NaN; NaN];
                        signals_aligned.position                 = [NaN; NaN];
                        signals_aligned.common_timestamp         = NaN;
                        signals_aligned.used_source_timestamp    = used_timestamps;
                        delay_info.reject_reason                  = 'PACKET_CORRUPT';
                        return;
                    end
                    signals_aligned.current_cal               = [iL_align; iR_align];
                    signals_aligned.position                  = [yL_align; yR_align];
                    signals_aligned.used_index                = [idx_iL; idx_iR; idx_posL; idx_posR];
                else
                    delay_info.reject_reason             = 'BUFFER_WARMING';
                    signals_aligned.valid_for_regression = false;
                    signals_aligned.current_cal          = [NaN; NaN];
                    signals_aligned.position             = [NaN; NaN];
                end
            else
                if strcmp(delay_info.reject_reason, 'NONE')
                    delay_info.reject_reason = 'BUFFER_WARMING';
                end
                signals_aligned.valid_for_regression = false;
                signals_aligned.current_cal          = [NaN; NaN];
                signals_aligned.position             = [NaN; NaN];
            end

        %% =================================================================
        %% 模式 3: DIFF_ONLY (未知路径延迟差模对齐降级模式)
        %% =================================================================
        case 'DIFF_ONLY'
            delay_info.method = 'DIFF_ONLY';

            % 路径对称性显式声明检查
            if ~opts.assume_symmetric_path
                delay_info.reject_reason = 'ASYMMETRIC_PATH_UNASSUMED';
                delay_info.did_update    = false;
                delay_info.d_meas_hat    = [NaN; NaN];
                return;
            end

            N_win = min([align_state_next.buffer_count, opts.xcorr_window_length, length(align_state_next.command_buffer_L)]);
            if N_win < 30
                delay_info.reject_reason = 'BUFFER_WARMING';
                delay_info.did_update    = false;
            else
                cmd_hist_L  = align_state_next.command_buffer_L(1:N_win);
                cmd_hist_R  = align_state_next.command_buffer_R(1:N_win);
                meas_hist_L = align_state_next.current_buffer_L(1:N_win);
                meas_hist_R = align_state_next.current_buffer_R(1:N_win);

                dcmd_L = diff(cmd_hist_L) / opts.dt;
                dcmd_R = diff(cmd_hist_R) / opts.dt;
                var_L  = var(dcmd_L);
                var_R  = var(dcmd_R);

                if (var_L < opts.th_cmd_var) || (var_R < opts.th_cmd_var)
                    delay_info.reject_reason = 'LOW_EXCITATION';
                    delay_info.did_update    = false;
                else
                    [d_tot_L, conf_L, ok_L] = estimate_channel_delay_xcorr(...
                        cmd_hist_L, meas_hist_L, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);
                    [d_tot_R, conf_R, ok_R] = estimate_channel_delay_xcorr(...
                        cmd_hist_R, meas_hist_R, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);

                    if ~ok_L || ~ok_R
                        delay_info.reject_reason = 'PEAK_INSIGNIFICANT';
                        delay_info.did_update    = false;
                    else
                        d_total_cand = [d_tot_L; d_tot_R];
                        delay_info.confidence = min(conf_L, conf_R);
                        align_state_next.last_confidence = delay_info.confidence;

                        % 独立通道确认计数维护
                        for ch = 1:2
                            if isfinite(align_state_next.candidate_delay(ch)) && (d_total_cand(ch) == align_state_next.candidate_delay(ch))
                                align_state_next.confirm_count(ch) = align_state_next.confirm_count(ch) + 1;
                            else
                                align_state_next.candidate_delay(ch) = d_total_cand(ch);
                                align_state_next.confirm_count(ch)   = 1;
                            end
                        end

                        old_trusted_total = align_state_next.last_trusted_total;

                        if all(align_state_next.confirm_count >= opts.N_confirm)
                            first_lock    = any(~isfinite(old_trusted_total));
                            delay_changed = first_lock || any(d_total_cand ~= old_trusted_total);

                            align_state_next.last_trusted_total = d_total_cand;
                            align_state_next.d_hat_L            = d_tot_L;
                            align_state_next.d_hat_R            = d_tot_R;
                            delay_info.did_update               = delay_changed;
                            align_state_next.is_initialized     = true;
                        end
                    end
                end
            end

            % 降级约束与绝对红线: 仅报告差模延迟估计，严禁输出对齐电流与回归准入
            delay_info.d_meas_hat = [NaN; NaN];
            if align_state_next.is_initialized
                delay_info.delta_d_hat = align_state_next.d_hat_L - align_state_next.d_hat_R;
                delay_info.d_total_hat = [align_state_next.d_hat_L; align_state_next.d_hat_R];
                signals_aligned.current_pair_delay_estimate_valid = true;
            else
                signals_aligned.current_pair_delay_estimate_valid = false;
            end

            signals_aligned.current_pair_valid       = false;
            signals_aligned.absolute_alignment_valid = false;
            signals_aligned.valid_for_regression     = false;
            signals_aligned.current_cal              = [NaN; NaN];
            signals_aligned.position                 = [NaN; NaN];
            signals_aligned.common_timestamp         = NaN;

        otherwise
            error('未知的对齐工作模式: %s', opts.mode);
    end

    %% 7. 最终有效性一致性保障
    if ~signals_aligned.valid_for_regression
        signals_aligned.current_cal = [NaN; NaN];
        signals_aligned.position    = [NaN; NaN];
    end
end

%% =========================================================================
%% 局部辅助函数 1: 严格因果检索满足 t_source <= t_common 的最近历史点
%% =========================================================================
function idx = find_causal_sample_index(t_buffer, t_target)
    % t_buffer 从 1(最新) 到 end(最旧) 递减
    % 检索满足 t_buffer(i) <= t_target + 1e-12 的最小 i (最新样本)
    mask = (t_buffer <= (t_target + 1e-12)) & isfinite(t_buffer);
    idx_all = find(mask);
    if isempty(idx_all)
        idx = [];
    else
        idx = idx_all(1);
    end
end

%% =========================================================================
%% 局部辅助函数 2: 单通道指令-量测滑动互相关与显著性峰值估计
%% =========================================================================
function [d_est, conf, ok] = estimate_channel_delay_xcorr(cmd_hist, meas_hist, max_delay, peak_margin, peak_min)
    N = length(cmd_hist);
    cmd_zero_mean  = cmd_hist - mean(cmd_hist);
    meas_zero_mean = meas_hist - mean(meas_hist);

    norm_cmd  = norm(cmd_zero_mean);
    norm_meas = norm(meas_zero_mean);

    if (norm_cmd < 1e-6) || (norm_meas < 1e-6)
        d_est = 0; conf = 0.0; ok = false; return;
    end

    corr_vals = zeros(max_delay + 1, 1);
    for d = 0:max_delay
        % 延迟 d 步: meas 滞后于 cmd
        len = N - d;
        if len > 10
            m_seg = meas_zero_mean(1:len);
            c_seg = cmd_zero_mean((1 + d):N);
            num = dot(c_seg, m_seg);
            den = norm(c_seg) * norm(m_seg);
            if den > 1e-9
                corr_vals(d + 1) = num / den;
            end
        end
    end

    [peak1, best_idx] = max(corr_vals);
    d_est = best_idx - 1; % 换算为 0-based 延迟步数
    conf = max(0.0, min(1.0, peak1));

    % 寻找除主峰邻域外的次高局部极大值 (独立次峰)
    % 邻域定义: |d - d_est| <= 1 为主峰同瓣过渡点
    secondary_peaks = [];
    for d_idx = 1:length(corr_vals)
        d_cand = d_idx - 1;
        if abs(d_cand - d_est) > 1
            is_local_max = true;
            if d_idx > 1 && corr_vals(d_idx) < corr_vals(d_idx - 1)
                is_local_max = false;
            end
            if d_idx < length(corr_vals) && corr_vals(d_idx) < corr_vals(d_idx + 1)
                is_local_max = false;
            end
            if is_local_max
                secondary_peaks = [secondary_peaks; corr_vals(d_idx)];
            end
        end
    end

    if isempty(secondary_peaks)
        peak2 = 0.0;
        margin = peak1;
    else
        peak2 = max(secondary_peaks);
        margin = peak1 - peak2;
    end

    if (peak1 >= peak_min) && (margin >= peak_margin)
        ok = true;
    else
        ok = false;
    end
end
