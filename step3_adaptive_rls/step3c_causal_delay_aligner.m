%% STEP3C_CAUSAL_DELAY_ALIGNER.M - 严苛因果历史对齐与通信时延估计器
% =========================================================================
% 架构定位:
% 位于独立电流量测校准器 step3c_current_channel_calibrator 之后、状态变量滤波器 (SVF)
% 与 RLS 估计器之前。
% 负责对左右回采电流量测与光栅尺位置量测建立严格同物理时刻的因果对齐基准。
%
% 接口规范:
% [signals_aligned, align_state_next, delay_info] = ...
%     step3c_causal_delay_aligner( ...
%         current_cal, current_cmd, position, ...
%         timestamp, quality, align_state, opts)
%
% 输入参数:
%   current_cal  - [2x1] 已校准回采电流 [iL_cal; iR_cal] (counts)
%   current_cmd  - [2x1] 控制器输出指令 [iL_cmd; iR_cmd] (counts)
%   position     - [2x1] 光栅尺位置量测 [yL; yR] (m)
%   timestamp    - 时间戳结构体:
%                  .t_source_L, .t_source_R, .t_source_pos (源采样时间戳, s)
%                  .t_recv_L, .t_recv_R, .t_recv_pos (总线接收时间戳, s)
%                  .seq_L, .seq_R, .seq_pos (报文包序号)
%                  .clock_id_L, .clock_id_R, .clock_id_pos (源时钟域标识)
%   quality      - 通道健康度与报文质量结构体:
%                  .is_saturated   - [2x1] logical: 指令饱和标志 [sat_L; sat_R]
%                  .current_valid  - [2x1] logical: 电流有效标志 [valid_L; valid_R]
%                  .position_valid - [2x1] logical: 位置有效标志 [valid_yL; valid_yR]
%                  .packet_valid   - logical: 报文校验有效标志
%   align_state  - 递推内部状态机结构体
%   opts         - 配置结构体:
%                  .dt                     - 采样周期 (默认 0.001 s)
%                  .buffer_depth           - 历史缓冲区深度 (默认 50)
%                  .mode                   - 工作模式 ('TIMESTAMP' | 'XCORR_KNOWN_PATH' | 'DIFF_ONLY')
%                  .d_path_known           - [2x1] 已知前向+驱动综合延迟 [d_path_L; d_path_R] (samples)
%                  .assume_symmetric_path  - logical, DIFF_ONLY 模式前置条件 (默认 false)
%                  .xcorr_window_length    - 互相关滑动窗口长度 (默认 200)
%                  .th_cmd_var             - 互相关最小指令变化率方差门限 (默认 100.0)
%                  .th_peak_margin         - 互相关第一峰第二峰显著性门限 (默认 0.15)
%                  .th_peak_min            - 互相关第一峰最小相关系数门限 (默认 0.60)
%                  .N_confirm              - 迟滞确认步数 (默认 5)
%                  .Imax                   - 硬件最大工作电流 (默认 16000.0 counts)
%                  .max_search_delay       - 最大搜索时滞 (默认 10 samples)
%                  .reset                  - 是否重置状态机 (默认 false)
%
% 输出参数:
%   signals_aligned  - 对齐信号结构体:
%                      .current_pair_valid        - logical: 左右电流差模对齐有效
%                      .absolute_alignment_valid  - logical: 电流-位置绝对时基对齐有效
%                      .valid_for_regression      - logical: 整体是否具备回归输入准入条件
%                      .common_timestamp          - scalar: 公共对齐物理时刻 (s)
%                      .current_cal               - [2x1]: 因果对齐后电流 (counts)
%                      .position                  - [2x1]: 因果对齐后位置 (m)
%   align_state_next - 更新后的跨步递推状态机
%   delay_info       - 诊断与时延识别信息结构体:
%                      .did_update    - logical: 本步是否更新了时延参数
%                      .method        - char: 当前使用的对齐算法模式
%                      .confidence    - scalar: 时延估计置信度 [0, 1]
%                      .reject_reason - 原因码 ('NONE' | 'LOW_EXCITATION' | 'SATURATION' | ...
%                                       'PEAK_INSIGNIFICANT' | 'NEGATIVE_DELAY' | ...
%                                       'ASYMMETRIC_PATH_UNASSUMED' | 'BUFFER_WARMING' | ...
%                                       'PACKET_CORRUPT' | 'CLOCK_MISMATCH' | 'SEQ_ROLLBACK')
%                      .d_total_hat   - [2x1]: 命令到回采的总延迟估计 (samples)
%                      .d_meas_hat    - [2x1]: 纯量测通信延迟估计 (samples)
%                      .delta_d_hat   - scalar: 左右通道差模延迟估计 (samples)
% =========================================================================

function [signals_aligned, align_state_next, delay_info] = ...
    step3c_causal_delay_aligner( ...
        current_cal, current_cmd, position, ...
        timestamp, quality, align_state, opts)

    %% 1. 参数缺省处理与输入契约校验
    if nargin < 7, opts = struct(); end
    if ~isfield(opts, 'dt'), opts.dt = 0.001; end
    if ~isfield(opts, 'xcorr_window_length'), opts.xcorr_window_length = 200; end
    if ~isfield(opts, 'buffer_depth'), opts.buffer_depth = max(300, opts.xcorr_window_length + 50); end
    opts.buffer_depth = max(opts.buffer_depth, opts.xcorr_window_length + 50);
    if ~isfield(opts, 'mode'), opts.mode = 'TIMESTAMP'; end
    if ~isfield(opts, 'd_path_known'), opts.d_path_known = [0.0; 0.0]; end
    if ~isfield(opts, 'assume_symmetric_path'), opts.assume_symmetric_path = false; end
    if ~isfield(opts, 'th_cmd_var'), opts.th_cmd_var = 100.0; end
    if ~isfield(opts, 'th_peak_margin'), opts.th_peak_margin = 0.05; end
    if ~isfield(opts, 'th_peak_min'), opts.th_peak_min = 0.60; end
    if ~isfield(opts, 'N_confirm'), opts.N_confirm = 5; end
    if ~isfield(opts, 'Imax'), opts.Imax = 16000.0; end
    if ~isfield(opts, 'max_search_delay'), opts.max_search_delay = 10; end
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

    %% 2. 状态机跨步初始化与显式重置
    if nargin < 6 || isempty(align_state) || opts.reset
        align_state = struct();
        align_state.mode                 = opts.mode;
        align_state.buffer_depth         = opts.buffer_depth;
        align_state.current_buffer_L     = NaN(opts.buffer_depth, 1);
        align_state.current_buffer_R     = NaN(opts.buffer_depth, 1);
        align_state.command_buffer_L     = NaN(opts.buffer_depth, 1);
        align_state.command_buffer_R     = NaN(opts.buffer_depth, 1);
        align_state.position_buffer_L    = NaN(opts.buffer_depth, 1);
        align_state.position_buffer_R    = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_L    = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_R    = NaN(opts.buffer_depth, 1);
        align_state.t_source_buffer_pos  = NaN(opts.buffer_depth, 1);
        align_state.buffer_count         = 0;
        align_state.d_hat_L              = 0;
        align_state.d_hat_R              = 0;
        align_state.d_total_hat          = [0.0; 0.0];
        align_state.last_trusted_delay   = [0.0; 0.0];
        align_state.candidate_delay      = [0.0; 0.0];
        align_state.confirm_count        = 0;
        align_state.last_seq_L           = -1;
        align_state.last_seq_R           = -1;
        align_state.last_seq_pos         = -1;
        align_state.last_ts_L            = -Inf;
        align_state.last_ts_R            = -Inf;
        align_state.last_ts_pos          = -Inf;
        align_state.last_confidence      = 0.0;
        align_state.is_initialized       = false;
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
    signals_aligned.current_pair_valid        = false;
    signals_aligned.absolute_alignment_valid  = false;
    signals_aligned.valid_for_regression      = false;
    signals_aligned.common_timestamp          = NaN;
    signals_aligned.current_cal               = [NaN; NaN];
    signals_aligned.position                  = [NaN; NaN];

    %% 4. 通道健康度与报文完整性检验
    packet_ok = isfield(quality, 'packet_valid') && quality.packet_valid && ...
                isfield(quality, 'current_valid') && all(quality.current_valid) && ...
                isfield(quality, 'position_valid') && all(quality.position_valid) && ...
                all(isfinite(current_cal)) && all(isfinite(current_cmd)) && all(isfinite(position));

    if ~packet_ok
        delay_info.reject_reason = 'PACKET_CORRUPT';
        return;
    end

    %% 5. 压入因果历史环形/滑动缓冲区 (新样本推入末尾，最旧样本挤出头部)
    % 严格因果写入: 当前样本写入 index 1 (最新)，历史依次后移
    align_state_next.current_buffer_L    = [current_cal(1); align_state_next.current_buffer_L(1:end-1)];
    align_state_next.current_buffer_R    = [current_cal(2); align_state_next.current_buffer_R(1:end-1)];
    align_state_next.command_buffer_L    = [current_cmd(1); align_state_next.command_buffer_L(1:end-1)];
    align_state_next.command_buffer_R    = [current_cmd(2); align_state_next.command_buffer_R(1:end-1)];
    align_state_next.position_buffer_L   = [position(1); align_state_next.position_buffer_L(1:end-1)];
    align_state_next.position_buffer_R   = [position(2); align_state_next.position_buffer_R(1:end-1)];
    align_state_next.t_source_buffer_L   = [timestamp.t_source_L; align_state_next.t_source_buffer_L(1:end-1)];
    align_state_next.t_source_buffer_R   = [timestamp.t_source_R; align_state_next.t_source_buffer_R(1:end-1)];
    align_state_next.t_source_buffer_pos = [timestamp.t_source_pos; align_state_next.t_source_buffer_pos(1:end-1)];

    align_state_next.buffer_count = align_state_next.buffer_count + 1;

    %% 6. 三大模式核心时延估计与因果对齐处理
    switch opts.mode

        %% =================================================================
        %% 模式 1: TIMESTAMP (硬件源采样时间戳硬对齐，工程主路径)
        %% =================================================================
        case 'TIMESTAMP'
            delay_info.method = 'TIMESTAMP';

            % 1.1 时钟域一致性检查
            clock_ok = isequal(timestamp.clock_id_L, timestamp.clock_id_R) && ...
                       isequal(timestamp.clock_id_L, timestamp.clock_id_pos);
            if ~clock_ok
                delay_info.reject_reason = 'CLOCK_MISMATCH';
                return;
            end

            % 1.2 序列号单调递增性检查 (防包乱序与倒退)
            if (align_state_next.last_seq_L >= 0 && timestamp.seq_L < align_state_next.last_seq_L) || ...
               (align_state_next.last_seq_R >= 0 && timestamp.seq_R < align_state_next.last_seq_R) || ...
               (align_state_next.last_seq_pos >= 0 && timestamp.seq_pos < align_state_next.last_seq_pos)
                delay_info.reject_reason = 'SEQ_ROLLBACK';
                return;
            end

            % 1.3 时间戳严格单调检查
            if (timestamp.t_source_L <= align_state_next.last_ts_L) || ...
               (timestamp.t_source_R <= align_state_next.last_ts_R) || ...
               (timestamp.t_source_pos <= align_state_next.last_ts_pos)
                delay_info.reject_reason = 'PACKET_CORRUPT';
                return;
            end

            align_state_next.last_seq_L   = timestamp.seq_L;
            align_state_next.last_seq_R   = timestamp.seq_R;
            align_state_next.last_seq_pos = timestamp.seq_pos;
            align_state_next.last_ts_L    = timestamp.t_source_L;
            align_state_next.last_ts_R    = timestamp.t_source_R;
            align_state_next.last_ts_pos  = timestamp.t_source_pos;

            % 1.4 计算当前量测物理通信延迟 (以采样周期 dt 换算为离散步数)
            dL_meas_steps = round((timestamp.t_recv_L - timestamp.t_source_L) / opts.dt);
            dR_meas_steps = round((timestamp.t_recv_R - timestamp.t_source_R) / opts.dt);
            dpos_steps    = round((timestamp.t_recv_pos - timestamp.t_source_pos) / opts.dt);

            align_state_next.d_hat_L     = dL_meas_steps;
            align_state_next.d_hat_R     = dR_meas_steps;
            delay_info.d_meas_hat        = [dL_meas_steps; dR_meas_steps];
            delay_info.d_total_hat       = delay_info.d_meas_hat;
            delay_info.delta_d_hat       = dL_meas_steps - dR_meas_steps;
            delay_info.confidence        = 1.0;
            delay_info.did_update        = true;

            % 1.5 公共对齐参考时刻定义 (所有通道历史最新交集点)
            t_common = min([timestamp.t_source_L, timestamp.t_source_R, timestamp.t_source_pos]);

            % 1.6 检查缓冲区预热深度是否足以因果覆盖回溯
            % 最大所需回溯步数:
            req_depth = max([dL_meas_steps, dR_meas_steps, dpos_steps, 0]) + 1;
            if align_state_next.buffer_count < req_depth
                delay_info.reject_reason = 'BUFFER_WARMING';
                return;
            end

            % 1.7 因果历史点检索 (从历史缓冲区中精准检索满足 t_source <= t_common 的最近点)
            % 缓冲区 index 1 为最新样本，历史递增
            idx_L   = find_causal_sample_index(align_state_next.t_source_buffer_L, t_common);
            idx_R   = find_causal_sample_index(align_state_next.t_source_buffer_R, t_common);
            idx_pos = find_causal_sample_index(align_state_next.t_source_buffer_pos, t_common);

            if isempty(idx_L) || isempty(idx_R) || isempty(idx_pos)
                delay_info.reject_reason = 'BUFFER_WARMING';
                return;
            end

            % 严格因果检查: 索引必须位于有效缓冲区内，严禁负值或越界
            assert(idx_L >= 1 && idx_R >= 1 && idx_pos >= 1, '因果索引越界异常');

            % 提取严格因果对齐后的信号
            iL_align = align_state_next.current_buffer_L(idx_L);
            iR_align = align_state_next.current_buffer_R(idx_R);
            yL_align = align_state_next.position_buffer_L(idx_pos);
            yR_align = align_state_next.position_buffer_R(idx_pos);

            signals_aligned.current_pair_valid        = true;
            signals_aligned.absolute_alignment_valid  = true;
            signals_aligned.valid_for_regression      = true;
            signals_aligned.common_timestamp          = t_common;
            signals_aligned.current_cal               = [iL_align; iR_align];
            signals_aligned.position                  = [yL_align; yR_align];
            align_state_next.is_initialized           = true;

        %% =================================================================
        %% 模式 2: XCORR_KNOWN_PATH (已知前向路径延迟互相关模式)
        %% =================================================================
        case 'XCORR_KNOWN_PATH'
            delay_info.method = 'XCORR_KNOWN_PATH';

            % 2.1 饱和状态检查
            is_sat = (isfield(quality, 'is_saturated') && any(quality.is_saturated)) || ...
                     (abs(current_cmd(1)) >= 0.95 * opts.Imax) || ...
                     (abs(current_cmd(2)) >= 0.95 * opts.Imax);

            if is_sat
                delay_info.reject_reason = 'SATURATION';
                delay_info.did_update    = false;
            else
                % 2.2 激励充分性检查: 统计指令滑动变化率方差
                N_win = min([align_state_next.buffer_count, opts.xcorr_window_length, length(align_state_next.command_buffer_L)]);
                if N_win < 30
                    delay_info.reject_reason = 'BUFFER_WARMING';
                else
                    cmd_hist_L = align_state_next.command_buffer_L(1:N_win);
                    cmd_hist_R = align_state_next.command_buffer_R(1:N_win);
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
                        % 2.3 执行归一化互相关估计
                        [d_tot_L, conf_L, ok_L] = estimate_channel_delay_xcorr(...
                            cmd_hist_L, meas_hist_L, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);
                        [d_tot_R, conf_R, ok_R] = estimate_channel_delay_xcorr(...
                            cmd_hist_R, meas_hist_R, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);

                        if ~ok_L || ~ok_R
                            delay_info.reject_reason = 'PEAK_INSIGNIFICANT';
                            delay_info.did_update    = false;
                        else
                            d_total_cand = [d_tot_L; d_tot_R];
                            % 2.4 纯通信延迟计算: d_meas = d_total - d_path_known
                            d_meas_cand = d_total_cand - opts.d_path_known;

                            % 2.5 负时延拒绝硬断言 (严禁截零)
                            if any(d_meas_cand < 0)
                                delay_info.reject_reason = 'NEGATIVE_DELAY';
                                delay_info.did_update    = false;
                            else
                                delay_info.confidence = min(conf_L, conf_R);
                                align_state_next.last_confidence = delay_info.confidence;
                                % 2.6 迟滞确认机制 (防单步跳变)
                                if isequal(d_total_cand, align_state_next.candidate_delay)
                                    align_state_next.confirm_count = align_state_next.confirm_count + 1;
                                else
                                    align_state_next.candidate_delay = d_total_cand;
                                    align_state_next.confirm_count   = 1;
                                end

                                if align_state_next.confirm_count >= opts.N_confirm
                                    align_state_next.last_trusted_delay = d_meas_cand;
                                    align_state_next.d_total_hat        = d_total_cand;
                                    align_state_next.d_hat_L            = d_meas_cand(1);
                                    align_state_next.d_hat_R            = d_meas_cand(2);
                                    delay_info.did_update               = true;
                                    align_state_next.is_initialized     = true;
                                end
                            end
                        end
                    end
                end
            end

            % 使用已确认的可信延迟执行因果对齐
            if align_state_next.is_initialized
                d_meas_L = align_state_next.last_trusted_delay(1);
                d_meas_R = align_state_next.last_trusted_delay(2);
                delay_info.d_meas_hat  = [d_meas_L; d_meas_R];
                delay_info.d_total_hat = align_state_next.d_total_hat;
                delay_info.delta_d_hat = d_meas_L - d_meas_R;

                dmax = max(d_meas_L, d_meas_R);
                dL_extra = dmax - d_meas_L;
                dR_extra = dmax - d_meas_R;

                req_depth = dmax + 2;
                if align_state_next.buffer_count >= req_depth
                    % 较快电流通道补齐额外滞后到 dmax，位置通道因果滞后 dmax
                    idx_iL = 1 + dL_extra;
                    idx_iR = 1 + dR_extra;
                    idx_pos = 1 + dmax;

                    iL_align = align_state_next.current_buffer_L(idx_iL);
                    iR_align = align_state_next.current_buffer_R(idx_iR);
                    yL_align = align_state_next.position_buffer_L(idx_pos);
                    yR_align = align_state_next.position_buffer_R(idx_pos);

                    signals_aligned.current_pair_valid        = true;
                    signals_aligned.absolute_alignment_valid  = true;
                    signals_aligned.valid_for_regression      = true;
                    signals_aligned.common_timestamp          = (align_state_next.buffer_count - 1 - dmax) * opts.dt;
                    signals_aligned.current_cal               = [iL_align; iR_align];
                    signals_aligned.position                  = [yL_align; yR_align];
                else
                    delay_info.reject_reason = 'BUFFER_WARMING';
                end
            else
                if strcmp(delay_info.reject_reason, 'NONE')
                    delay_info.reject_reason = 'BUFFER_WARMING';
                end
            end

        %% =================================================================
        %% 模式 3: DIFF_ONLY (未知路径延迟差模对齐降级模式)
        %% =================================================================
        case 'DIFF_ONLY'
            delay_info.method = 'DIFF_ONLY';

            % 3.1 路径对称性显式声明检查
            if ~opts.assume_symmetric_path
                delay_info.reject_reason = 'ASYMMETRIC_PATH_UNASSUMED';
                delay_info.did_update    = false;
                delay_info.d_meas_hat    = [NaN; NaN];
            else
                % 3.2 估算双通道总时延差模 Delta d_total
                N_win = min([align_state_next.buffer_count, opts.xcorr_window_length, length(align_state_next.command_buffer_L)]);
                if N_win < 30
                    delay_info.reject_reason = 'BUFFER_WARMING';
                else
                    cmd_hist_L  = align_state_next.command_buffer_L(1:N_win);
                    cmd_hist_R  = align_state_next.command_buffer_R(1:N_win);
                    meas_hist_L = align_state_next.current_buffer_L(1:N_win);
                    meas_hist_R = align_state_next.current_buffer_R(1:N_win);

                    [d_tot_L, conf_L, ok_L] = estimate_channel_delay_xcorr(...
                        cmd_hist_L, meas_hist_L, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);
                    [d_tot_R, conf_R, ok_R] = estimate_channel_delay_xcorr(...
                        cmd_hist_R, meas_hist_R, opts.max_search_delay, opts.th_peak_margin, opts.th_peak_min);

                    if ok_L && ok_R
                        delta_d = d_tot_L - d_tot_R;
                        delay_info.delta_d_hat = delta_d;
                        delay_info.d_total_hat = [d_tot_L; d_tot_R];
                        delay_info.confidence  = min(conf_L, conf_R);
                        align_state_next.last_confidence = delay_info.confidence;
                        delay_info.did_update  = true;
                        align_state_next.d_hat_L = d_tot_L;
                        align_state_next.d_hat_R = d_tot_R;
                        align_state_next.is_initialized = true;
                    end
                end
            end

            % 3.3 降级约束与绝对红线: 仅允许电流差模相对补齐，严禁输出绝对时延与回归准入
            delay_info.d_meas_hat = [NaN; NaN]; % 绝对测量时延不可辨识，强制输出 NaN

            if align_state_next.is_initialized && opts.assume_symmetric_path
                signals_aligned.current_pair_valid = true;
            else
                signals_aligned.current_pair_valid = false;
            end

            signals_aligned.absolute_alignment_valid = false;
            signals_aligned.valid_for_regression     = false;

            % 回归无效期输出强制为 [NaN; NaN]，下游 SVF/RLS 必须完全冻结更新
            signals_aligned.current_cal      = [NaN; NaN];
            signals_aligned.position         = [NaN; NaN];
            signals_aligned.common_timestamp = NaN;

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
        idx = idx_all(1); % 满足因果约束的最靠近目标时刻的点
    end
end

%% =========================================================================
%% 局部辅助函数 2: 单通道指令-量测滑动互相关与显著性峰值估计
%% =========================================================================
function [d_est, conf, ok] = estimate_channel_delay_xcorr(cmd_hist, meas_hist, max_delay, peak_margin, peak_min)
    N = length(cmd_hist);
    cmd_zero_mean = cmd_hist - mean(cmd_hist);
    meas_zero_mean = meas_hist - mean(meas_hist);

    norm_cmd = norm(cmd_zero_mean);
    norm_meas = norm(meas_zero_mean);

    if (norm_cmd < 1e-6) || (norm_meas < 1e-6)
        d_est = 0; conf = 0.0; ok = false; return;
    end

    corr_vals = zeros(max_delay + 1, 1);
    for d = 0:max_delay
        % 延迟 d 步: meas 滞后于 cmd (meas(1) = meas(k) 对应 cmd(1+d) = cmd(k-d))
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
