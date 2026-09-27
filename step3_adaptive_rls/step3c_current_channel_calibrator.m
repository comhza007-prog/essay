function [current_cal, calib_state_next, calib_info] = ...
    step3c_current_channel_calibrator(current_raw, cmd, motion, calib_state, opts)
% STEP3C_CURRENT_CHANNEL_CALIBRATOR - 独立电流量测通道校准器 (静态零偏与增益匹配)
% =========================================================================
% 架构定位:
% 位于物理传感器回采与因果时延对齐器之间，严格因果单向递推。
% C4-A 阶段专职负责静态霍尔传感器零偏的稳健中位数/Hampel识别与冻结锁定。
%
% 接口调用规范:
% [current_cal, calib_state_next, calib_info] = ...
%     step3c_current_channel_calibrator(current_raw, cmd, motion, calib_state, opts)
%
% 输入参数:
%   current_raw - [2x1] 左右通道原始测量电流 [iL_meas; iR_meas] (counts)
%   cmd         - 结构体，控制器输出电流指令:
%                 .iL (counts), .iR (counts)
%   motion      - 结构体，台车刚体运动状态与驱动使能标志:
%                 .vG (m/s), .omega (rad/s), .aG (m/s^2),
%                 .drive_torque_disabled (logical, true 表示驱动输出/PWM关闭或处于零转矩模式)
%   calib_state - 结构体，校准器内部跨步递推状态机:
%                 .mode ('IDLE' | 'ACCUMULATING' | 'FROZEN')
%                 .buffer_L, .buffer_R (历史累计缓冲区)
%                 .valid_count (当前连续合法样本计数)
%                 .bias_hat ([2x1] 估计零偏 [bias_L; bias_R], counts)
%                 .gain_scale ([2x1] 增益比例，C4-A 固定为 [1; 1])
%                 .is_calibrated (logical, 是否已完成标定并冻结)
%   opts        - 配置参数结构体 (可选):
%                 .calibration_duration (s, 默认 0.5)
%                 .dt (s, 默认 0.001)
%                 .N_min (最小有效样本量，默认 ceil(calibration_duration/dt) = 500)
%                 .min_retained_ratio (Hampel 剔除后最小保留比例，默认 0.90)
%                 .th_cmd (电流指令零门限 counts, 默认 1e-4)
%                 .th_v (速度零门限 m/s, 默认 1e-5)
%                 .th_omega (角速度零门限 rad/s, 默认 1e-5)
%                 .th_a (加速度零门限 m/s^2, 默认 1e-5)
%                 .hampel_nsigma (Hampel 门限倍数，默认 3.0)
%                 .reset (logical, 是否强制重置为 IDLE，默认 false)
%
% 输出参数:
%   current_cal      - [2x1] 校准后电流 [iL_cal; iR_cal] (counts)
%   calib_state_next - 更新后的校准器状态结构体
%   calib_info       - 辅助诊断信息结构体:
%                      .is_calibrated (logical)
%                      .bias_hat ([2x1])
%                      .gain_scale ([2x1])
%                      .mode (char)
%                      .reject_reason ('NONE' | 'DRIVE_ENABLED' | 'CMD_NONZERO' | ...
%                                      'MOTION_NONZERO' | 'NONFINITE_INPUT' | 'INSUFFICIENT_SAMPLES')
%                      .is_admissible (logical)
%                      .N_eff_L, .N_eff_R (保留有效样本数)
% =========================================================================

    %% 1. 参数缺省处理与规范化
    if nargin < 5, opts = struct(); end
    if ~isfield(opts, 'calibration_duration'), opts.calibration_duration = 0.5; end
    if ~isfield(opts, 'dt'), opts.dt = 0.001; end
    if ~isfield(opts, 'N_min'), opts.N_min = ceil(opts.calibration_duration / opts.dt); end
    if ~isfield(opts, 'min_retained_ratio'), opts.min_retained_ratio = 0.90; end
    if ~isfield(opts, 'th_cmd'), opts.th_cmd = 1e-4; end
    if ~isfield(opts, 'th_v'), opts.th_v = 1e-5; end
    if ~isfield(opts, 'th_omega'), opts.th_omega = 1e-5; end
    if ~isfield(opts, 'th_a'), opts.th_a = 1e-5; end
    if ~isfield(opts, 'hampel_nsigma'), opts.hampel_nsigma = 3.0; end
    if ~isfield(opts, 'reset'), opts.reset = false; end

    assert(isnumeric(current_raw) && isreal(current_raw) && ...
           numel(current_raw) == 2, ...
           'current_raw必须是包含两个实数元素的向量');
    current_raw = current_raw(:);

    %% 2. 状态机初始化与显式重置
    if nargin < 4 || isempty(calib_state) || opts.reset
        calib_state = struct();
        calib_state.mode           = 'IDLE';
        calib_state.buffer_L       = [];
        calib_state.buffer_R       = [];
        calib_state.valid_count    = 0;
        calib_state.bias_hat       = [0.0; 0.0];
        calib_state.gain_scale     = [1.0; 1.0]; % C4-A 阶段严格固定为 [1; 1]
        calib_state.is_calibrated  = false;
        calib_state.last_valid_raw = [0.0; 0.0];
        calib_state.N_eff_L        = 0;
        calib_state.N_eff_R        = 0;
    end

    if ~isfield(calib_state, 'is_calibrated'), calib_state.is_calibrated = false; end
    if ~isfield(calib_state, 'last_valid_raw') || isempty(calib_state.last_valid_raw)
        calib_state.last_valid_raw = [0.0; 0.0];
    end
    if ~isfield(calib_state, 'N_eff_L'), calib_state.N_eff_L = 0; end
    if ~isfield(calib_state, 'N_eff_R'), calib_state.N_eff_R = 0; end

    calib_state_next = calib_state;
    % C4-A 阶段红线约束: 绝不在零偏模块暗含增益校正
    calib_state_next.gain_scale = [1.0; 1.0];

    calib_info = struct();
    calib_info.is_admissible = false;
    calib_info.reject_reason = 'NONE';
    calib_info.did_update    = false;
    calib_info.N_eff_L       = calib_state_next.N_eff_L;
    calib_info.N_eff_R       = calib_state_next.N_eff_R;

    %% 3. 统一输入准入条件判据 (在 FROZEN 之前执行，客观诊断当前样本)
    drive_flag_valid = isscalar(motion.drive_torque_disabled) && ...
        (islogical(motion.drive_torque_disabled) || ...
        (isnumeric(motion.drive_torque_disabled) && ...
         isfinite(motion.drive_torque_disabled) && ...
         any(motion.drive_torque_disabled == [0, 1])));

    is_finite_in = all(isfinite(current_raw)) && ...
                   isfinite(cmd.iL) && isfinite(cmd.iR) && ...
                   isfinite(motion.vG) && isfinite(motion.omega) && isfinite(motion.aG) && ...
                   drive_flag_valid;

    if ~is_finite_in
        calib_info.reject_reason = 'NONFINITE_INPUT';
    elseif ~motion.drive_torque_disabled
        calib_info.reject_reason = 'DRIVE_ENABLED';
    elseif (abs(cmd.iL) > opts.th_cmd) || (abs(cmd.iR) > opts.th_cmd)
        calib_info.reject_reason = 'CMD_NONZERO';
    elseif (abs(motion.vG) > opts.th_v) || (abs(motion.omega) > opts.th_omega) || (abs(motion.aG) > opts.th_a)
        calib_info.reject_reason = 'MOTION_NONZERO';
    else
        calib_info.reject_reason = 'NONE';
        calib_info.is_admissible = true;
    end

    % 非有限值输入保护
    raw_safe = current_raw;
    if any(~isfinite(raw_safe))
        raw_safe(~isfinite(raw_safe)) = calib_state_next.last_valid_raw(~isfinite(raw_safe));
    else
        calib_state_next.last_valid_raw = current_raw;
    end

    %% 4. 状态机流转逻辑
    if strcmp(calib_state.mode, 'FROZEN')
        % -----------------------------------------------------------------
        % 状态 FROZEN: 标定完成，参数永久冻结，绝不被运动电流污染
        % -----------------------------------------------------------------
        calib_info.is_calibrated = true;
        calib_info.bias_hat      = calib_state_next.bias_hat;
        calib_info.gain_scale    = calib_state_next.gain_scale;
        calib_info.mode          = 'FROZEN';
        calib_info.did_update    = false; % 冻结状态下绝不发生参数更新

        % 扣除已冻结零偏并应用增益直通
        current_cal = (raw_safe - calib_state_next.bias_hat) ./ calib_state_next.gain_scale;
        return;
    end

    if ~calib_info.is_admissible
        % 准入失败: 若先前处于 ACCUMULATING，立即中止并清空窗口，回退至 IDLE
        if strcmp(calib_state.mode, 'ACCUMULATING')
            calib_state_next.mode        = 'IDLE';
            calib_state_next.buffer_L    = [];
            calib_state_next.buffer_R    = [];
            calib_state_next.valid_count = 0;
        else
            calib_state_next.mode        = 'IDLE';
        end
        calib_info.did_update = false;
    else
        % 准入成功: 推进累计样本
        calib_state_next.mode = 'ACCUMULATING';
        calib_state_next.buffer_L = [calib_state_next.buffer_L; current_raw(1)];
        calib_state_next.buffer_R = [calib_state_next.buffer_R; current_raw(2)];
        calib_state_next.valid_count = calib_state_next.valid_count + 1;
        calib_info.did_update = true;

        % 检查是否满足最小采样样本量要求
        if calib_state_next.valid_count >= opts.N_min
            % 执行 Hampel / MAD 稳健统计过滤
            [clean_L, N_eff_L] = robust_hampel_filter(calib_state_next.buffer_L, opts.hampel_nsigma);
            [clean_R, N_eff_R] = robust_hampel_filter(calib_state_next.buffer_R, opts.hampel_nsigma);

            calib_info.N_eff_L = N_eff_L;
            calib_info.N_eff_R = N_eff_R;
            calib_state_next.N_eff_L = N_eff_L;
            calib_state_next.N_eff_R = N_eff_R;

            N_req = ceil(opts.N_min * opts.min_retained_ratio);

            if (N_eff_L >= N_req) && (N_eff_R >= N_req)
                % 满足保留率: 取稳健中位数完成标定并永久冻结
                bias_hat_L = median(clean_L);
                bias_hat_R = median(clean_R);

                calib_state_next.bias_hat      = [bias_hat_L; bias_hat_R];
                calib_state_next.is_calibrated = true;
                calib_state_next.mode          = 'FROZEN';

                % 释放内存缓冲区
                calib_state_next.buffer_L      = [];
                calib_state_next.buffer_R      = [];
            else
                % 有效样本保留率不足 (异常值过多): 拒绝标定并清空回退 IDLE
                calib_info.reject_reason     = 'INSUFFICIENT_SAMPLES';
                calib_info.did_update        = false;
                calib_state_next.mode        = 'IDLE';
                calib_state_next.buffer_L    = [];
                calib_state_next.buffer_R    = [];
                calib_state_next.valid_count = 0;
            end
        end
    end

    %% 4. 打包输出与信号校正
    calib_info.is_calibrated = calib_state_next.is_calibrated;
    calib_info.bias_hat      = calib_state_next.bias_hat;
    calib_info.gain_scale    = calib_state_next.gain_scale;
    calib_info.mode          = calib_state_next.mode;

    % 若未完成校准，bias_hat 保持 [0; 0]，实现安全直通
    current_cal = (raw_safe - calib_state_next.bias_hat) ./ calib_state_next.gain_scale;
end

%% =========================================================================
%% 内部稳健过滤核心算法: Hampel / MAD 离群点剔除器
%% =========================================================================
function [clean_vec, N_eff] = robust_hampel_filter(x, nsigma)
    if isempty(x)
        clean_vec = [];
        N_eff = 0;
        return;
    end
    x = x(isfinite(x));
    if isempty(x)
        clean_vec = [];
        N_eff = 0;
        return;
    end
    med_val = median(x);
    mad_val = median(abs(x - med_val));
    sigma_est = 1.4826 * mad_val;

    if sigma_est < 1e-12
        % 方差极小无异常离群点
        clean_vec = x;
        N_eff = numel(x);
    else
        valid_mask = (abs(x - med_val) <= nsigma * sigma_est);
        clean_vec = x(valid_mask);
        N_eff = sum(valid_mask);
    end
end
