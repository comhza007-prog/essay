function [current_corrected, calib_state_next, calib_info] = ...
    step3c_gain_calibrator(current_in, current_ref, motion, calib_state, opts)
% =========================================================================
% STEP 3C-4 Gate C4-C: 电流通道增益校准与可辨识性界定器
%
% 物理建模与可辨识性设计规范:
% 1. 数学可辨识性边界与混淆定理:
%    刚架动力学推力模型: F_i = Kf_i * i_true,i
%    回采电流量测模型:   i_meas,i = g_i * i_true,i = (1 + delta_g_i) * i_true,i
%    表观推力系数:       F_i = (Kf_i / g_i) * i_meas,i = Kf_app,i * i_meas,i
%    纯回采量测 (电流+位置/加速度) 在数学上不可解耦传感器增益误差 delta_g 与推力不对称 Delta_Kf!
% 2. 三大标定模式分层契约:
%    - 'ORACLE_GAIN': 仿真已知真实增益真值，仅验证算法数学正确性
%    - 'EXTERNAL_REFERENCE': 模拟工程外置标准表/分流器/基准源，执行工程真实标定
%    - 'RELATIVE_BALANCE_ASSUMED': 仅验证对称运行假设，输出标记为 UNIDENTIFIABLE / ASSUMPTION_ONLY，
%                                  严禁宣称绝对标定，严禁进入 C8A-eng 主闭环
% 3. 异常工况零伪装硬冻结:
%    在电流饱和、低激励、NaN/Inf 或时延未确认时，严格 did_update=false, is_frozen=true
% =========================================================================

    %% 1. 配置参数缺省处理与输入契约校验
    if nargin < 5 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'mode'), opts.mode = 'ORACLE_GAIN'; end
    if ~isfield(opts, 'true_gains'), opts.true_gains = [1.0; 1.0]; end
    if ~isfield(opts, 'Imax'), opts.Imax = 16000.0; end
    if ~isfield(opts, 'th_sat'), opts.th_sat = 0.95 * opts.Imax; end
    if ~isfield(opts, 'th_cmd_var'), opts.th_cmd_var = 50.0; end
    if ~isfield(opts, 'th_current_min'), opts.th_current_min = 50.0; end % 最小激励电流幅值 (counts)
    if ~isfield(opts, 'N_min'), opts.N_min = 200; end                   % 最小有效标定样本数
    if ~isfield(opts, 'max_asymmetry_range'), opts.max_asymmetry_range = 0.0020; end % 标称最大允许差模 0.20%
    if ~isfield(opts, 'reference_kind'), opts.reference_kind = 'UNKNOWN_REFERENCE'; end
    if ~isfield(opts, 'calibration_profile'), opts.calibration_profile = 'C4C_DEFAULT'; end
    if ~isfield(opts, 'Kf_nominal'), opts.Kf_nominal = 0.00539; end     % 标称推力系数 N/count
    if ~isfield(opts, 'delay_confirmed'), opts.delay_confirmed = true; end % C4-B 时延确认标志
    if ~isfield(opts, 'reset'), opts.reset = false; end

    assert(isnumeric(current_in) && isreal(current_in) && numel(current_in) == 2, ...
        'current_in 必须为包含左右两轴电流的实数 2 元素向量');
    current_in = current_in(:);

    if nargin < 2 || isempty(current_ref)
        current_ref = [NaN; NaN];
    else
        current_ref = current_ref(:);
    end

    if nargin < 3 || isempty(motion)
        motion = struct('vG', 0.0, 'omega', 0.0, 'aG', 0.0, 'drive_torque_disabled', false);
    end

    %% 2. 状态机跨步初始化与显式重置
    if nargin < 4 || isempty(calib_state) || opts.reset
        calib_state = struct();
        calib_state.mode                   = opts.mode;
        calib_state.gain_hat               = [1.0; 1.0];
        calib_state.gain_residual          = [0.0; 0.0];
        calib_state.identifiability_status = 'NONE';
        calib_state.gain_source            = 'NONE';
        calib_state.accum_ratio_L          = zeros(opts.N_min * 2, 1);
        calib_state.accum_ratio_R          = zeros(opts.N_min * 2, 1);
        calib_state.sample_count           = 0;
        calib_state.is_frozen              = false;
        calib_state.is_calibrated          = false;
        calib_state.apparent_delta_kf      = 0.0;
        calib_state.freeze_latched         = false;
        calib_state.freeze_reason          = 'NONE';
    end

    if ~isfield(calib_state, 'freeze_latched')
        calib_state.freeze_latched = false;
        calib_state.freeze_reason  = 'NONE';
    end

    calib_state_next = calib_state;
    calib_state_next.mode = opts.mode;

    %% 2.1 故障锁存前置检查 (未显式 reset 前刚性保持硬拦截，禁止偷偷恢复)
    if isfield(calib_state_next, 'freeze_latched') && ...
            calib_state_next.freeze_latched && ~opts.reset

        current_corrected = current_in;

        calib_info = struct();
        calib_info.mode                   = calib_state_next.mode;
        calib_info.identifiability_status = calib_state_next.identifiability_status;
        calib_info.gain_source            = calib_state_next.gain_source;
        calib_info.calibration_profile    = opts.calibration_profile;
        calib_info.reference_kind         = opts.reference_kind;
        calib_info.did_update             = false;
        calib_info.is_frozen              = true;
        calib_info.reject_reason          = calib_state_next.freeze_reason;
        calib_info.gain_residual          = calib_state_next.gain_residual;
        calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

        return;
    end

    %% 3. 初始化诊断与输出结构体 (全 10 项规范字段)
    calib_info = struct();
    calib_info.mode                   = opts.mode;
    calib_info.identifiability_status = calib_state_next.identifiability_status;
    calib_info.gain_source            = calib_state_next.gain_source;
    calib_info.calibration_profile    = opts.calibration_profile;
    calib_info.reference_kind         = opts.reference_kind;
    calib_info.did_update             = false;
    calib_info.is_frozen              = calib_state_next.is_frozen;
    calib_info.reject_reason          = 'NONE';
    calib_info.gain_residual          = calib_state_next.gain_residual;
    calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

    current_corrected = current_in; % 默认安全直通

    %% 4. 前置严密有效性校验 (门禁过滤与故障锁存)
    % 4.1 数值非有限检查 (NaN/Inf) -> 故障锁存
    if any(~isfinite(current_in))
        calib_info.reject_reason          = 'NONFINITE_INPUT';
        calib_info.identifiability_status = 'INVALID_INPUT';
        calib_info.did_update             = false;
        calib_info.is_frozen              = true;
        calib_state_next.is_frozen        = true;
        calib_state_next.freeze_latched   = true;
        calib_state_next.freeze_reason    = 'NONFINITE_INPUT';
        current_corrected                 = current_in;
        return;
    end

    % 4.2 电流饱和刚性拦截 (严禁在饱和区更新与校正) -> 故障锁存
    if any(abs(current_in) >= opts.th_sat)
        calib_info.reject_reason          = 'SATURATION';
        calib_info.did_update             = false;
        calib_info.is_frozen              = true;
        calib_state_next.is_frozen        = true;
        calib_state_next.freeze_latched   = true;
        calib_state_next.freeze_reason    = 'SATURATION';
        current_corrected                 = current_in;
        return;
    end

    % 4.3 前置通信时延因果对齐状态检查 (Gate C4-B 联动) -> 故障锁存
    if ~opts.delay_confirmed
        calib_info.reject_reason          = 'UNCONFIRMED_DELAY';
        calib_info.did_update             = false;
        calib_info.is_frozen              = true;
        calib_state_next.is_frozen        = true;
        calib_state_next.freeze_latched   = true;
        calib_state_next.freeze_reason    = 'UNCONFIRMED_DELAY';
        current_corrected                 = current_in;
        return;
    end

    %% 5. 三大独立校正模式执行
    switch opts.mode

        %% -----------------------------------------------------------------
        %% 模式 1: ORACLE_GAIN (仿真已知真值，数学算法正确性验证)
        %% -----------------------------------------------------------------
        case 'ORACLE_GAIN'
            assert(numel(opts.true_gains) == 2 && all(isfinite(opts.true_gains)) && all(opts.true_gains > 0), ...
                'ORACLE_GAIN 模式必须提供双轴正有限 true_gains 向量');
            
            calib_state_next.gain_hat               = opts.true_gains(:);
            calib_state_next.gain_source            = 'ORACLE_SIM_PARAM';
            calib_state_next.identifiability_status = 'CALIBRATED_ORACLE_SIM';
            calib_state_next.is_calibrated          = true;
            calib_state_next.is_frozen              = true;
            calib_state_next.freeze_latched         = false;
            calib_state_next.freeze_reason          = 'NONE';
            calib_state_next.gain_residual          = abs(calib_state_next.gain_hat - [1.0; 1.0]);
            calib_state_next.apparent_delta_kf      = opts.Kf_nominal * (calib_state_next.gain_hat(1) - calib_state_next.gain_hat(2));

            calib_info.mode                   = 'ORACLE_GAIN';
            calib_info.identifiability_status = 'CALIBRATED_ORACLE_SIM';
            calib_info.gain_source            = 'ORACLE_SIM_PARAM';
            calib_info.did_update             = ~calib_state.is_calibrated; % 首次锁定报告 true
            calib_info.is_frozen              = true;
            calib_info.reject_reason          = 'NONE';
            calib_info.gain_residual          = calib_state_next.gain_residual;
            calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

            current_corrected = current_in ./ calib_state_next.gain_hat;

        %% -----------------------------------------------------------------
        %% 模式 2: EXTERNAL_REFERENCE (外置标准源工程标定)
        %% -----------------------------------------------------------------
        case 'EXTERNAL_REFERENCE'
            % 如果已经完成标定并处于冻结状态，直接应用锁定增益
            if calib_state_next.is_calibrated && calib_state_next.is_frozen
                calib_info.mode                   = 'EXTERNAL_REFERENCE';
                calib_info.identifiability_status = calib_state_next.identifiability_status;
                calib_info.gain_source            = calib_state_next.gain_source;
                calib_info.calibration_profile    = opts.calibration_profile;
                calib_info.reference_kind         = opts.reference_kind;
                calib_info.did_update             = false; % 稳态防重更
                calib_info.is_frozen              = true;
                calib_info.reject_reason          = 'NONE';
                calib_info.gain_residual          = calib_state_next.gain_residual;
                calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

                current_corrected = current_in ./ calib_state_next.gain_hat;
                return;
            end

            % 外置基准必须有限且有效
            if any(~isfinite(current_ref))
                calib_info.reject_reason          = 'NONFINITE_INPUT';
                calib_info.identifiability_status = 'INVALID_INPUT';
                calib_info.did_update             = false;
                calib_info.is_frozen              = true;
                calib_state_next.is_frozen        = true;
                calib_state_next.freeze_latched   = true;
                calib_state_next.freeze_reason    = 'NONFINITE_INPUT';
                current_corrected                 = current_in;
                return;
            end

            % 激励充分性检查: 电流幅值必须超越最小量程以防除零噪声放大
            if any(abs(current_ref) < opts.th_current_min) || any(abs(current_in) < opts.th_current_min)
                calib_info.reject_reason          = 'LOW_EXCITATION';
                calib_info.identifiability_status = 'UNIDENTIFIABLE';
                calib_info.did_update             = false;
                calib_info.is_frozen              = true;
                calib_state_next.freeze_latched   = false;
                calib_state_next.freeze_reason    = 'NONE';
                current_corrected                 = current_in;
                return;
            end

            % 累计有效比值样本
            calib_state_next.sample_count = calib_state_next.sample_count + 1;
            idx = calib_state_next.sample_count;
            if idx > length(calib_state_next.accum_ratio_L)
                calib_state_next.accum_ratio_L = [calib_state_next.accum_ratio_L; zeros(opts.N_min, 1)];
                calib_state_next.accum_ratio_R = [calib_state_next.accum_ratio_R; zeros(opts.N_min, 1)];
            end
            calib_state_next.accum_ratio_L(idx) = current_in(1) / current_ref(1);
            calib_state_next.accum_ratio_R(idx) = current_in(2) / current_ref(2);

            % 样本充足时进行稳健估计
            if calib_state_next.sample_count >= opts.N_min
                ratios_L = calib_state_next.accum_ratio_L(1:calib_state_next.sample_count);
                ratios_R = calib_state_next.accum_ratio_R(1:calib_state_next.sample_count);

                % Hampel / 中位数稳健增益估计
                gL_hat = median(ratios_L);
                gR_hat = median(ratios_R);

                delta_g_diff = abs(gL_hat - gR_hat);

                switch opts.reference_kind
                    case 'SIMULATED_EXTERNAL_REFERENCE'
                        gain_source_name = 'EXTERNAL_REFERENCE_SIM';
                    case 'HARDWARE_EXTERNAL_REFERENCE'
                        gain_source_name = 'EXTERNAL_HARDWARE_SOURCE';
                    otherwise
                        gain_source_name = 'EXTERNAL_REFERENCE_UNSPECIFIED';
                end

                % 差模分区判定: 超出最大允许设计校正范围则判定为超标并拒绝
                if delta_g_diff > opts.max_asymmetry_range
                    calib_state_next.identifiability_status = 'OUT_OF_CALIBRATION_RANGE';
                    calib_state_next.gain_source            = 'NONE';
                    calib_state_next.is_calibrated          = false;
                    calib_state_next.is_frozen              = true;
                    calib_state_next.freeze_latched         = true;
                    calib_state_next.freeze_reason          = 'OUT_OF_RANGE';
                    calib_state_next.gain_hat               = [1.0; 1.0];
                    calib_state_next.gain_residual          = [delta_g_diff; delta_g_diff];
                    calib_state_next.apparent_delta_kf      = opts.Kf_nominal * (gL_hat - gR_hat);

                    calib_info.identifiability_status = 'OUT_OF_CALIBRATION_RANGE';
                    calib_info.gain_source            = 'NONE';
                    calib_info.calibration_profile    = opts.calibration_profile;
                    calib_info.reference_kind         = opts.reference_kind;
                    calib_info.did_update             = false;
                    calib_info.is_frozen              = true;
                    calib_info.reject_reason          = 'OUT_OF_RANGE';
                    calib_info.gain_residual          = calib_state_next.gain_residual;
                    calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

                    current_corrected = current_in; % 超标刚性拦截，禁止缩放
                else
                    % 正常在设计范围内，完成工程标定并锁定
                    calib_state_next.gain_hat               = [gL_hat; gR_hat];
                    calib_state_next.identifiability_status = 'CALIBRATED_EXTERNAL_REFERENCE';
                    calib_state_next.gain_source            = gain_source_name;
                    calib_state_next.is_calibrated          = true;
                    calib_state_next.is_frozen              = true;
                    calib_state_next.freeze_latched         = false;
                    calib_state_next.freeze_reason          = 'NONE';
                    calib_state_next.gain_residual          = abs(calib_state_next.gain_hat - [1.0; 1.0]);
                    calib_state_next.apparent_delta_kf      = opts.Kf_nominal * (gL_hat - gR_hat);

                    calib_info.identifiability_status = 'CALIBRATED_EXTERNAL_REFERENCE';
                    calib_info.gain_source            = gain_source_name;
                    calib_info.calibration_profile    = opts.calibration_profile;
                    calib_info.reference_kind         = opts.reference_kind;
                    calib_info.did_update             = true;
                    calib_info.is_frozen              = true;
                    calib_info.reject_reason          = 'NONE';
                    calib_info.gain_residual          = calib_state_next.gain_residual;
                    calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

                    current_corrected = current_in ./ calib_state_next.gain_hat;
                end
            else
                % 样本预热中
                calib_info.identifiability_status = 'UNIDENTIFIABLE';
                calib_info.gain_source            = 'NONE';
                calib_info.calibration_profile    = opts.calibration_profile;
                calib_info.reference_kind         = opts.reference_kind;
                calib_info.did_update             = false;
                calib_info.is_frozen              = false;
                calib_info.reject_reason          = 'LOW_EXCITATION';
                calib_state_next.freeze_latched   = false;
                calib_state_next.freeze_reason    = 'NONE';
                current_corrected                 = current_in;
            end

        %% -----------------------------------------------------------------
        %% 模式 3: RELATIVE_BALANCE_ASSUMED (对称运行假设均衡分析)
        %% -----------------------------------------------------------------
        case 'RELATIVE_BALANCE_ASSUMED'
            % 本模式仅在声明对称运行假设前提下分析相对差模
            % 绝对禁止宣称真实 Delta_Kf 已辨识，绝对禁止用于 C8A-eng 主闭环
            calib_state_next.identifiability_status = 'ASSUMPTION_ONLY';
            calib_state_next.gain_source            = 'SYMMETRIC_MOTION_ASSUMPTION';
            calib_state_next.is_calibrated          = false;
            calib_state_next.is_frozen              = true;
            calib_state_next.freeze_latched         = false;
            calib_state_next.freeze_reason          = 'NONE';
            
            % 计算表观不对称比例
            if all(abs(current_in) > opts.th_current_min)
                apparent_ratio = current_in(1) / current_in(2);
                calib_state_next.apparent_delta_kf = opts.Kf_nominal * (apparent_ratio - 1.0);
            else
                calib_state_next.apparent_delta_kf = 0.0;
            end

            calib_info.mode                   = 'RELATIVE_BALANCE_ASSUMED';
            calib_info.identifiability_status = 'ASSUMPTION_ONLY';
            calib_info.gain_source            = 'SYMMETRIC_MOTION_ASSUMPTION';
            calib_info.did_update             = false;
            calib_info.is_frozen              = true;
            calib_info.reject_reason          = 'NONE';
            calib_info.gain_residual          = [0.0; 0.0];
            calib_info.apparent_delta_kf      = calib_state_next.apparent_delta_kf;

            % 安全直通原量测，严禁输出缩放后的电流用于回归
            current_corrected = current_in;

        otherwise
            error('未知的校准模式: %s', opts.mode);
    end
end
