%% BUILD_STEP3B_REGRESSION.M - 从传感器通道构造真实滤波回归量
% =========================================================================
% 功能说明:
% 1. 严格从左右导轨传感器通道 (yL, yR) 重构位移 yG 与偏转角 alpha，严禁使用动力学真值
% 2. 采用标准 4 阶因果巴特沃斯 SVF 滤波器 (fc = 10Hz):
%    alpha_f      = W0(alpha_raw);
%    alpha_dot_f  = W1(alpha_raw);
%    alpha_ddot_f = W2(alpha_raw);
% 3. 从实测差分速度重构摩擦项:
%    vL_meas = [0; diff(yL)] / dt;
%    vR_meas = [0; diff(yR)] / dt;
%    FfricL_raw = plant.b_nom * vL_meas + plant.fc_nom * tanh(100 * vL_meas);
%    FfricR_raw = plant.b_nom * vR_meas + plant.fc_nom * tanh(100 * vR_meas);
%    Tfric_raw  = 0.5 * mech.Le .* (FfricR_raw - FfricL_raw);
%    Tfric_f    = W0(Tfric_raw);
% 4. 构造滤波后的所需阻抗力矩与输入电流:
%    Treq_f = mech.J_alpha_nom .* alpha_ddot_f + plant.B_alpha .* alpha_dot_f ...
%           + plant.K_alpha .* alpha_f + Tfric_f;
%    sum_current_f  = W0(iL_actual + iR_actual);
%    diff_current_f = W0(iL_actual - iR_actual);
% 5. 最终生成真实滤波回归信号:
%    phi_f = 0.25 * mech.Le .* diff_current_f;
%    y_f   = -Treq_f - 0.5 * mech.Le * Kf_mean .* sum_current_f;
% =========================================================================

function reg = build_step3b_regression( ...
    yL, yR, iL_actual, iR_actual, ...
    dt, mech, plant, Kf_mean, mode)

    if nargin < 9 || isempty(mode)
        mode = 'ideal';
    end

    Le = mech.Le;

    % 1. 从左右传感器重构几何状态 (严禁读取 alpha_true)
    yG_raw    = 0.5 * (yL + yR);
    alpha_raw = (yR - yL) / Le;

    % 2. 构造 4 阶因果巴特沃斯状态变量滤波器 (SVF, 与 Step 3A 保持严格一致)
    fc = 10.0; % 截止频率 10 Hz
    wc = 2.0 * pi * fc;
    poly_den = [1.0, 2.61312592975275 * wc, 3.41421356237310 * (wc^2), ...
                2.61312592975275 * (wc^3), wc^4];
    sys_w0_d = c2d(tf(wc^4, poly_den), dt, 'tustin');
    sys_w1_d = c2d(tf([wc^4, 0.0], poly_den), dt, 'tustin');
    sys_w2_d = c2d(tf([wc^4, 0.0, 0.0], poly_den), dt, 'tustin');

    [num_w0, den_a] = tfdata(sys_w0_d, 'v');
    [num_w1, ~]     = tfdata(sys_w1_d, 'v');
    [num_w2, ~]     = tfdata(sys_w2_d, 'v');

    % 滤波角运动量
    alpha_f      = filter(num_w0, den_a, alpha_raw);
    alpha_dot_f  = filter(num_w1, den_a, alpha_raw);
    alpha_ddot_f = filter(num_w2, den_a, alpha_raw);
    yG_f         = filter(num_w0, den_a, yG_raw);

    % 3. 从测量速度重构摩擦项 (严禁使用 T_fric_true)
    vL_meas = [0; diff(yL)] / dt;
    vR_meas = [0; diff(yR)] / dt;

    FfricL_raw = plant.b_nom * vL_meas + plant.fc_nom * tanh(100.0 * vL_meas);
    FfricR_raw = plant.b_nom * vR_meas + plant.fc_nom * tanh(100.0 * vR_meas);
    Tfric_raw  = 0.5 * Le .* (FfricR_raw - FfricL_raw);
    Tfric_f    = filter(num_w0, den_a, Tfric_raw);

    % 4. 构造滤波阻抗力矩与输入电流
    % Phase 0: d = 0, delta_m = 0
    Treq_f = mech.J_alpha_nom .* alpha_ddot_f ...
           + plant.B_alpha .* alpha_dot_f ...
           + plant.K_alpha .* alpha_f ...
           + Tfric_f;

    sum_current_f  = filter(num_w0, den_a, iL_actual + iR_actual);
    diff_current_f = filter(num_w0, den_a, iL_actual - iR_actual);

    phi_f = 0.25 * Le .* diff_current_f;
    y_f   = -Treq_f - 0.5 * Le * Kf_mean .* sum_current_f;

    % 5. 打包输出
    reg = struct();
    reg.mode         = mode;
    reg.yG_raw       = yG_raw;
    reg.alpha_raw    = alpha_raw;
    reg.yG_f         = yG_f;
    reg.alpha_f      = alpha_f;
    reg.alpha_dot_f  = alpha_dot_f;
    reg.alpha_ddot_f = alpha_ddot_f;
    reg.vL_meas      = vL_meas;
    reg.vR_meas      = vR_meas;
    reg.Tfric_raw    = Tfric_raw;
    reg.Tfric_f      = Tfric_f;
    reg.Treq_f       = Treq_f;
    reg.phi_f        = phi_f;
    reg.y_f          = y_f;
end
