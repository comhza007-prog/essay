%% STEP3C_APPLY_IMPERFECTIONS.M - 三层电流解耦与物理非理想扰动注入函数
% =========================================================================
% 功能说明:
% 依据 Step 3C 技术方案设计，实现工业台架真实非理想扰动的严格因果注入:
% 1. 三层电流物理拓扑:
%    i_cmd -> 限幅/执行时滞 -> i_applied -> 驱动动力学 -> Plant -> 传感器回采 -> i_meas
% 2. 执行通信时滞与因果动力学重积分:
%    - 执行延迟 d_act 作用于控制器到电机端: i_applied(k) = sat(i_cmd(k - d_act))
%    - 历史初值严格为零: i(k - d) = 0 (k <= d)
%    - 若存在非零执行延迟 (d_act > 0)，必须逐步显式调用公共单步函数:
%      common/gantry_dynamics_step_rk4.m 重新进行数值积分生成真实的因果状态响应
% 3. 测量反馈层非理想注入:
%    - 测量延迟 d_meas 作用于传感器回采端: i_applied(k - d_meas)
%    - 比例增益误差 delta_g: (1 + delta_g) * i_applied
%    - 霍尔零漂偏置 i_bias: + i_bias
%    - 测量高斯白噪声 v_i: + v_i, v_i ~ N(0, sigma_i^2)
% 4. 几何传感测量层噪声:
%    - 光栅尺/编码器量化: quant(y, quant_res)
%    - 随机测量白噪声: + v_y, v_y ~ N(0, sigma_y^2)
%
% 输入:
%   base_data : 基础开环仿真数据集 (包含 iL_cmd, iR_cmd, mech, plant, Kf_L, Kf_R, dt 等)
%   cfg       : 扰动配置结构体
% 输出:
%   pert_data : 注入扰动后的完整时域结构体 (含 i_applied, i_meas, yL_meas, yR_meas 等)
% =========================================================================

function [pert_data] = step3c_apply_imperfections(base_data, cfg)
    % 1. 配置项缺省与校验
    if nargin < 2 || isempty(cfg)
        cfg = struct();
    end

    % 电流执行层时滞 (ms, 整数拍)
    if ~isfield(cfg, 'd_act_L'),  cfg.d_act_L  = 0; end
    if ~isfield(cfg, 'd_act_R'),  cfg.d_act_R  = 0; end

    % 电流测量层时滞 (ms, 整数拍)
    if ~isfield(cfg, 'd_meas_L'), cfg.d_meas_L = 0; end
    if ~isfield(cfg, 'd_meas_R'), cfg.d_meas_R = 0; end

    % 电流传感器比例增益误差 (相对无量纲偏差，如 0.02 表示 +2%)
    if ~isfield(cfg, 'delta_g_L'), cfg.delta_g_L = 0.0; end
    if ~isfield(cfg, 'delta_g_R'), cfg.delta_g_R = 0.0; end

    % 电流传感器零漂偏置 (counts)
    if ~isfield(cfg, 'i_bias_L'), cfg.i_bias_L = 0.0; end
    if ~isfield(cfg, 'i_bias_R'), cfg.i_bias_R = 0.0; end

    % 电流测量高斯白噪声标准差 (counts)
    if ~isfield(cfg, 'sigma_i_L'), cfg.sigma_i_L = 0.0; end
    if ~isfield(cfg, 'sigma_i_R'), cfg.sigma_i_R = 0.0; end

    % 编码器位置测量高斯白噪声标准差 (m)
    if ~isfield(cfg, 'sigma_y_L'), cfg.sigma_y_L = 0.0; end
    if ~isfield(cfg, 'sigma_y_R'), cfg.sigma_y_R = 0.0; end

    % 位置量化台阶分辨率 (m, 标称 1e-6 即 1um; 0 表示连续无量化)
    if ~isfield(cfg, 'quant_res'), cfg.quant_res = 1.0e-6; end

    % 偏载物理参数 (kg, m) 与导轨摩擦非对称系数
    if ~isfield(cfg, 'delta_m'),    cfg.delta_m    = 0.0; end
    if ~isfield(cfg, 'd_load'),     cfg.d_load     = 0.0; end
    if ~isfield(cfg, 'delta_fric'), cfg.delta_fric = 0.0; end

    % 阶跃故障跳变 (针对 Test C7 凸集投影安全性强扰动)
    if ~isfield(cfg, 'step_fault_t'),     cfg.step_fault_t     = Inf; end
    if ~isfield(cfg, 'step_fault_amp_L'), cfg.step_fault_amp_L = 0.0; end
    if ~isfield(cfg, 'step_fault_amp_R'), cfg.step_fault_amp_R = 0.0; end

    % 随机数发生器种子 (确保 Monte Carlo 严格确定性复现)
    if isfield(cfg, 'seed') && ~isempty(cfg.seed)
        rng(cfg.seed);
    end

    % 提取基础信号与参数
    dt = base_data.dt;
    if isfield(base_data, 't')
        t = base_data.t;
    else
        t = (0:dt:(base_data.N - 1) * dt)';
    end
    N = length(t);
    Le = base_data.mech.Le;

    if isfield(base_data, 'Imax')
        Imax = base_data.Imax;
    else
        Imax = 16000.0; % 标称硬件限幅
    end

    mech = base_data.mech;
    plant = base_data.plant;
    Kf_L = base_data.Kf_L;
    Kf_R = base_data.Kf_R;

    if isfield(base_data, 'iL_cmd')
        iL_cmd = base_data.iL_cmd;
    else
        iL_cmd = base_data.iL_actual;
    end

    if isfield(base_data, 'iR_cmd')
        iR_cmd = base_data.iR_cmd;
    else
        iR_cmd = base_data.iR_actual;
    end

    %% 2. 层级一 -> 层级二: 生成物理施加电流 i_applied (执行时滞 + 物理限幅)
    dL_act = cfg.d_act_L;
    dR_act = cfg.d_act_R;

    % 历史初值严格为零: i(k - d) = 0 for k <= d
    iL_delayed_cmd = zeros(N, 1);
    if dL_act < N
        iL_delayed_cmd((dL_act + 1):N) = iL_cmd(1:(N - dL_act));
    end

    iR_delayed_cmd = zeros(N, 1);
    if dR_act < N
        iR_delayed_cmd((dR_act + 1):N) = iR_cmd(1:(N - dR_act));
    end

    % 物理执行器限幅
    iL_applied = max(-Imax, min(Imax, iL_delayed_cmd));
    iR_applied = max(-Imax, min(Imax, iR_delayed_cmd));

    %% 3. 执行时滞与偏载耦合下的动力学响应生成 (因果重积分 vs 复用基准轨迹)
    has_resim = (dL_act > 0 || dR_act > 0 || cfg.delta_m ~= 0 || cfg.d_load ~= 0 || cfg.delta_fric ~= 0);

    if has_resim
        % 严格要求: 显式逐步调用公共单步函数 common/gantry_dynamics_step_rk4.m 重新积分
        yG_hist         = zeros(N, 1);
        alpha_hist      = zeros(N, 1);
        yG_dot_hist     = zeros(N, 1);
        alpha_dot_hist  = zeros(N, 1);
        alpha_ddot_hist = zeros(N, 1);
        T_fric_hist     = zeros(N, 1);

        x = zeros(4, 1); % [yG, alpha, yG_dot, alpha_dot]
        for k = 1:N
            [x_next, details] = gantry_dynamics_step_rk4(...
                x, iL_applied(k), iR_applied(k), mech, plant, ...
                cfg.delta_m, cfg.d_load, cfg.delta_fric, dt, Kf_L, Kf_R);

            yG_hist(k)         = x(1);
            alpha_hist(k)      = x(2);
            yG_dot_hist(k)     = x(3);
            alpha_dot_hist(k)  = x(4);
            alpha_ddot_hist(k) = details.alpha_ddot;
            T_fric_hist(k)     = details.T_fric_alpha;

            x = x_next;
        end

        yL_true = yG_hist - 0.5 * Le * alpha_hist;
        yR_true = yG_hist + 0.5 * Le * alpha_hist;
        vL_true = yG_dot_hist - 0.5 * Le * alpha_dot_hist;
        vR_true = yG_dot_hist + 0.5 * Le * alpha_dot_hist;
        yG_true = yG_hist;
        alpha_true = alpha_hist;
    else
        % 无执行时滞: 严格复用基准动力学轨迹
        yL_true = base_data.yL;
        yR_true = base_data.yR;
        vL_true = base_data.vL;
        vR_true = base_data.vR;
        yG_true = base_data.yG;
        alpha_true = base_data.alpha;
        if isfield(base_data, 'alpha_ddot_hist')
            alpha_ddot_hist = base_data.alpha_ddot_hist;
        elseif isfield(base_data, 'alpha_ddot')
            alpha_ddot_hist = base_data.alpha_ddot;
        else
            alpha_ddot_hist = zeros(N, 1);
        end
        if isfield(base_data, 'T_fric_hist')
            T_fric_hist = base_data.T_fric_hist;
        elseif isfield(base_data, 'T_fric')
            T_fric_hist = base_data.T_fric;
        else
            T_fric_hist = zeros(N, 1);
        end
    end

    %% 4. 层级二 -> 层级三: 生成测量电流 i_meas (回采时滞 + 增益漂移 + 偏置 + 白噪声)
    dL_meas = cfg.d_meas_L;
    dR_meas = cfg.d_meas_R;

    iL_meas_delayed = zeros(N, 1);
    if dL_meas < N
        iL_meas_delayed((dL_meas + 1):N) = iL_applied(1:(N - dL_meas));
    end

    iR_meas_delayed = zeros(N, 1);
    if dR_meas < N
        iR_meas_delayed((dR_meas + 1):N) = iR_applied(1:(N - dR_meas));
    end

    % 高斯白噪声生成 (支持预置白噪声序列以确保严格配对消融)
    if isfield(cfg, 'z_iL') && ~isempty(cfg.z_iL)
        z_iL = cfg.z_iL(:);
        assert(numel(z_iL) == N && all(isfinite(z_iL)), ...
            'cfg.z_iL必须包含N个有限样本');
        v_iL = cfg.sigma_i_L * z_iL;
    elseif cfg.sigma_i_L > 0
        v_iL = cfg.sigma_i_L * randn(N, 1);
    else
        v_iL = zeros(N, 1);
    end

    if isfield(cfg, 'z_iR') && ~isempty(cfg.z_iR)
        z_iR = cfg.z_iR(:);
        assert(numel(z_iR) == N && all(isfinite(z_iR)), ...
            'cfg.z_iR必须包含N个有限样本');
        v_iR = cfg.sigma_i_R * z_iR;
    elseif cfg.sigma_i_R > 0
        v_iR = cfg.sigma_i_R * randn(N, 1);
    else
        v_iR = zeros(N, 1);
    end

    % 测量电流合成
    iL_meas = (1.0 + cfg.delta_g_L) * iL_meas_delayed + cfg.i_bias_L + v_iL;
    iR_meas = (1.0 + cfg.delta_g_R) * iR_meas_delayed + cfg.i_bias_R + v_iR;

    % 阶跃故障跳变 (针对 Test C7 凸集投影安全性强扰动，作用于回采测量电流物理层)
    % 明确命名为: Measured_Current_Sensor_Step_Fault (回采电流传感器阶跃测量故障)
    if isfinite(cfg.step_fault_t)
        fault_mask = (t >= cfg.step_fault_t);
        iL_meas(fault_mask) = iL_meas(fault_mask) + cfg.step_fault_amp_L;
        iR_meas(fault_mask) = iR_meas(fault_mask) + cfg.step_fault_amp_R;
    end

    %% 5. 传感器位置测量层生成 (量化台阶 + 高斯随机噪声)
    % 5.1 量化处理
    if cfg.quant_res > 0
        yL_q = round(yL_true / cfg.quant_res) * cfg.quant_res;
        yR_q = round(yR_true / cfg.quant_res) * cfg.quant_res;
    else
        yL_q = yL_true;
        yR_q = yR_true;
    end

    % 5.2 位置高斯测量噪声 (支持预置白噪声序列以确保严格配对消融)
    if isfield(cfg, 'z_yL') && ~isempty(cfg.z_yL)
        z_yL = cfg.z_yL(:);
        assert(numel(z_yL) == N && all(isfinite(z_yL)), ...
            'cfg.z_yL必须包含N个有限样本');
        v_yL = cfg.sigma_y_L * z_yL;
    elseif cfg.sigma_y_L > 0
        v_yL = cfg.sigma_y_L * randn(N, 1);
    else
        v_yL = zeros(N, 1);
    end

    if isfield(cfg, 'z_yR') && ~isempty(cfg.z_yR)
        z_yR = cfg.z_yR(:);
        assert(numel(z_yR) == N && all(isfinite(z_yR)), ...
            'cfg.z_yR必须包含N个有限样本');
        v_yR = cfg.sigma_y_R * z_yR;
    elseif cfg.sigma_y_R > 0
        v_yR = cfg.sigma_y_R * randn(N, 1);
    else
        v_yR = zeros(N, 1);
    end

    yL_meas = yL_q + v_yL;
    yR_meas = yR_q + v_yR;

    %% 6. 打包输出
    pert_data = struct();
    pert_data.cfg             = cfg;
    pert_data.t               = t;
    pert_data.dt              = dt;
    pert_data.N               = N;
    pert_data.Imax            = Imax;
    pert_data.mech            = mech;
    pert_data.plant           = plant;
    pert_data.Kf_L            = Kf_L;
    pert_data.Kf_R            = Kf_R;
    pert_data.Kf_mean         = base_data.Kf_mean;
    pert_data.Delta_Kf_true   = base_data.Delta_Kf_true;
    
    % 三层电流链
    pert_data.iL_cmd          = iL_cmd;
    pert_data.iR_cmd          = iR_cmd;
    pert_data.iL_applied      = iL_applied;
    pert_data.iR_applied      = iR_applied;
    pert_data.iL_meas         = iL_meas;
    pert_data.iR_meas         = iR_meas;

    % 真实与测量状态
    pert_data.yL_true         = yL_true;
    pert_data.yR_true         = yR_true;
    pert_data.vL_true         = vL_true;
    pert_data.vR_true         = vR_true;
    pert_data.yG_true         = yG_true;
    pert_data.alpha_true      = alpha_true;
    pert_data.alpha_ddot_hist = alpha_ddot_hist;
    pert_data.T_fric_hist     = T_fric_hist;
    pert_data.delta_m         = cfg.delta_m;
    pert_data.d_load          = cfg.d_load;
    pert_data.delta_fric      = cfg.delta_fric;

    pert_data.yL_q            = yL_q;
    pert_data.yR_q            = yR_q;
    pert_data.v_yL            = v_yL;
    pert_data.v_yR            = v_yR;
    pert_data.yL_meas         = yL_meas;
    pert_data.yR_meas         = yR_meas;

    % 时滞标志与统计起始时间
    d_max_act = max(dL_act, dR_act);
    d_max_meas = max(dL_meas, dR_meas);
    pert_data.d_max_act       = d_max_act;
    pert_data.d_max_meas      = d_max_meas;
    pert_data.t_eval_start    = 0.5 + max(d_max_act, d_max_meas) * dt;
    pert_data.t_eval_end      = 2.3;
end
