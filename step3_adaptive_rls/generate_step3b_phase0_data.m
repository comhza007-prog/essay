%% GENERATE_STEP3B_PHASE0_DATA.M - Step 3B Phase 0 理想连续状态开环基准数据生成器
% =========================================================================
% 功能说明:
% 1. 生成执行器推力增益非对称 (Delta_Kf = Kf_L - Kf_R) 专用开环基准时域数据集
% 2. 严格锁定 Phase 0 物理边界:
%    - d_load = 0 m, delta_m = 0 kg (彻底消除平动-偏转惯性耦合项 delta_m*d*yG_ddot)
%    - delta_fric = 0 (左右导轨对称摩擦: bL=bR=35 N*s/m, fcL=fcR=8 N 作为仿真真值)
% 3. 输入端先经过执行器饱和限幅 [-Imax, Imax]，保存实际作用电流 iL_actual, iR_actual
% 4. 显式记录并导出左右传感器通道与动力学全状态:
%    yL, yR, vL, vR, yG, alpha, alpha_dot, alpha_ddot, iL_actual, iR_actual,
%    phi_Delta_T, y_Delta_T, Delta_Kf_true
% 5. 分别生成工况 A (r = 0.70) 与工况 B (r = 1.30) 并导出为独立 MAT 文件
% =========================================================================

function generate_step3b_phase0_data()
    % 环境与路径配置
    script_dir = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, '..');
    
    addpath(fullfile(output_dir, 'step1_baseline_c0'));
    addpath(fullfile(output_dir, 'step2_advanced_controllers'));
    addpath(fullfile(output_dir, 'step3_adaptive_rls'));
    
    [ctrl, mech, plant] = param_init();
    
    dt = 0.001;               % 采样周期 1ms
    T_total = 4.0;            % 总时长 4s
    t = (0:dt:T_total)';
    N = length(t);
    Le = mech.Le;
    Kf_mean = mech.Kf;
    Imax = ctrl.spd_max_out;  % 标称 16000 counts
    
    % 设计包含激活激励段 (0.2~2.5s) 与停顿静止段 (2.7~4.0s) 的严格测试电流包络
    env = zeros(size(t));
    for k = 1:N
        tk = t(k);
        if tk < 0.2
            env(k) = tk / 0.2;
        elseif tk <= 2.5
            env(k) = 1.0;
        elseif tk <= 2.7
            env(k) = 1.0 - (tk - 2.5) / 0.2;
        else
            env(k) = 0.0;
        end
    end
    
    i_comm = (2000.0 * cos(2.0*pi*1.0*t) - 1000.0 * cos(2.0*pi*2.0*t)) .* env;
    i_diff = (120.0 * sin(2.0*pi*1.5*t)) .* env;
    
    % 指令电流与执行器饱和截断
    iL_cmd = i_comm + i_diff;
    iR_cmd = -i_comm + i_diff;
    
    iL_actual = max(-Imax, min(Imax, iL_cmd));
    iR_actual = max(-Imax, min(Imax, iR_cmd));
    
    % 测试工况列表: r = 0.70 (工况 A) 与 r = 1.30 (工况 B)
    r_list = [0.70, 1.30];
    case_names = {'r070', 'r130'};
    
    for c = 1:length(r_list)
        r_val = r_list(c);
        case_tag = case_names{c};
        
        % 保持平均总推力恒定: (Kf_L + Kf_R)/2 = Kf_mean
        Kf_L = 2.0 * Kf_mean * r_val / (1.0 + r_val);
        Kf_R = 2.0 * Kf_mean / (1.0 + r_val);
        Delta_Kf_true = Kf_L - Kf_R;
        
        fprintf('====================================================\n');
        fprintf('>>> 生成 Step 3B Phase 0 开环数据集: 工况 %s (r = %.2f)\n', case_tag, r_val);
        fprintf('    Kf_L = %.7f, Kf_R = %.7f N/count\n', Kf_L, Kf_R);
        fprintf('    Delta_Kf_true = %.7f N/count\n', Delta_Kf_true);
        
        % 状态存储预分配
        yG_hist         = zeros(N, 1);
        alpha_hist      = zeros(N, 1);
        yG_dot_hist     = zeros(N, 1);
        alpha_dot_hist  = zeros(N, 1);
        alpha_ddot_hist = zeros(N, 1);
        T_fric_hist     = zeros(N, 1);
        
        x = zeros(4, 1); % [yG, alpha, yG_dot, alpha_dot]
        
        % 动力学单步推演
        for k = 1:N
            [x_next, details] = gantry_dynamics_step_step3b(...
                x, iL_actual(k), iR_actual(k), mech, plant, ...
                0.0, 0.0, 0.0, dt, Kf_L, Kf_R);
            
            yG_hist(k)         = x(1);
            alpha_hist(k)      = x(2);
            yG_dot_hist(k)     = x(3);
            alpha_dot_hist(k)  = x(4);
            alpha_ddot_hist(k) = details.alpha_ddot;
            T_fric_hist(k)     = details.T_fric_alpha;
            
            x = x_next;
        end
        
        % 显式重构左右物理传感器理想连续通道
        yL_ideal = yG_hist - 0.5 * Le * alpha_hist;
        yR_ideal = yG_hist + 0.5 * Le * alpha_hist;
        vL_ideal = yG_dot_hist - 0.5 * Le * alpha_dot_hist;
        vR_ideal = yG_dot_hist + 0.5 * Le * alpha_dot_hist;
        
        % Phase 0: 引入编码器量化测量 (分辨率 q_y = 1.21e-6 m)，暂不加入随机测量噪声
        q_y = 1.21e-6;     % 线位移量化分辨率 (m)
        rng(20260923);     % 固定随机种子保证可复现
        yL_quant = q_y * round(yL_ideal / q_y);
        yR_quant = q_y * round(yR_ideal / q_y);
        
        % 动力学真实值 (仅作为误差对照基准，严禁作为传感器辨识输入)
        alpha_true      = alpha_hist;
        alpha_dot_true  = alpha_dot_hist;
        alpha_ddot_true = alpha_ddot_hist;
        T_fric_true     = T_fric_hist;
        
        % 几何一致性断言
        assert(max(abs(yG_hist - 0.5 * (yL_ideal + yR_ideal))) < 1e-12, 'yG 左右合成一致性检验失败');
        assert(max(abs(alpha_true - (yR_ideal - yL_ideal) / Le)) < 1e-12, 'alpha 左右合成一致性检验失败');
        
        % 构造严格回归基底与可测输出 (Phase 0: d = 0, delta_m = 0)
        phi_Delta_T = 0.25 * Le * (iL_actual - iR_actual); % [count*m]
        T_req = mech.J_alpha_nom * alpha_ddot_true + plant.B_alpha * alpha_dot_true ...
                + plant.K_alpha * alpha_true + T_fric_true;
        y_Delta_T = -T_req - 0.5 * Le * Kf_mean * (iL_actual + iR_actual); % [N*m]
        
        % 独立代数残差检验
        ideal_res = y_Delta_T - phi_Delta_T * Delta_Kf_true;
        max_alg_err = max(abs(ideal_res));
        fprintf('    最大瞬时理论代数残差: %.2e N*m\n', max_alg_err);
        assert(max_alg_err < 1e-12, '理论代数自洽性残差超限！');
        
        % 结构体打包保存
        data_step3b = struct();
        data_step3b.t             = t;
        data_step3b.dt            = dt;
        data_step3b.r_val         = r_val;
        data_step3b.Delta_Kf_true = Delta_Kf_true;
        data_step3b.Kf_L          = Kf_L;
        data_step3b.Kf_R          = Kf_R;
        data_step3b.Kf_mean       = Kf_mean;
        data_step3b.Le            = Le;
        data_step3b.mech          = mech;
        data_step3b.plant         = plant;
        
        % 理想与量化传感器通道
        data_step3b.yL_ideal = yL_ideal;
        data_step3b.yR_ideal = yR_ideal;
        data_step3b.yL_quant = yL_quant;
        data_step3b.yR_quant = yR_quant;
        data_step3b.q_y      = q_y;
        
        % 动力学真实值 (仅用于对照，严禁直接接入回归)
        data_step3b.alpha_true      = alpha_true;
        data_step3b.alpha_dot_true  = alpha_dot_true;
        data_step3b.alpha_ddot_true = alpha_ddot_true;
        data_step3b.T_fric_true     = T_fric_true;
        
        % 向下兼容通用字段
        data_step3b.yL         = yL_ideal;
        data_step3b.yR         = yR_ideal;
        data_step3b.vL         = vL_ideal;
        data_step3b.vR         = vR_ideal;
        data_step3b.yG         = yG_hist;
        data_step3b.alpha      = alpha_true;
        data_step3b.alpha_dot  = alpha_dot_true;
        data_step3b.alpha_ddot = alpha_ddot_true;
        data_step3b.T_fric     = T_fric_true;
        
        % 实际执行电流与理论回归基底
        data_step3b.iL_actual   = iL_actual;
        data_step3b.iR_actual   = iR_actual;
        data_step3b.phi_Delta_T = phi_Delta_T;
        data_step3b.y_Delta_T   = y_Delta_T;
        
        save_file = fullfile(script_dir, sprintf('data_step3b_phase0_%s.mat', case_tag));
        save(save_file, '-struct', 'data_step3b');
        fprintf('    成功保存: %s\n', save_file);
    end
    fprintf('====================================================\n');
    fprintf('>>> Step 3B Phase 0 理想连续数据集生成全部完成！\n\n');
end
