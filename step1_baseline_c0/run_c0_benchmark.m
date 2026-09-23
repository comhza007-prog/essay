%% RUN_C0_BENCHMARK.M - 双M3508龙门系统 C0 基线模型严格复现主脚本 (V2.0)
% =========================================================================
% 本脚本完成第一阶段的严格规范复现：
% 1. 严格复现比赛代码符号逻辑:
%    - 左电机: 正向旋转为向前 (ecd_L > 0, rpm_L > 0, 目标 ecd_target > 0)
%    - 右电机: 对向安装，反向旋转为向前 (ecd_R < 0, rpm_R < 0, 目标 -ecd_target < 0)
% 2. 严格实现 chassis_calculate() 中的动态梯度位置环限幅:
%    dynamic_max_out = min(error_rev, travel_rev) * CHASSIS_ANG_GRADIENT_SLOPE;
%    限幅范围 [1000, 12000] rpm
% 3. M3508 减速比采用代码实际值 N_code = 19.0
% 4. 规范将附加质量表述为“右侧偏心附加质量”，摩擦为左右非对称
% 5. 严谨判定调节时间，记录未稳定 (Unsettled) 状态与饱和时间占比
% =========================================================================

if ~exist('is_test_runner', 'var')
    clear; clc; close all;
end

%% 1. 初始化系统与机械参数
[ctrl, mech, plant] = param_init();

% 仿真时间与离散步长
Ts = ctrl.Ts;                     % 0.001 s (1 ms)
t_span = 3.5;                     % 仿真总时长 3.5 s

% 期望目标轨迹 (1.0米行程, 梯形速度规划)
y_target = 1.0;                   % 目标位移 1.0 m
v_max = 0.6;                      % 最大巡航速度 0.6 m/s
a_max = 1.5;                      % 加减速加速度 1.5 m/s^2
traj = trajectory_gen(t_span, Ts, y_target, v_max, a_max);
N_steps = length(traj.t);

% 传动几何换算比例 (采用代码实际减速比 N = 19.0)
% 齿条线位移 (m) -> 电机转角 (rev) -> 编码器计数 (counts)
m_to_ecd = (mech.N * ctrl.ecd_cpr) / (2.0 * pi * mech.rp);
% 齿条线速度 (m/s) -> 电机转速 (rpm)
ms_to_rpm = (60.0 * mech.N) / (2.0 * pi * mech.rp);

%% 2. 定义四组测试工况
cases(1).name = 'Case 1: 标称对称工况(无偏心质量)';
cases(1).delta_m = 0.0;
cases(1).d_load = 0.0;
cases(1).delta_fric = 0.0;
cases(1).Imax = 16000.0;
cases(1).color = [0.0, 0.45, 0.74]; % 蓝色

cases(2).name = 'Case 2: 右侧偏心质量1.5kg+摩擦差15%';
cases(2).delta_m = 1.5;
cases(2).d_load = 0.25;
cases(2).delta_fric = 0.15;
cases(2).Imax = 16000.0;
cases(2).color = [0.85, 0.33, 0.10]; % 橙色

cases(3).name = 'Case 3: 右侧偏心质量3.0kg+摩擦差30%';
cases(3).delta_m = 3.0;
cases(3).d_load = 0.28;
cases(3).delta_fric = 0.30;
cases(3).Imax = 16000.0;
cases(3).color = [0.93, 0.69, 0.13]; % 黄色

cases(4).name = 'Case 4: 右侧偏心质量3.0kg+限流(4500)';
cases(4).delta_m = 3.0;
cases(4).d_load = 0.28;
cases(4).delta_fric = 0.30;
cases(4).Imax = 4500.0;             % 严格限制可用电流指令
cases(4).color = [0.64, 0.08, 0.18]; % 深红

%% 3. 循环运行仿真
results = struct();

for c_idx = 1:length(cases)
    cfg = cases(c_idx);
    fprintf('正在仿真 %s ...\n', cfg.name);
    
    % 初始化被控状态: x = [yG; alpha; yG_dot; alpha_dot]
    x = zeros(4, 1);
    
    % 初始化控制器状态结构体
    pid_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, ctrl.ang_base_max_out, ctrl.ang_max_iout);
    pid_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, ctrl.ang_base_max_out, ctrl.ang_max_iout);
    pid_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, cfg.Imax, ctrl.spd_max_iout);
    pid_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, cfg.Imax, ctrl.spd_max_iout);
    
    % 起始编码器参考点 (对应 chassis_calculate() 中的 erro_int)
    ecd_start_L = 0.0;
    
    % 数据记录数组
    log_yG = zeros(1, N_steps);
    log_alpha = zeros(1, N_steps);
    log_yL = zeros(1, N_steps);
    log_yR = zeros(1, N_steps);
    log_iL = zeros(1, N_steps);
    log_iR = zeros(1, N_steps);
    log_dyn_max_out = zeros(1, N_steps);
    log_sat = zeros(1, N_steps);
    
    for k = 1:N_steps
        % 1. 读取当前物理状态 (以向前为正)
        yG_curr = x(1);
        alpha_curr = x(2);
        yG_dot_curr = x(3);
        alpha_dot_curr = x(4);
        
        % 左右导轨几何位移与线速度 (物理量: 向前为正)
        yL_curr = yG_curr - 0.5 * mech.Le * alpha_curr;
        yR_curr = yG_curr + 0.5 * mech.Le * alpha_curr;
        vL_curr = yG_dot_curr - 0.5 * mech.Le * alpha_dot_curr;
        vR_curr = yG_dot_curr + 0.5 * mech.Le * alpha_dot_curr;
        
        % 2. 映射到电机编码器与反馈转速 (严格复现比赛代码对向安装符号)
        ecd_L =  yL_curr * m_to_ecd;
        ecd_R = -yR_curr * m_to_ecd; % 右侧电机符号相反
        rpm_L =  vL_curr * ms_to_rpm;
        rpm_R = -vR_curr * ms_to_rpm; % 右侧转速符号相反
        
        % 3. 设定目标值生成 (与 chassis_calculate 严格一致)
        yd_target = traj.y(k);
        ecd_target_L =  yd_target * m_to_ecd;
        ecd_target_R = -yd_target * m_to_ecd; % 右侧目标编码器符号相反
        
        % 4. 动态位置环梯度限幅 (严格复刻 chassis_calculate)
        if ctrl.use_dynamic_ang_limit
            % 剩余误差(圈) 与 已走距离(圈)
            error_rev = abs(ecd_target_L - ecd_L) / ctrl.ecd_cpr;
            travel_rev = abs(ecd_L - ecd_start_L) / ctrl.ecd_cpr;
            dynamic_max_out = min(error_rev, travel_rev) * ctrl.ang_gradient_slope;
            
            % 梯度限幅截断
            if dynamic_max_out > ctrl.ang_gradient_max_out
                dynamic_max_out = ctrl.ang_gradient_max_out;
            elseif dynamic_max_out < ctrl.ang_gradient_min_out
                dynamic_max_out = ctrl.ang_gradient_min_out;
            end
            
            pid_ang_L.max_out = dynamic_max_out;
            pid_ang_R.max_out = dynamic_max_out;
            log_dyn_max_out(k) = dynamic_max_out;
        end
        
        % 5. 级联 PID 计算 (严格对应 motor.c)
        % 位置环计算 -> 输出目标转速 rpm
        [spd_cmd_L, pid_ang_L] = pid_calc(pid_ang_L, ecd_L, ecd_target_L);
        [spd_cmd_R, pid_ang_R] = pid_calc(pid_ang_R, ecd_R, ecd_target_R);
        
        % 速度环计算 -> 输出 CAN 控制电流指令 (raw counts)
        [iL_raw, pid_spd_L] = pid_calc(pid_spd_L, rpm_L, spd_cmd_L);
        [iR_raw, pid_spd_R] = pid_calc(pid_spd_R, rpm_R, spd_cmd_R);
        
        % 6. 执行器限幅 (iR 对应负值区限幅 [-Imax, Imax])
        iL_cmd = max(-cfg.Imax, min(cfg.Imax, iL_raw));
        iR_cmd = max(-cfg.Imax, min(cfg.Imax, iR_raw));
        
        is_sat_L = (abs(iL_raw) >= cfg.Imax);
        is_sat_R = (abs(iR_raw) >= cfg.Imax);
        log_sat(k) = (is_sat_L || is_sat_R);
        
        % 7. 动力学推演 (RK4, 内置右电机负向推力关系: FR = -Kf * iR_cmd)
        x = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, ...
                                cfg.delta_m, cfg.d_load, cfg.delta_fric, Ts);
                            
        % 8. 记录物理与控制变量
        log_yG(k) = yG_curr;
        log_alpha(k) = alpha_curr;
        log_yL(k) = yL_curr;
        log_yR(k) = yR_curr;
        log_iL(k) = iL_cmd;
        log_iR(k) = iR_cmd;
    end
    
    % 保存结果结构体
    results(c_idx).t = traj.t;
    results(c_idx).yG = log_yG;
    results(c_idx).alpha = log_alpha;
    results(c_idx).esync = (log_yR - log_yL) * 1e3;    % 左右同步误差 (mm)
    results(c_idx).eyG = (log_yG - traj.y) * 1e3;       % 质心跟踪误差 (mm)
    results(c_idx).iL = log_iL;
    results(c_idx).iR = log_iR;
    results(c_idx).dyn_max_out = log_dyn_max_out;
    results(c_idx).sat_ratio = mean(log_sat) * 100.0;   % 饱和时间占比 (%)
    
    % 性能指标统计
    results(c_idx).rmse_yG = sqrt(mean((results(c_idx).eyG).^2));
    results(c_idx).rmse_sync = sqrt(mean((results(c_idx).esync).^2));
    results(c_idx).max_sync = max(abs(results(c_idx).esync));
    results(c_idx).max_alpha = max(abs(log_alpha)) * 1e3; % mrad
    
    % 严谨计算调节时间 Ts (进入最终位移 ±1mm 容差带)
    settle_tol = 1.0; % 1 mm
    err_tail = abs(results(c_idx).yG - y_target) * 1e3;
    if err_tail(end) > settle_tol
        results(c_idx).is_settled = false;
        results(c_idx).settle_time = NaN;
    else
        idx_out = find(err_tail > settle_tol, 1, 'last');
        if isempty(idx_out)
            results(c_idx).settle_time = 0.0;
        else
            results(c_idx).settle_time = traj.t(min(idx_out + 1, N_steps));
        end
        results(c_idx).is_settled = true;
    end
end

%% 4. 控制台打印格式化学术评价指标表
fprintf('\n===========================================================================================================\n');
fprintf('                双 M3508 龙门平台基线 (Baseline C0) 仿真性能评价指标表 (V2.0 严格复现版)\n');
fprintf('===========================================================================================================\n');
fprintf('%-32s | %-10s | %-12s | %-10s | %-12s | %-9s | %-10s\n', ...
    '实验工况 (Scenario)', 'RMSE_yG(mm)', 'Max_Sync(mm)', 'RMSE_Sync', 'Max_Alpha(mr)', 'Sat_Ratio', 'Settle_Ts');
fprintf('-----------------------------------------------------------------------------------------------------------\n');
for c_idx = 1:length(cases)
    r = results(c_idx);
    if r.is_settled
        ts_str = sprintf('%6.3f s', r.settle_time);
    else
        ts_str = 'Unsettled';
    end
    fprintf('%-30s | %10.3f | %12.3f | %10.3f | %12.3f | %8.1f%% | %10s\n', ...
        cases(c_idx).name, r.rmse_yG, r.max_sync, r.rmse_sync, r.max_alpha, r.sat_ratio, ts_str);
end
fprintf('===========================================================================================================\n\n');

%% 5. 绘制高质量论文插图 (四合一图)
fig = figure('Color', 'w', 'Position', [100, 100, 1100, 780]);

% 子图 1: 质心位移跟踪曲线
subplot(2, 2, 1); hold on; grid on; box on;
plot(traj.t, traj.y, 'k--', 'LineWidth', 1.5, 'DisplayName', '期望轨迹 y_d');
for c_idx = 1:length(cases)
    plot(results(c_idx).t, results(c_idx).yG, 'Color', cases(c_idx).color, ...
         'LineWidth', 1.3, 'DisplayName', cases(c_idx).name);
end
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('质心位移 y_G (m)', 'FontSize', 10);
title('(a) 质心平动轨迹跟踪', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'Southeast', 'FontSize', 8);

% 子图 2: 左右同步位置差 (yR - yL)
subplot(2, 2, 2); hold on; grid on; box on;
for c_idx = 1:length(cases)
    plot(results(c_idx).t, results(c_idx).esync, 'Color', cases(c_idx).color, ...
         'LineWidth', 1.3, 'DisplayName', cases(c_idx).name);
end
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('同步位置差 y_R - y_L (mm)', 'FontSize', 10);
title('(b) 左右两侧同步位置差', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'Northeast', 'FontSize', 8);

% 子图 3: 横梁偏转偏角 alpha
subplot(2, 2, 3); hold on; grid on; box on;
for c_idx = 1:length(cases)
    plot(results(c_idx).t, results(c_idx).alpha * 1e3, 'Color', cases(c_idx).color, ...
         'LineWidth', 1.3, 'DisplayName', cases(c_idx).name);
end
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('横梁偏转角 \alpha (mrad)', 'FontSize', 10);
title('(c) 横梁倾斜偏角动态响应', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'Northeast', 'FontSize', 8);

% 子图 4: 右侧电机 CAN 控制指令响应 (展示真实负向指令与对向限幅)
subplot(2, 2, 4); hold on; grid on; box on;
yline(-16000, 'k:', 'LineWidth', 1.2, 'DisplayName', '标称限幅 -16000');
yline(-4500, 'r:', 'LineWidth', 1.2, 'DisplayName', '紧缩限幅 -4500');
for c_idx = 1:length(cases)
    plot(results(c_idx).t, results(c_idx).iR, 'Color', cases(c_idx).color, ...
         'LineWidth', 1.3, 'DisplayName', cases(c_idx).name);
end
xlabel('时间 t (s)', 'FontSize', 10);
ylabel('右电机 CAN 指令 i_R (counts)', 'FontSize', 10);
title('(d) 右电机控制指令响应 (对向安装)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'Southeast', 'FontSize', 8);

% 保存高质量矢量/位图
saveas(fig, 'baseline_c0_comparison.png');
fprintf('仿真完成！严格复现图表已自动保存为 baseline_c0_comparison.png\n');

%% 辅助函数: 初始化 PID 结构体
function s = init_pid_struct(kp, ki, kd, max_out, max_iout)
    s.Kp = kp;
    s.Ki = ki;
    s.Kd = kd;
    s.max_out = max_out;
    s.max_iout = max_iout;
    s.error = [0.0, 0.0, 0.0];
    s.Dbuf = [0.0, 0.0, 0.0];
    s.Iout = 0.0;
    s.out = 0.0;
    s.set = 0.0;
    s.fdb = 0.0;
end
