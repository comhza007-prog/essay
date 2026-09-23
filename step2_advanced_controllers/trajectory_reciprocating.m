%% TRAJECTORY_RECIPROCATING.M - 往复换向离散梯形速度轨迹发生器
% =========================================================================
% 特性说明:
% 生成总时长 t_span (默认 7.0s) 的正反向往复运动轨迹:
% 1. 阶段 1 (0 ~ t_half): 从 0.0m 加速、巡航、制动到达 y_target (约 2.07s)，
%    并在 y_target 处静止保持至 t_half (3.5s)，用于观察正向终点超调与恢复时间；
% 2. 阶段 2 (t_half ~ t_span): 从 y_target 反向加速、巡航、制动返回 0.0m，
%    并在原点处静止保持至 t_span (7.0s)，用于观察反向原点超调与恢复时间；
% 3. 位置与速度全局严格连续，加速度分段常值有界，阶段切换点发生有限阶跃。
% =========================================================================

function traj = trajectory_reciprocating(t_span_total, Ts, y_target, v_max, a_max)
    if nargin < 1 || isempty(t_span_total), t_span_total = 7.0; end
    if nargin < 2 || isempty(Ts), Ts = 0.001; end
    if nargin < 3 || isempty(y_target), y_target = 1.0; end
    if nargin < 4 || isempty(v_max), v_max = 0.6; end
    if nargin < 5 || isempty(a_max), a_max = 1.5; end
    
    t = 0:Ts:t_span_total;
    N_pts = length(t);
    t_half = t_span_total / 2.0;
    
    % 单程梯形时间与位移参数
    t_acc = v_max / a_max;
    y_acc = 0.5 * a_max * (t_acc^2);
    
    if 2.0 * y_acc > y_target
        % 三角形速度曲线
        t_acc = sqrt(y_target / a_max);
        v_cruise = a_max * t_acc;
        t_flat = 0.0;
        t_dec_start = t_acc;
        t_end_stroke = 2.0 * t_acc;
    else
        % 梯形速度曲线
        v_cruise = v_max;
        y_flat = y_target - 2.0 * y_acc;
        t_flat = y_flat / v_cruise;
        t_dec_start = t_acc + t_flat;
        t_end_stroke = t_dec_start + t_acc;
    end
    
    assert(t_end_stroke <= t_half, '单程运动时间必须小于半周期时长以提供静止保持观察窗口');
    
    y = zeros(1, N_pts);
    ydot = zeros(1, N_pts);
    yddot = zeros(1, N_pts);
    
    for k = 1:N_pts
        tk = t(k);
        
        if tk < t_half
            % --- 阶段 1: 正向运动与正向目标保持 ---
            if tk <= t_acc
                % 正向加速
                yddot(k) = a_max;
                ydot(k) = a_max * tk;
                y(k) = 0.5 * a_max * (tk^2);
            elseif tk <= t_dec_start
                % 正向匀速
                yddot(k) = 0.0;
                ydot(k) = v_cruise;
                y(k) = y_acc + v_cruise * (tk - t_acc);
            elseif tk <= t_end_stroke
                % 正向减速
                dt = tk - t_dec_start;
                yddot(k) = -a_max;
                ydot(k) = v_cruise - a_max * dt;
                y(k) = y_acc + y_flat + (v_cruise * dt - 0.5 * a_max * (dt^2));
            else
                % 正向到达后静止保持
                yddot(k) = 0.0;
                ydot(k) = 0.0;
                y(k) = y_target;
            end
            
        else
            % --- 阶段 2: 反向运动与原点保持 ---
            tau = tk - t_half; % 从半周期起始的相对时间
            
            if tau <= t_acc
                % 反向加速 (加速度向下)
                yddot(k) = -a_max;
                ydot(k) = -a_max * tau;
                y(k) = y_target - 0.5 * a_max * (tau^2);
            elseif tau <= t_dec_start
                % 反向匀速
                yddot(k) = 0.0;
                ydot(k) = -v_cruise;
                y(k) = y_target - y_acc - v_cruise * (tau - t_acc);
            elseif tau <= t_end_stroke
                % 反向减速 (制动加速度向上)
                dt = tau - t_dec_start;
                yddot(k) = a_max;
                ydot(k) = -v_cruise + a_max * dt;
                y(k) = y_target - y_acc - y_flat - (v_cruise * dt - 0.5 * a_max * (dt^2));
            else
                % 返回原点后静止保持
                yddot(k) = 0.0;
                ydot(k) = 0.0;
                y(k) = 0.0;
            end
        end
    end
    
    traj.t = t;
    traj.y = y;
    traj.ydot = ydot;
    traj.yddot = yddot;
    traj.t_half = t_half;
    traj.t_fwd_end = t_end_stroke;
    traj.t_rev_end = t_half + t_end_stroke;
    traj.y_target = y_target;
    traj.v_max = v_max;
    traj.a_max = a_max;
end
