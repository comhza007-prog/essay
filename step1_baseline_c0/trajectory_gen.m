%% TRAJECTORY_GEN.M - 离散梯形速度轨迹 (T型速度规划) 发生器
% =========================================================================
% 特性说明:
% 生成具有连续位置与连续速度的梯形速度规划轨迹；
% 加速度在加速、匀速与减速切换处存在阶段性阶跃（非S曲线高阶平滑轨迹）。
% =========================================================================

function traj = trajectory_gen(t_span, Ts, y_target, v_max, a_max)
    % t_span: 仿真总时长 (s)
    % Ts: 离散采样步长 (s)
    % y_target: 目标平动位移 (m)
    % v_max: 最大巡航速度 (m/s)
    % a_max: 加减速加速度大小 (m/s^2)
    
    t = 0:Ts:t_span;
    N_pts = length(t);
    
    % 加速段所需时间与位移
    t_acc = v_max / a_max;
    y_acc = 0.5 * a_max * (t_acc^2);
    
    if 2.0 * y_acc > y_target
        % 三角形速度轨迹 (未达最大巡航速度即减速)
        t_acc = sqrt(y_target / a_max);
        v_cruise = a_max * t_acc;
        t_flat = 0.0;
        t_dec_start = t_acc;
        t_end = 2.0 * t_acc;
    else
        % 梯形速度轨迹 (包含匀速巡航段)
        v_cruise = v_max;
        y_flat = y_target - 2.0 * y_acc;
        t_flat = y_flat / v_cruise;
        t_dec_start = t_acc + t_flat;
        t_end = t_dec_start + t_acc;
    end
    
    y = zeros(1, N_pts);
    ydot = zeros(1, N_pts);
    yddot = zeros(1, N_pts);
    
    for k = 1:N_pts
        tk = t(k);
        if tk <= t_acc
            % 加速阶段 (a = +a_max)
            yddot(k) = a_max;
            ydot(k) = a_max * tk;
            y(k) = 0.5 * a_max * (tk^2);
        elseif tk <= t_dec_start
            % 匀速巡航阶段 (a = 0)
            yddot(k) = 0.0;
            ydot(k) = v_cruise;
            y(k) = 0.5 * a_max * (t_acc^2) + v_cruise * (tk - t_acc);
        elseif tk <= t_end
            % 减速阶段 (a = -a_max)
            dt = tk - t_dec_start;
            yddot(k) = -a_max;
            ydot(k) = v_cruise - a_max * dt;
            y(k) = 0.5 * a_max * (t_acc^2) + v_cruise * t_flat + (v_cruise * dt - 0.5 * a_max * (dt^2));
        else
            % 到位保持阶段 (a = 0, v = 0)
            yddot(k) = 0.0;
            ydot(k) = 0.0;
            y(k) = y_target;
        end
    end
    
    traj.t = t;
    traj.y = y;
    traj.ydot = ydot;
    traj.yddot = yddot;
    traj.t_end = t_end;
end
