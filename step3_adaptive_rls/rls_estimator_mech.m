%% RLS_ESTIMATOR_MECH.M - 机械参数在线递推最小二乘 (RLS) 估计器类
% =========================================================================
% 功能：
% 1. 在线估计平动机械参数: θ_mech = [M_tot; bG; fcG]
% 2. 状态回归方程: FG_f = M_tot * yddot_f + bG * ydot_f + fcG * Sf_f
% 3. 先验物理尺度归一化: D_prior = diag([1.5, 0.6, 1.0])，避免未来数据统计泄漏
% 4. 300ms 滑动窗 Gram 矩阵 PE 门控，静止与匀速段绝对冻结 (严防协方差风积)
% 5. 紧凑凸集物理投影保护: M in [12, 21] kg, bG in [55, 85], fcG in [12, 20]
% 6. 单步速率限制器: |ΔM| <= 0.010 kg/ms (10 kg/s) + 5Hz 低通平滑输出
% =========================================================================

classdef rls_estimator_mech
    properties
        dt = 0.001;                 % 采样周期 (s)
        lambda = 0.995;             % 遗忘因子
        epsilon_PE = 1.0e-4;        % 持续激励门限
        NW = 300;                   % 滑动窗口长度 (300 ms)
        
        % 估计参数状态向量 [M_tot; bG; fcG]
        theta_hat;                  % 当前原始估计值
        theta_proj;                 % 投影后参数
        theta_rate;                 % 速率限制后参数
        theta_smooth;               % 最终平滑输出参数
        
        % 协方差矩阵 P (3x3)
        P;
        P_init_val = 1.0e4;         % 初始协方差对角值
        P_max_trace = 1.0e7;        % 协方差上限 (防风积)
        
        % 先验固定物理尺度矩阵 D_prior = diag([a_max, v_max, Sf_max])
        D_prior = diag([1.5, 0.6, 1.0]);
        D_prior_inv;
        
        % 滑动窗循环缓冲区 (3 x NW)
        phi_bar_buffer;
        buf_idx = 1;
        buf_filled = false;
        
        % 投影区间 [min, max]
        theta_min = [12.0; 55.0; 12.0];
        theta_max = [21.0; 85.0; 20.0];
        
        % 单步最大变化率 (基于 1ms 步长)
        % dM/dt <= 10 kg/s -> 0.010 kg/ms
        rate_limit = [0.010; 0.050; 0.020];
        
        % 输出低通平滑因子 (对应约 5 Hz 截止频率)
        smooth_alpha = 0.031;
        
        % 状态指示
        is_pe_active = false;
        lambda_min_curr = NaN;
        step_count = 0;
    end
    
    methods
        %% 构造函数
        function obj = rls_estimator_mech(theta_init, lambda_in, eps_pe_in, dt_in)
            if nargin >= 1 && ~isempty(theta_init)
                obj.theta_hat = theta_init(:);
            else
                % 默认以标称值初始化: M_nom=13.1kg, bG=70.0, fcG=16.0
                obj.theta_hat = [13.1; 70.0; 16.0];
            end
            if nargin >= 2 && ~isempty(lambda_in), obj.lambda = lambda_in; end
            if nargin >= 3 && ~isempty(eps_pe_in), obj.epsilon_PE = eps_pe_in; end
            if nargin >= 4 && ~isempty(dt_in), obj.dt = dt_in; end
            
            obj.D_prior_inv = inv(obj.D_prior);
            obj = obj.reset(obj.theta_hat);
        end
        
        %% 重置状态
        function obj = reset(obj, theta_init)
            if nargin >= 2 && ~isempty(theta_init)
                obj.theta_hat = theta_init(:);
            else
                obj.theta_hat = [13.1; 70.0; 16.0];
            end
            obj.theta_proj   = obj.theta_hat;
            obj.theta_rate   = obj.theta_hat;
            obj.theta_smooth = obj.theta_hat;
            
            obj.P = obj.P_init_val * eye(3);
            obj.phi_bar_buffer = zeros(3, obj.NW);
            obj.buf_idx = 1;
            obj.buf_filled = false;
            obj.is_pe_active = false;
            obj.lambda_min_curr = NaN;
            obj.step_count = 0;
        end
        
        %% 单步在线估计推演
        function [obj, theta_out, info] = step(obj, y_output, phi_regressor)
            % y_output: 标量, 滤波后推进总力 FG_f
            % phi_regressor: 3x1 向量, [yddot_f; ydot_f; Sf_f]
            
            obj.step_count = obj.step_count + 1;
            phi = phi_regressor(:);
            y = y_output;
            
            % 1. 先验尺度归一化
            phi_bar = obj.D_prior_inv * phi;
            
            % 2. 存入滑动窗口环形缓冲区
            obj.phi_bar_buffer(:, obj.buf_idx) = phi_bar;
            obj.buf_idx = obj.buf_idx + 1;
            if obj.buf_idx > obj.NW
                obj.buf_idx = 1;
                obj.buf_filled = true;
            end
            
            % 3. 滑动窗 Gram 矩阵持续激励 (PE) 判定
            if ~obj.buf_filled
                % 窗口未满 (t < 0.3s): 保持初始参数，冻结更新
                obj.is_pe_active = false;
                obj.lambda_min_curr = NaN;
            else
                % 构造严格对称 Gram 矩阵
                G_k = (obj.phi_bar_buffer * obj.phi_bar_buffer') / obj.NW;
                G_k = 0.5 * (G_k + G_k');
                eig_vals = eig(G_k);
                obj.lambda_min_curr = max(0.0, min(eig_vals));
                
                % 门控判断
                if obj.lambda_min_curr >= obj.epsilon_PE
                    obj.is_pe_active = true;
                else
                    obj.is_pe_active = false;
                end
            end
            
            % 4. RLS 核心递推
            theta_candidate = obj.theta_hat;
            if obj.is_pe_active
                % 预测误差
                y_pred = phi' * obj.theta_hat;
                err = y - y_pred;
                
                % 卡尔曼增益
                P_phi = obj.P * phi;
                denom = obj.lambda + phi' * P_phi;
                K = P_phi / denom;
                
                % 参数与协方差更新 (投影 RLS: 内部估计状态直接投影至紧凑凸集)
                theta_candidate = obj.theta_hat + K * err;
                obj.theta_hat = max(obj.theta_min, min(obj.theta_max, theta_candidate));
                obj.theta_proj = obj.theta_hat;
                
                obj.P = (obj.P - K * (phi' * obj.P)) / obj.lambda;
                
                % 协方差矩阵对称化与上限保护
                obj.P = 0.5 * (obj.P + obj.P');
                if trace(obj.P) > obj.P_max_trace
                    obj.P = obj.P * (obj.P_max_trace / trace(obj.P));
                end
            else
                % PE 门控冻结: 严禁协方差除以 lambda (彻底防风积)
                % theta_hat 保持不变
            end
            
            % 5. 单步速率限制器 (Slew-Rate Limiter)
            delta_theta = obj.theta_hat - obj.theta_rate;
            delta_clamped = max(-obj.rate_limit, min(obj.rate_limit, delta_theta));
            obj.theta_rate = obj.theta_rate + delta_clamped;
            
            % 7. 低通平滑输出 (5 Hz)
            obj.theta_smooth = (1.0 - obj.smooth_alpha) * obj.theta_smooth + ...
                               obj.smooth_alpha * obj.theta_rate;
                           
            theta_out = obj.theta_smooth;
            
            % 输出诊断结构体
            if nargout >= 3
                info.theta_raw = obj.theta_hat;
                info.theta_candidate = theta_candidate;
                info.theta_proj = obj.theta_proj;
                info.theta_rate = obj.theta_rate;
                info.theta_smooth = obj.theta_smooth;
                info.is_pe_active = obj.is_pe_active;
                info.lambda_min = obj.lambda_min_curr;
                info.trace_P = trace(obj.P);
            end
        end
    end
end
