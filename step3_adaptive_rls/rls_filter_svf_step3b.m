%% RLS_FILTER_SVF_STEP3B.M - Step 3B 专用 4 阶因果巴特沃斯状态变量滤波器 (SVF) 类
% =========================================================================
% 功能说明:
% 1. 采用 4 阶稳定巴特沃斯多项式与 Tustin 双线性变换构造因果状态变量滤波器 (fc = 10Hz)
% 2. 内部全面采用 Direct Form II Transposed (规范二型转置) 逐点差分方程
% 3. 严格因果处理左右传感器位置 (yL, yR) 与输入电流 (iL, iR)
% 4. 摩擦重构使用因果单步后向速度差分:
%    首步: vL = 0, vR = 0, 记录 yL_prev, yR_prev
%    后步: vL = (yL - yL_prev)/dt, vR = (yR - yR_prev)/dt
% 5. 与 build_step3b_regression 批处理在理论与数值上完全逐点等价 (残差 <= 1e-12)
% =========================================================================

classdef rls_filter_svf_step3b
    properties
        fc = 10.0;           % 截止频率 (Hz)
        dt = 0.001;          % 采样步长 (s)
        
        % 离散 IIR 滤波器系数 (Direct Form II Transposed)
        den_a;               % 分母 a (1x5)
        num_w0;              % 分子 w0 (平滑滤波, 1x5)
        num_w1;              % 分子 w1 (一阶微分, 1x5)
        num_w2;              % 分子 w2 (二阶微分, 1x5)
        
        % 内部状态向量 (Direct Form II Transposed, 4x1 状态)
        state_alpha_w0;      % alpha_raw -> alpha_f
        state_alpha_w1;      % alpha_raw -> alpha_dot_f
        state_alpha_w2;      % alpha_raw -> alpha_ddot_f
        state_yG_w0;         % yG_raw    -> yG_f
        state_Tfric_w0;      % Tfric_raw -> Tfric_f
        state_sum_curr_w0;   % sum_curr  -> sum_current_f
        state_diff_curr_w0;  % diff_curr -> diff_current_f
        
        % 上一时刻传感器测量值
        yL_prev;
        yR_prev;
        is_initialized = false;
    end
    
    methods
        %% 构造函数: 初始化滤波器系数与内部状态
        function obj = rls_filter_svf_step3b(fc_in, dt_in)
            if nargin >= 1 && ~isempty(fc_in), obj.fc = fc_in; end
            if nargin >= 2 && ~isempty(dt_in), obj.dt = dt_in; end
            
            wc = 2.0 * pi * obj.fc;
            % 4 阶巴特沃斯标准分母多项式系数:
            % s^4 + c3*wc*s^3 + c2*wc^2*s^2 + c1*wc^3*s + wc^4
            % c1 = c3 = 2.61312592975275, c2 = 3.41421356237310
            poly_den = [1.0, 2.61312592975275 * wc, 3.41421356237310 * (wc^2), ...
                        2.61312592975275 * (wc^3), wc^4];
                    
            sys_w0 = tf(wc^4, poly_den);
            sys_w1 = tf([wc^4, 0.0], poly_den);
            sys_w2 = tf([wc^4, 0.0, 0.0], poly_den);
            
            % Tustin 双线性变换离散化
            sys_w0_d = c2d(sys_w0, obj.dt, 'tustin');
            sys_w1_d = c2d(sys_w1, obj.dt, 'tustin');
            sys_w2_d = c2d(sys_w2, obj.dt, 'tustin');
            
            [num0, den0] = tfdata(sys_w0_d, 'v');
            [num1, ~]    = tfdata(sys_w1_d, 'v');
            [num2, ~]    = tfdata(sys_w2_d, 'v');
            
            obj.den_a  = den0(:)';
            obj.num_w0 = num0(:)';
            obj.num_w1 = num1(:)';
            obj.num_w2 = num2(:)';
            
            obj = obj.reset();
        end
        
        %% 重置状态
        function obj = reset(obj)
            obj.state_alpha_w0     = zeros(4, 1);
            obj.state_alpha_w1     = zeros(4, 1);
            obj.state_alpha_w2     = zeros(4, 1);
            obj.state_yG_w0        = zeros(4, 1);
            obj.state_Tfric_w0     = zeros(4, 1);
            obj.state_sum_curr_w0  = zeros(4, 1);
            obj.state_diff_curr_w0 = zeros(4, 1);
            
            obj.yL_prev = 0.0;
            obj.yR_prev = 0.0;
            obj.is_initialized = false;
        end
        
        %% 单步因果递推
        % 输入:
        %   yL_meas, yR_meas: 左右导轨传感器位置 (m)
        %   iL_meas, iR_meas: 左右电机输入电流 (counts)
        %   mech: 机械参数结构体 (含 Le, J_alpha_nom)
        %   plant: 标称电气与摩擦参数 (含 b_nom, fc_nom, B_alpha, K_alpha)
        %   Kf_mean: 标称对称推力系数平均值 (N/count)
        % 输出:
        %   phi_f: 回归输入特征量 (count*m)
        %   y_f:   回归目标响应量 (N*m)
        %   diag_out: 详细诊断输出结构体
        function [obj, phi_f, y_f, diag_out] = step(obj, yL_meas, yR_meas, ...
                                                   iL_meas, iR_meas, ...
                                                   mech, plant, Kf_mean)
            Le = mech.Le;
            
            % 1. 速度测量与初值因果处理 (与批处理 [0; diff(y)]/dt 完全一致)
            if ~obj.is_initialized
                vL_meas = 0.0;
                vR_meas = 0.0;
                obj.yL_prev = yL_meas;
                obj.yR_prev = yR_meas;
                obj.is_initialized = true;
            else
                vL_meas = (yL_meas - obj.yL_prev) / obj.dt;
                vR_meas = (yR_meas - obj.yR_prev) / obj.dt;
                obj.yL_prev = yL_meas;
                obj.yR_prev = yR_meas;
            end
            
            % 2. 几何状态与摩擦项重构
            yG_raw    = 0.5 * (yL_meas + yR_meas);
            alpha_raw = (yR_meas - yL_meas) / Le;
            
            FfricL_raw = plant.b_nom * vL_meas + plant.fc_nom * tanh(100.0 * vL_meas);
            FfricR_raw = plant.b_nom * vR_meas + plant.fc_nom * tanh(100.0 * vR_meas);
            Tfric_raw  = 0.5 * Le * (FfricR_raw - FfricL_raw);
            
            sum_curr_raw  = iL_meas + iR_meas;
            diff_curr_raw = iL_meas - iR_meas;
            
            % 3. Direct Form II Transposed 逐通道因果滤波
            [alpha_f, obj.state_alpha_w0] = ...
                obj.filter_step(alpha_raw, obj.num_w0, obj.den_a, obj.state_alpha_w0);
            [alpha_dot_f, obj.state_alpha_w1] = ...
                obj.filter_step(alpha_raw, obj.num_w1, obj.den_a, obj.state_alpha_w1);
            [alpha_ddot_f, obj.state_alpha_w2] = ...
                obj.filter_step(alpha_raw, obj.num_w2, obj.den_a, obj.state_alpha_w2);
            
            [yG_f, obj.state_yG_w0] = ...
                obj.filter_step(yG_raw, obj.num_w0, obj.den_a, obj.state_yG_w0);
            [Tfric_f, obj.state_Tfric_w0] = ...
                obj.filter_step(Tfric_raw, obj.num_w0, obj.den_a, obj.state_Tfric_w0);
            
            [sum_current_f, obj.state_sum_curr_w0] = ...
                obj.filter_step(sum_curr_raw, obj.num_w0, obj.den_a, obj.state_sum_curr_w0);
            [diff_current_f, obj.state_diff_curr_w0] = ...
                obj.filter_step(diff_curr_raw, obj.num_w0, obj.den_a, obj.state_diff_curr_w0);
            
            % 4. 回归特征与响应计算
            Treq_f = mech.J_alpha_nom * alpha_ddot_f ...
                   + plant.B_alpha * alpha_dot_f ...
                   + plant.K_alpha * alpha_f ...
                   + Tfric_f;
               
            phi_f = 0.25 * Le * diff_current_f;
            y_f   = -Treq_f - 0.5 * Le * Kf_mean * sum_current_f;
            
            % 5. 详细诊断结构体打包
            if nargout >= 4
                diag_out = struct();
                diag_out.alpha_raw    = alpha_raw;
                diag_out.yG_raw       = yG_raw;
                diag_out.vL_meas      = vL_meas;
                diag_out.vR_meas      = vR_meas;
                diag_out.Tfric_raw    = Tfric_raw;
                diag_out.alpha_f      = alpha_f;
                diag_out.alpha_dot_f  = alpha_dot_f;
                diag_out.alpha_ddot_f = alpha_ddot_f;
                diag_out.yG_f         = yG_f;
                diag_out.Tfric_f      = Tfric_f;
                diag_out.sum_curr_f   = sum_current_f;
                diag_out.diff_curr_f  = diff_current_f;
                diag_out.Treq_f       = Treq_f;
                diag_out.phi_f        = phi_f;
                diag_out.y_f          = y_f;
            end
        end
    end
    
    methods (Static, Access = private)
        %% Direct Form II Transposed 单通道单步滤波核
        function [out_y, next_state] = filter_step(in_u, b, a, state)
            % 假定分母 a(1) == 1.0 (标准归一化多项式)
            out_y = b(1) * in_u + state(1);
            next_state = zeros(4, 1);
            next_state(1) = b(2) * in_u + state(2) - a(2) * out_y;
            next_state(2) = b(3) * in_u + state(3) - a(3) * out_y;
            next_state(3) = b(4) * in_u + state(4) - a(4) * out_y;
            next_state(4) = b(5) * in_u            - a(5) * out_y;
        end
    end
end
