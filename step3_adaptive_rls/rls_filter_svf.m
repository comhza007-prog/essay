%% RLS_FILTER_SVF.M - 4 阶因果巴特沃斯状态变量滤波器 (SVF) 类
% =========================================================================
% 功能：
% 1. 采用 4 阶稳定巴特沃斯多项式构造因果状态变量滤波器
% 2. 避免对带量化噪声的编码器位移进行二次差分，高频增益滚降抑制高频微分噪声
% 3. 输入位移 yG, 施加总推力 FG = Kf_nom * (iL - iR)
% 4. 摩擦项按严格物理因果顺序滤波: Sf_raw = tanh(100*v_meas) -> W0(z) -> Sf_f
% 5. 支持单步实时递推 (Direct Form II Transposed)
% =========================================================================

classdef rls_filter_svf
    properties
        fc = 10.0;           % 截止频率 (Hz)
        dt = 0.001;          % 采样步长 (s)
        
        % 离散 IIR 差分方程系数 (Tustin 变换)
        % 分母 a 相同
        den_a;
        % 分子 b: num_w0 (滤波量), num_w1 (一阶导), num_w2 (二阶导)
        num_w0;
        num_w1;
        num_w2;
        
        % 滤波器内部时延状态向量 (Direct Form II Transposed, 4 维)
        state_y_w0;          % yG -> yG_f
        state_y_w1;          % yG -> ydot_f
        state_y_w2;          % yG -> yddot_f
        state_F_w0;          % FG -> FG_f
        state_Sf_w0;         % Sf_raw -> Sf_f
        
        % 上一步位移 (用于单步后向差分测速)
        yG_prev;
        is_initialized = false;
    end
    
    methods
        %% 构造函数: 初始化滤波器系数与内部状态
        function obj = rls_filter_svf(fc_in, dt_in)
            if nargin >= 1 && ~isempty(fc_in), obj.fc = fc_in; end
            if nargin >= 2 && ~isempty(dt_in), obj.dt = dt_in; end
            
            wc = 2.0 * pi * obj.fc;
            % 4 阶巴特沃斯标准分母多项式系数:
            % s^4 + c3*wc*s^3 + c2*wc^2*s^2 + c1*wc^3*s + wc^4
            % c1 = c3 = 2.6131259, c2 = 3.4142136
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
            obj.state_y_w0  = zeros(4, 1);
            obj.state_y_w1  = zeros(4, 1);
            obj.state_y_w2  = zeros(4, 1);
            obj.state_F_w0  = zeros(4, 1);
            obj.state_Sf_w0 = zeros(4, 1);
            obj.yG_prev = 0.0;
            obj.is_initialized = false;
        end
        
        %% 单步因果递推 (Direct Form II Transposed)
        function [obj, yG_f, ydot_f, yddot_f, FG_f, Sf_f] = step(obj, yG_meas, FG_applied)
            if ~obj.is_initialized
                obj.yG_prev = yG_meas;
                obj.is_initialized = true;
            end
            
            % 1. 测量速度与物理原摩擦信号
            v_meas = (yG_meas - obj.yG_prev) / obj.dt;
            obj.yG_prev = yG_meas;
            Sf_raw = tanh(100.0 * v_meas);
            
            % 2. Direct Form II Transposed 滤波递推
            % y(k) = b(1)*u(k) + z1(k-1)
            % z1(k) = b(2)*u(k) + z2(k-1) - a(2)*y(k)
            % ...
            % yG -> yG_f
            [yG_f, obj.state_y_w0] = obj.filter_step(yG_meas, obj.num_w0, obj.den_a, obj.state_y_w0);
            % yG -> ydot_f
            [ydot_f, obj.state_y_w1] = obj.filter_step(yG_meas, obj.num_w1, obj.den_a, obj.state_y_w1);
            % yG -> yddot_f
            [yddot_f, obj.state_y_w2] = obj.filter_step(yG_meas, obj.num_w2, obj.den_a, obj.state_y_w2);
            % FG_applied -> FG_f
            [FG_f, obj.state_F_w0] = obj.filter_step(FG_applied, obj.num_w0, obj.den_a, obj.state_F_w0);
            % Sf_raw -> Sf_f (严格物理原信号滤波)
            [Sf_f, obj.state_Sf_w0] = obj.filter_step(Sf_raw, obj.num_w0, obj.den_a, obj.state_Sf_w0);
        end
    end
    
    methods (Static, Access = private)
        %% IIR 单通道单步滤波
        function [out_y, next_state] = filter_step(in_u, b, a, state)
            % b, a 长度为 5
            out_y = b(1) * in_u + state(1);
            next_state = zeros(4, 1);
            next_state(1) = b(2) * in_u + state(2) - a(2) * out_y;
            next_state(2) = b(3) * in_u + state(3) - a(3) * out_y;
            next_state(3) = b(4) * in_u + state(4) - a(4) * out_y;
            next_state(4) = b(5) * in_u            - a(5) * out_y;
        end
    end
end
