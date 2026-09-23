%% RLS_ESTIMATOR_DELTA_KF.M - 单参数 Delta_Kf 递推最小二乘 (RLS) 估计器类
% =========================================================================
% 功能说明:
% 1. 针对 Step 3B 执行器推力非对称单参数线性回归模型:
%    y_f(k) = phi_f(k) * Delta_Kf + e(k)
% 2. 具备滑动窗口均方根能量 PE (持续激励) 门控逻辑:
%    G_k = (1/Nw) * sum(phi_f^2)
%    pe_metric = sqrt(max(G_k, 0)) [count*m]
%    当 pe_metric >= sigma_PE_th 且窗口充满时激活更新；
%    当 PE 不满足时，参数与协方差完全冻结 (严禁除以 lambda，彻底杜绝协方差风积与零漂)
% 3. 严格规范的标量 RLS 递推与数值保护:
%    K = (P * phi) / (lambda + phi * P * phi)
%    theta_unprojected = theta + K * (y - phi * theta)
%    P_unprojected     = (P - K * phi * P) / lambda
%    P_bounded         = min(max(P_unprojected, P_min), P_max)
%    theta_projected   = min(max(theta_unprojected, theta_min), theta_max)
% 4. 显式记录并输出三态参数 (theta_prev, theta_unprojected, theta_projected) 及协方差变化
% =========================================================================

classdef rls_estimator_delta_kf
    properties
        % 算法超参数
        lambda = 1.0;             % 遗忘因子 (默认 1.0 用于离线收敛, 可配置 0.98~1.0)
        sigma_PE_th = 50.0;       % PE 门控均方根能量阈值 (count*m)
        G_PE_th;                  % PE 能量方阈值 (count*m)^2, 自动计算
        window_length = 200;      % PE 环形滑动窗口长度 (默认 200 步 = 200 ms)
        
        % 凸集投影边界 (单位: N/count)
        % 物理精确边界 (对应 r = Kf_L / Kf_R in [0.65, 1.35]):
        %   theta_min = -0.0026295 N/count (r = 0.65)
        %   theta_max = +0.0018465 N/count (r = 1.35)
        % 对称工程边界 (覆盖但宽于 [0.65, 1.35]):
        %   [-0.00263, +0.00263] N/count
        theta_min = -0.0026295;
        theta_max =  0.0018465;
        
        % 协方差初值与数值边界 (单位: (N/count)^2)
        P0 = 1.0e-4;              % 协方差初值
        P_min = 1.0e-12;          % 协方差保护下界
        P_max = 1.0;              % 协方差保护上界
        
        % 估计器当前有效状态
        theta_hat = 0.0;          % 当前有效估计值 (N/count)
        P;                        % 当前有效协方差 ((N/count)^2)
        
        % 内部历史记录
        theta_prev = 0.0;
        P_prev;
        
        % PE 环形缓冲区
        phi_buffer;
        buf_idx = 1;
        buf_count = 0;
        
        % 统计计数器
        steps_count = 0;
        projection_count = 0;     % 累计触发投影截断次数
    end
    
    methods
        %% 构造函数
        function obj = rls_estimator_delta_kf(opts)
            if nargin >= 1 && isstruct(opts)
                if isfield(opts, 'lambda'),        obj.lambda = opts.lambda; end
                if isfield(opts, 'sigma_PE_th'),   obj.sigma_PE_th = opts.sigma_PE_th; end
                if isfield(opts, 'window_length'), obj.window_length = opts.window_length; end
                if isfield(opts, 'theta_min'),     obj.theta_min = opts.theta_min; end
                if isfield(opts, 'theta_max'),     obj.theta_max = opts.theta_max; end
                if isfield(opts, 'P0'),            obj.P0 = opts.P0; end
                if isfield(opts, 'P_min'),         obj.P_min = opts.P_min; end
                if isfield(opts, 'P_max'),         obj.P_max = opts.P_max; end
                if isfield(opts, 'theta0'),        obj.theta_hat = opts.theta0; end
            end
            
            obj.G_PE_th = obj.sigma_PE_th ^ 2;
            obj = obj.reset();
        end
        
        %% 重置估计器状态
        function obj = reset(obj, theta0, P0)
            if nargin >= 2 && ~isempty(theta0), obj.theta_hat = theta0; end
            if nargin >= 3 && ~isempty(P0),     obj.P0 = P0; end
            
            obj.P = obj.P0;
            obj.theta_prev = obj.theta_hat;
            obj.P_prev = obj.P0;
            
            obj.phi_buffer = zeros(obj.window_length, 1);
            obj.buf_idx = 1;
            obj.buf_count = 0;
            
            obj.steps_count = 0;
            obj.projection_count = 0;
        end
        
        %% 单步因果递推更新主函数
        % 输入:
        %   phi: 回归特征量 (count*m)
        %   y:   回归目标响应量 (N*m)
        % 输出:
        %   obj: 更新后的估计器对象
        %   theta_hat: 当前时刻最终投影后估计值 (N/count)
        %   info: 详细诊断与状态结构体
        function [obj, theta_hat, info] = update(obj, phi, y)
            obj.steps_count = obj.steps_count + 1;
            
            % 1. 维护环形滑动窗口
            obj.phi_buffer(obj.buf_idx) = phi;
            obj.buf_idx = mod(obj.buf_idx, obj.window_length) + 1;
            obj.buf_count = min(obj.buf_count + 1, obj.window_length);
            
            % 2. 计算 PE 均方根标量能量指标
            if obj.buf_count < obj.window_length
                G = 0.0;
                pe_metric = 0.0;
                is_pe = false;
            else
                G = mean(obj.phi_buffer .^ 2);
                pe_metric = sqrt(max(G, 0.0));
                is_pe = (pe_metric >= obj.sigma_PE_th);
            end
            
            % 3. 门控分支处理
            if is_pe
                % 3.1 标量增益计算
                den = obj.lambda + phi * obj.P * phi;
                K   = (obj.P * phi) / den;
                
                % 3.2 预测残差与未投影更新
                innov = y - phi * obj.theta_hat;
                theta_unprojected = obj.theta_hat + K * innov;
                P_unprojected     = (obj.P - K * phi * obj.P) / obj.lambda;
                
                % 3.3 协方差数值保护与对称标量截断
                P_bounded = min(max(P_unprojected, obj.P_min), obj.P_max);
                
                % 3.4 紧凑凸集投影
                theta_projected = min(max(theta_unprojected, obj.theta_min), obj.theta_max);
                is_proj = (theta_projected ~= theta_unprojected);
                if is_proj
                    obj.projection_count = obj.projection_count + 1;
                end
                
                % 3.5 状态保存
                obj.theta_prev = obj.theta_hat;
                obj.P_prev     = obj.P;
                
                obj.theta_hat  = theta_projected;
                obj.P          = P_bounded;
            else
                % 3.6 PE 不满足：绝对完全冻结！
                % 参数与协方差均保持不变，严禁除以 lambda
                innov = y - phi * obj.theta_hat;
                K = 0.0;
                
                theta_unprojected = obj.theta_hat;
                theta_projected   = obj.theta_hat;
                is_proj           = false;
                
                obj.theta_prev = obj.theta_hat;
                obj.P_prev     = obj.P;
                % obj.theta_hat 与 obj.P 保持原值不变
            end
            
            theta_hat = obj.theta_hat;
            
            % 4. 构造完整诊断输出结构体
            if nargout >= 3
                info = struct();
                info.pe_metric         = pe_metric;
                info.is_pe             = is_pe;
                info.innovation        = innov;
                info.gain              = K;
                info.theta_prev        = obj.theta_prev;
                info.theta_unprojected = theta_unprojected;
                info.theta_projected   = theta_projected;
                info.P_prev            = obj.P_prev;
                info.P_next            = obj.P;
                info.is_projected      = is_proj;
            end
        end
        
        %% 单步递推更新别名
        function [obj, theta_hat, info] = step(obj, phi, y)
            [obj, theta_hat, info] = obj.update(phi, y);
        end
    end
end
