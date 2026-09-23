%% STEP3B_OFFLINE_CALIBRATION.M - 执行器推力系数不对称度离线标定与前馈增益解算函数
% =========================================================================
% 功能说明:
% 1. 基于 Phase 1 辨识获得的推力非对称参数 Delta_Kf_hat 与标称推力系数 Kf_mean,
%    严格依据物理定义反解双轴单侧推力系数标定值:
%      Kf_L_hat = Kf_mean + 0.5 * Delta_Kf_hat
%      Kf_R_hat = Kf_mean - 0.5 * Delta_Kf_hat
% 2. 计算双轴驱动器对称化前馈补偿无量纲增益因子:
%      gamma_L = Kf_mean / Kf_L_hat
%      gamma_R = Kf_mean / Kf_R_hat
% 3. 严格的前置合法性检查与正值断言, 确保无除零与物理非法值
% =========================================================================

function calib = step3b_offline_calibration(Delta_Kf_hat, Kf_mean, opts)
    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    
    % 1. 前置有限值与合法性断言
    assert(isfinite(Delta_Kf_hat), 'Delta_Kf_hat 必须为有限值');
    assert(isfinite(Kf_mean) && Kf_mean > 0, 'Kf_mean 必须为正有限值');
    
    % 2. 标定参数反解
    Kf_L_hat = Kf_mean + 0.5 * Delta_Kf_hat;
    Kf_R_hat = Kf_mean - 0.5 * Delta_Kf_hat;
    
    % 3. 物理正值断言
    assert(Kf_L_hat > 0, '左侧推力标定值 Kf_L_hat 必须严格大于零');
    assert(Kf_R_hat > 0, '右侧推力标定值 Kf_R_hat 必须严格大于零');
    
    % 4. 补偿增益计算
    gamma_L = Kf_mean / Kf_L_hat;
    gamma_R = Kf_mean / Kf_R_hat;
    
    assert(isfinite(gamma_L) && gamma_L > 0, '左侧增益 gamma_L 必须为正有限值');
    assert(isfinite(gamma_R) && gamma_R > 0, '右侧增益 gamma_R 必须为正有限值');
    
    % 5. 标定不对称比
    r_hat = Kf_L_hat / Kf_R_hat;
    is_valid = (calib_in_bounds(r_hat, opts));
    
    % 6. 构造输出结构体
    calib = struct();
    calib.Delta_Kf_hat = Delta_Kf_hat;
    calib.Kf_mean      = Kf_mean;
    calib.Kf_L_hat     = Kf_L_hat;
    calib.Kf_R_hat     = Kf_R_hat;
    calib.gamma_L      = gamma_L;
    calib.gamma_R      = gamma_R;
    calib.r_hat        = r_hat;
    calib.is_valid     = is_valid;
end

function ok = calib_in_bounds(r, opts)
    % 默认对应 Phase 1 声明的物理允许区间 r in [0.65, 1.35] (含数值容限 1e-4)
    r_min = 0.65;
    r_max = 1.35;
    tol = 1e-4;
    if isfield(opts, 'r_min'), r_min = opts.r_min; end
    if isfield(opts, 'r_max'), r_max = opts.r_max; end
    if isfield(opts, 'tol'),   tol   = opts.tol;   end
    ok = (r >= (r_min - tol) && r <= (r_max + tol));
end
