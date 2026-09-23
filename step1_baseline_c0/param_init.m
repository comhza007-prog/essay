%% PARAM_INIT.M - 双M3508龙门搬运机构参数初始化与溯源表 (V2.0 严格复现版)
% =========================================================================
% 数据溯源说明规范（严格契合学术规范）：
% [程序参数]：直接来自下位机 Crane-down/Task/motor.h 与 motor.c
% [资料参数]：来自 团队技术报告V2.0 与 机械CAD零件
% [假设参数]：导轨摩擦、转动刚度等未测物理量，用于构造等效环境并做敏感性分析
% =========================================================================

function [ctrl, mech, plant] = param_init()

    %% 1. 控制器参数 [程序参数]
    ctrl.Ts = 0.001;                 % 控制周期 1ms (motor_task osDelay(1))
    
    % 速度环参数 (左右电机对称配置, 源自 motor.h)
    ctrl.spd_kp = 5.3;
    ctrl.spd_ki = 0.02;
    ctrl.spd_kd = 2.5;
    ctrl.spd_max_out = 16000.0;      % CAN电流控制指令上限 (raw counts, 标称16000)
    ctrl.spd_max_iout = 2000.0;      % 速度环积分限幅
    
    % 角度/位置环基础参数 (源自 motor.h)
    ctrl.ang_kp = 0.2;
    ctrl.ang_ki = 0.0;
    ctrl.ang_kd = 0.5;
    ctrl.ang_base_max_out = 3200.0;  % 静态位置环输出上限 (rpm)
    ctrl.ang_max_iout = 1000.0;
    
    % 角度环梯度动态限幅参数 (源自 motor.h: 严格复现 chassis_calculate 逻辑)
    ctrl.use_dynamic_ang_limit = true;   % 启用动态限幅
    ctrl.ang_gradient_min_out = 1000.0;  % 最小速度上限 (rpm)
    ctrl.ang_gradient_max_out = 12000.0; % 最大速度上限 (rpm)
    ctrl.ang_gradient_slope   = 200.0;   % 斜率 (rpm/圈)
    
    % 编码器参数 (源自 CAN_receive.c & motor.c)
    ctrl.ecd_cpr = 8192;             % 8192 counts/rev

    %% 2. 机械与传动结构参数 [资料参数]
    mech.mG_nom = 13.1;              % 整车空载标称质量 (kg, 团队技术报告V2.0)
    mech.Le = 0.60;                  % 龙门两导轨跨距 (m, 结构尺寸 600mm)
    mech.J_alpha_nom = (1/12) * mech.mG_nom * (mech.Le^2); % 标称转动惯量 (kg*m^2)
    
    % M3508 减速比区分说明:
    % - N_code: 现有程序 CAN_receive.c 中用于输出转数换算的值 (19.0)
    % - N_theory: M3508 自带行星减速箱理论齿轮比 (3591 / 187 ≈ 19.2032)
    mech.N_code = 19.0;
    mech.N_theory = 3591.0 / 187.0;
    mech.N = mech.N_code;            % 基线 C0 严格采用代码中实际使用的 19.0
    
    % 齿轮齿条传动参数 (源自机械 CAD: 3模 20齿)
    mech.m_gear = 3.0;               % 模数 m = 3 mm
    mech.z_gear = 20;                % 齿数 z = 20
    mech.rp = (mech.m_gear * mech.z_gear / 2.0) * 1e-3; % 分度圆半径 rp = 0.03 m (30 mm)
    mech.eta_g = 0.85;               % 传动系统机械效率 [假设参数]
    % 编码器单线脉冲当量 (8192 counts/rev, 减速比 N, 分度圆半径 rp)
    mech.dy_ecd = (2.0 * pi * mech.rp) / (mech.N * ctrl.ecd_cpr);
    
    % CAN 电流指令 -> 输出轴推力标定常数 [假设参数/待辨识]
    % 说明: 16000 为 CAN 指令 raw counts，非直接电流安培数。
    % 此处采用等效转矩系数假设值：设满幅 16000 对应输出轴标称力矩 3.5 N*m。
    mech.Kt_cmd = 3.5 / 16000.0;     % (N*m / count)
    % 单侧线性驱动力增益: F_i = (Kt_cmd * eta_g / rp) * i_cmd  (单位: N / count)
    mech.Kf = (mech.Kt_cmd * mech.eta_g) / mech.rp; 

    %% 3. 接触刚度与环境摩擦参数 [假设参数]
    plant.b_nom = 35.0;              % 单侧导轨标称黏性摩擦 (N*s/m)
    plant.fc_nom = 8.0;              % 单侧导轨标称库仑静摩擦 (N)
    plant.K_alpha = 2000.0;          % 横梁与导轨等效抗扭刚度 (N*m/rad)
    plant.B_alpha = 25.0;            % 偏转模态阻尼 (N*m*s/rad)
end
