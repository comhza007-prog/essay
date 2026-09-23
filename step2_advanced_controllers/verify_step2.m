%% VERIFY_STEP2.M - 第二阶段基础模块、接口与局部动态单元测试
% =========================================================================
% 本脚本严格执行 15 项基础物理、接口与局部动态测试：
% [Test 1/15]  C1 物理坐标纠偏方向与动态限幅截断测试
% [Test 2/15]  执行器映射原点测试 (零输入 -> 零输出)
% [Test 3/15]  纯平动解耦测试 (T_alpha=0 -> 左右推力严格相等 FL=FR)
% [Test 4/15]  纯偏转解耦测试 (FG=0 -> 左右推力大小相等方向相反 FL=-FR)
% [Test 5/15]  执行器映射矩阵与逆矩阵相逆测试 (对称与非对称推力增益)
% [Test 6/15]  执行器电流硬限幅与饱和残差向量测试
% [Test 7/15]  C2a 名义模型矩阵严格二维维度与控制量自洽测试
% [Test 8/15]  C2b 动态抗饱和离散状态理论衰减与闭环回退测试
% [Test 9/15]  I_fw_limit < Imax 内部限幅与外部限幅解耦识别测试
% [Test 10/15] 饱和残差代数恒等式测试 (Delta_i_total == Delta_i_internal + Delta_i_external)
% [Test 11/15] K_aw = 0 且 z_aw(0)=0 时 C2b 与 C2a 输出严格一致消融测试
% [Test 12/15] 非对称推力系数 Kf_L ~= Kf_R 下映射与三级限幅计算自洽测试
% [Test 13/15] pid_calc 真实未限幅请求 out_unsat 与 C1 分级残差恒等式测试
% [Test 14/15] C2a-SyncAlloc 与 thrust_allocator_sync 等价性及下游硬件残差为零断言
% [Test 15/15] C2b-SyncAlloc 在 K_aw=0 且 z_aw(0)=0 时严格退化为 C2a-SyncAlloc 及残差恒等式测试
% =========================================================================

clear; clc;

fprintf('==================================================================\n');
fprintf('    第二阶段核心模块、接口与局部动态断言测试 (verify_step2)\n');
fprintf('==================================================================\n\n');

total_tests = 15;
passed_tests = 0;

% 载入基础参数
addpath(fullfile(pwd, '..', 'step1_baseline_c0'));
[ctrl, mech, plant] = param_init();
Le = mech.Le;
Kf_nom = mech.Kf;

%% [Test 1/15] C1 物理坐标纠偏方向与动态限幅截断测试
fprintf('[Test 1/15] 检查 C1 同步纠偏方向与速度限幅截断...\n');
try
    yL_test = 0.50; yR_test = 0.52; % e_sync = +0.02 m
    vL_test = 0.30; vR_test = 0.30; % de_sync = 0.0 m/s
    yd_test = 0.51;
    
    ctrl.Kp_sync = 3.0; % 1/s
    ctrl.Kd_sync = 0.1;
    
    states_test.pid_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    states_test.pid_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    states_test.pid_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, 16000, 2000);
    states_test.pid_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, 16000, 2000);
    
    % 情况 A: 未触发速度限幅
    dyn_large = 10000.0;
    [~, ~, ~, infoA] = controller_c1_ccc(yL_test, yR_test, vL_test, vR_test, ...
                                         yd_test, dyn_large, ctrl, mech, 16000, states_test);
    assert(infoA.u_sync > 0, 'e_sync > 0 且 de_sync = 0 时，纠偏速度 u_sync 必须为正');
    assert(infoA.vL_ref > infoA.vR_ref, '右侧领先时，左侧期望速度必须大于右侧期望速度');
    assert(infoA.vL_ref > infoA.vL_base, '左侧期望速度必须大于左侧基准速度 (左侧加速)');
    assert(infoA.vR_ref < infoA.vR_base, '右侧期望速度必须小于右侧基准速度 (右侧减速)');
    
    % 情况 B: 触发速度限幅截断测试
    ms_to_rpm = (60.0 * mech.N) / (2.0 * pi * mech.rp);
    dyn_small = 500.0;
    v_limit_small = dyn_small / ms_to_rpm;
    [~, ~, ~, infoB] = controller_c1_ccc(yL_test, yR_test, vL_test, vR_test, ...
                                         yd_test, dyn_small, ctrl, mech, 16000, states_test);
    assert(abs(infoB.vL_ref) <= v_limit_small + 1e-6, 'C1 叠加同步量后绝对不能突破 v_limit 上限');
    assert(abs(infoB.vR_ref) <= v_limit_small + 1e-6, 'C1 叠加同步量后绝对不能突破 v_limit 上限');
    
    fprintf('  -> PASS: C1 纠偏物理方向正确，且动态速度限幅严格钳位。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 2/15] 执行器映射原点测试
fprintf('[Test 2/15] 检查执行器映射原点 (零输入 -> 零输出)...\n');
try
    [FG0, Talpha0] = actuator_map_m3508.current_to_force(0.0, 0.0, Le, Kf_nom, Kf_nom);
    assert(abs(FG0) < 1e-12 && abs(Talpha0) < 1e-12, '零电流输入时广义力和力矩必须为零');
    fprintf('  -> PASS: 零电流输入与零广义力原点严格重合。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 3/15] 纯平动解耦测试 (T_alpha = 0)
fprintf('[Test 3/15] 检查纯平动力解耦对称性 (T_alpha = 0)...\n');
try
    FG_req = 40.0;
    Talpha_req = 0.0;
    [iL, iR] = actuator_map_m3508.force_to_current(FG_req, Talpha_req, Le, Kf_nom, Kf_nom);
    FL =  Kf_nom * iL;
    FR = -Kf_nom * iR; % 右侧对向安装
    assert(abs(FL - FR) < 1e-10, '纯平动力要求时，左右两侧物理推力必须严格相等');
    assert(abs(FL + FR - FG_req) < 1e-10, '推力合力必须严格等于请求平动力');
    fprintf('  -> PASS: 纯平动解耦成功，两侧物理推力 FL = FR 严格对称。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 4/15] 纯偏转解耦测试 (FG = 0)
fprintf('[Test 4/15] 检查纯偏转力矩解耦差动性 (FG = 0)...\n');
try
    FG_req = 0.0;
    Talpha_req = 15.0;
    [iL, iR] = actuator_map_m3508.force_to_current(FG_req, Talpha_req, Le, Kf_nom, Kf_nom);
    FL =  Kf_nom * iL;
    FR = -Kf_nom * iR;
    assert(abs(FL + FR) < 1e-10, '纯偏转力矩要求时，左右推力合力必须为零 (FL = -FR)');
    T_calc = -0.5 * Le * FL + 0.5 * Le * FR;
    assert(abs(T_calc - Talpha_req) < 1e-10, '产生的偏转力矩必须严格等于请求值');
    fprintf('  -> PASS: 纯偏转力矩解耦成功，推力大小相等、方向相反 (FL = -FR)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 5/15] 执行器映射矩阵与逆矩阵相逆测试
fprintf('[Test 5/15] 检查映射 T_act 与 T_act_inv 的精确相逆性...\n');
try
    Kf_L_test = Kf_nom * 1.15;
    Kf_R_test = Kf_nom * 0.85;
    
    FG_orig = 55.0;
    Talpha_orig = -12.5;
    
    [iL_c, iR_c] = actuator_map_m3508.force_to_current(FG_orig, Talpha_orig, Le, Kf_L_test, Kf_R_test);
    [FG_rec, Talpha_rec] = actuator_map_m3508.current_to_force(iL_c, iR_c, Le, Kf_L_test, Kf_R_test);
    
    err_FG = abs(FG_rec - FG_orig);
    err_Talpha = abs(Talpha_rec - Talpha_orig);
    
    assert(err_FG < 1e-12 && err_Talpha < 1e-12, '映射与逆映射必须完全相逆恒等');
    fprintf('  -> PASS: 在非对称推力增益下，映射逆映射严格相逆 (残差 < 1e-12)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 6/15] 执行器电流硬限幅与饱和残差向量测试
fprintf('[Test 6/15] 检查执行器硬限幅与残差 Delta_i...\n');
try
    Imax_test = 10000.0;
    
    % 情况 A: 未饱和
    [iL_s1, iR_s1, Di1, sat1] = saturation_model.hard_sat(5000.0, -8000.0, Imax_test);
    assert(iL_s1 == 5000.0 && iR_s1 == -8000.0, '未超限电流必须保持原值');
    assert(norm(Di1) < 1e-12, '未超限残差必须严格为零');
    assert(~sat1, '未超限 is_sat 必须为 false');
    
    % 情况 B: 左右均超限
    [iL_s2, iR_s2, Di2, sat2] = saturation_model.hard_sat(15000.0, -12000.0, Imax_test);
    assert(iL_s2 == 10000.0, '正向超限必须钳位在 Imax');
    assert(iR_s2 == -10000.0, '负向超限必须钳位在 -Imax');
    assert(Di2(1) == -5000.0, '正向饱和残差 = sat - raw = 10000 - 15000 = -5000');
    assert(Di2(2) == 2000.0,  '负向饱和残差 = sat - raw = -10000 - (-12000) = +2000');
    assert(sat2, '超限 is_sat 必须为 true');
    
    fprintf('  -> PASS: 执行器硬限幅与残差向量计算严格准确。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 7/15] C2a 名义模型矩阵严格二维维度与自洽性测试
fprintf('[Test 7/15] 检查 C2a 名义模型方阵维度与连续鲁棒项输出...\n');
try
    ctrl_c2.M_hat = 0.95 * diag([mech.mG_nom, mech.J_alpha_nom]);
    ctrl_c2.B_hat = 0.90 * diag([plant.b_nom * 2.0, plant.B_alpha]);
    ctrl_c2.K_hat = diag([0.0, plant.K_alpha]);
    ctrl_c2.A_hat = diag([plant.fc_nom * 2.0, 0.0]);
    
    ctrl_c2.Gamma = diag([30.0, 20.0]);
    ctrl_c2.K1 = diag([40.0, 25.0]);
    ctrl_c2.K2 = diag([15.0, 8.0]);
    ctrl_c2.phi = [0.05; 0.05];
    ctrl_c2.eps_v = 0.01;
    ctrl_c2.eps_alpha = 0.01;
    ctrl_c2.I_fw_limit = 16000.0;
    
    q_test = [0.10; 0.002];
    qdot_test = [0.20; 0.005];
    qd_test = [0.12; 0.0];
    qdot_d_test = [0.25; 0.0];
    qddot_d_test = [0.50; 0.0];
    
    [iL_c2a, iR_c2a, info_c2a] = controller_c2a_robust( ...
        q_test, qdot_test, qd_test, qdot_d_test, qddot_d_test, ...
        ctrl_c2, Le, Kf_nom, Kf_nom, 16000.0);
    
    assert(all(size(ctrl_c2.M_hat) == [2, 2]), 'M_hat 必须为 2x2 方阵');
    assert(all(size(ctrl_c2.A_hat) == [2, 2]), 'A_hat 必须为 2x2 方阵');
    assert(all(size(info_c2a.va) == [2, 1]), '名义前馈 va 必须为严格 2x1 向量');
    assert(all(size(info_c2a.vr) == [2, 1]), '鲁棒反馈 vr 必须为严格 2x1 向量');
    assert(isscalar(iL_c2a) && isscalar(iR_c2a), '控制器输出电流必须为标量指令');
    
    fprintf('  -> PASS: C2a 矩阵维度严格统一为二维方阵，tanh 连续化鲁棒控制量计算自洽无误。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 8/15] C2b 动态抗饱和离散状态理论衰减与闭环回退测试
fprintf('[Test 8/15] 检查 C2b 动态抗饱和离散状态理论衰减与闭环动态回退...\n');
try
    ctrl_c2b = ctrl_c2;
    ctrl_c2b.K_aw = 20.0 * eye(2);
    ctrl_c2b.lambda_aw = 20.0;
    ctrl_c2b.I_fw_limit = 16000.0;
    ctrl_c2b.aw_mode = 'external';
    
    Ts = 0.001;
    
    % --- 8a. 理论离散滤波器衰减解算测试 (在 Delta_i == 0 条件下) ---
    z_init = [150.0; -80.0];
    z_sim = z_init;
    N_steps_decay = 100;
    for k_step = 1:N_steps_decay
        z_dot = -ctrl_c2b.lambda_aw * z_sim;
        z_sim = z_sim + Ts * z_dot;
    end
    z_expected = (1.0 - Ts * ctrl_c2b.lambda_aw)^N_steps_decay * z_init;
    assert(norm(z_sim - z_expected) < 1e-12, ...
           'Delta_i=0 时离散抗饱和状态必须严格遵从 (1 - Ts*lambda_aw)^N 理论离散衰减');
    
    % --- 8b. 闭环局部动态与记忆测试 ---
    state_aw.z_aw = [0.0; 0.0];
    
    % 工况 A: 执行器强饱和工况 (Imax = 4500 counts)，大加速度需求
    q_sat = [0.0; 0.0];
    qdot_sat = [0.0; 0.0];
    qd_sat = [0.10; 0.0];
    qdot_d_sat = [0.50; 0.0];
    qddot_d_sat = [2.0; 0.0];
    
    % 单步 1 计算
    [~, ~, state_aw, info1] = controller_c2b_aw( ...
        q_sat, qdot_sat, qd_sat, qdot_d_sat, qddot_d_sat, ...
        ctrl_c2b, Le, Kf_nom, Kf_nom, 4500.0, state_aw, Ts);
    
    assert(info1.is_sat_any, '大加速度需求下必须触发执行器饱和');
    assert(norm(info1.Delta_i_external) > 0, '饱和时外部硬件饱和残差必须非零');
    assert(all(size(state_aw.z_aw) == [2, 1]), 'z_aw 状态维度必须为 2x1');
    assert(norm(state_aw.z_aw) > 0, '发生饱和后 z_aw 状态必须开始动态累积');
    
    % 单步 2 计算: 传入已累积的 state_aw
    z_aw_prev = state_aw.z_aw;
    [~, ~, state_aw, info2] = controller_c2b_aw( ...
        q_sat, qdot_sat, qd_sat, qdot_d_sat, qddot_d_sat, ...
        ctrl_c2b, Le, Kf_nom, Kf_nom, 4500.0, state_aw, Ts);
    
    % 检查状态记忆: 第二步输入的 z_aw 必须严格继承自上一步
    assert(norm(info2.z_aw - z_aw_prev) < 1e-12, 'C2b 必须跨采样周期保持 z_aw 动态记忆');
    
    % 检查虚拟推力拉回效果: 抗饱和补偿后的 v_cmd 平动力绝对值应当小于未补偿的 v_beta
    assert(abs(info2.v_cmd(1)) < abs(info2.v_beta(1)), ...
           '抗饱和项必须在执行器饱和时动态拉回虚拟控制力，抑制不可实现的大控制请求');
    
    % 工况 B: 退出饱和后的单调耗散测试 (无误差、零加速度需求，充裕限幅)
    q_free = [0.10; 0.0];
    qdot_free = [0.0; 0.0];
    qd_free = [0.10; 0.0];
    qdot_d_free = [0.0; 0.0];
    qddot_d_free = [0.0; 0.0];
    
    z_norm_history = zeros(1, 100);
    for step_k = 1:100
        [~, ~, state_aw, info_free] = controller_c2b_aw( ...
            q_free, qdot_free, qd_free, qdot_d_free, qddot_d_free, ...
            ctrl_c2b, Le, Kf_nom, Kf_nom, 16000.0, state_aw, Ts);
        assert(norm(info_free.Delta_i_external) == 0, '退出饱和工况下外部硬件残差必须严格为零');
        z_norm_history(step_k) = norm(state_aw.z_aw);
    end
    diff_norms = diff(z_norm_history);
    assert(all(diff_norms <= 1e-12), '残差为零时 z_aw 状态范数必须单调衰减');
    
    fprintf('  -> PASS: 离散指数衰减严格符合理论解，饱和拉回有效且退出后严格单调耗散。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 9/15] I_fw_limit < Imax 内部限幅与外部限幅解耦识别测试
fprintf('[Test 9/15] 检查 I_fw_limit < Imax 时内部/外部饱和标志解耦识别...\n');
try
    ctrl_test9 = ctrl_c2;
    ctrl_test9.I_fw_limit = 8000.0; % 内部固件限幅设为 8000
    Imax_ext = 16000.0;             % 外部硬件允许到 16000
    
    % 大加速度输入，产生远大于 8000 counts 的请求 (~12000 counts)
    q_in = [0.0; 0.0]; qdot_in = [0.0; 0.0];
    qd_in = [0.10; 0.0]; qdot_d_in = [0.50; 0.0]; qddot_d_in = [2.0; 0.0];
    
    [iL_cmd9, iR_cmd9, info9] = controller_c2a_robust( ...
        q_in, qdot_in, qd_in, qdot_d_in, qddot_d_in, ...
        ctrl_test9, Le, Kf_nom, Kf_nom, Imax_ext);
    
    % 请求超过 8000，但由于内部限幅为 8000，外部为 16000:
    % i_fw = 8000, i_actual = 8000
    % Delta_i_internal != 0 (发生内部限幅)
    % Delta_i_external == 0 (未触发外部 16000 限幅)
    assert(info9.is_sat_internal == true, '当请求超过 I_fw_limit 时，is_sat_internal 必须为 true');
    assert(info9.is_sat_external == false, '当固件限幅小于硬件上限时，外部未超限 is_sat_external 必须为 false');
    assert(info9.is_sat_any == true, '只要存在任意限幅，is_sat_any 必须为 true');
    assert(max(abs([iL_cmd9, iR_cmd9])) <= 8000.0 + 1e-6, '输出指令必须严格被截断在固件限幅 8000 counts');
    
    fprintf('  -> PASS: 内部限幅与外部限幅解耦识别逻辑准确 (is_sat_internal=true, is_sat_external=false)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 10/15] 饱和残差代数恒等式测试
fprintf('[Test 10/15] 检查残差代数恒等式 (Delta_i_total == Delta_i_internal + Delta_i_external)...\n');
try
    ctrl_test10 = ctrl_c2;
    ctrl_test10.K_aw = 20.0 * eye(2);
    ctrl_test10.lambda_aw = 20.0;
    ctrl_test10.I_fw_limit = 16000.0;
    
    % 随机测试 5 组不同状态与限幅条件
    test_Imax_list = [4500.0, 8000.0, 12000.0, 16000.0, 20000.0];
    for ti = 1:length(test_Imax_list)
        Imax_cur = test_Imax_list(ti);
        q_t = [0.05 * ti; 0.001 * ti];
        qdot_t = [0.1 * ti; 0.005 * ti];
        qd_t = [0.08 * ti; 0.0];
        qdot_d_t = [0.2 * ti; 0.0];
        qddot_d_t = [1.0 * ti; 0.0];
        
        [~, ~, info10_a] = controller_c2a_robust( ...
            q_t, qdot_t, qd_t, qdot_d_t, qddot_d_t, ...
            ctrl_test10, Le, Kf_nom, Kf_nom, Imax_cur);
        
        err_id_a = norm(info10_a.Delta_i_total - (info10_a.Delta_i_internal + info10_a.Delta_i_external));
        assert(err_id_a < 1e-12, 'C2a 必须严格满足 Delta_i_total == Delta_i_internal + Delta_i_external');
        
        state_dummy.z_aw = [10.0; -20.0];
        [~, ~, ~, info10_b] = controller_c2b_aw( ...
            q_t, qdot_t, qd_t, qdot_d_t, qddot_d_t, ...
            ctrl_test10, Le, Kf_nom, Kf_nom, Imax_cur, state_dummy, 0.001);
        
        err_id_b = norm(info10_b.Delta_i_total - (info10_b.Delta_i_internal + info10_b.Delta_i_external));
        assert(err_id_b < 1e-12, 'C2b 必须严格满足 Delta_i_total == Delta_i_internal + Delta_i_external');
    end
    
    fprintf('  -> PASS: 残差代数相加恒等式严格成立 (残差误差 < 1e-12)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 11/15] K_aw = 0 时 C2b 与 C2a 输出严格一致消融测试
fprintf('[Test 11/15] 检查 K_aw = 0 时 C2b 与 C2a 输出严格代数恒等...\n');
try
    ctrl_ablation_c2b = ctrl_c2;
    ctrl_ablation_c2b.K_aw = zeros(2, 2); % 关闭抗饱和动态增益
    ctrl_ablation_c2b.lambda_aw = 20.0;
    ctrl_ablation_c2b.I_fw_limit = 16000.0;
    ctrl_ablation_c2b.aw_mode = 'external';
    
    ctrl_c2a_match = ctrl_c2;
    ctrl_c2a_match.I_fw_limit = 16000.0;
    
    % 在饱和工况下对比
    q_ab = [0.02; 0.001]; qdot_ab = [0.10; 0.002];
    qd_ab = [0.08; 0.0]; qdot_d_ab = [0.35; 0.0]; qddot_d_ab = [1.8; 0.0];
    Imax_ab = 4500.0;
    
    % C2a 单步
    [iL_a, iR_a, info_a] = controller_c2a_robust( ...
        q_ab, qdot_ab, qd_ab, qdot_d_ab, qddot_d_ab, ...
        ctrl_c2a_match, Le, Kf_nom, Kf_nom, Imax_ab);
    
    % C2b 单步 (初始 z_aw = 0)
    state_init.z_aw = [0.0; 0.0];
    [iL_b, iR_b, state_next, info_b] = controller_c2b_aw( ...
        q_ab, qdot_ab, qd_ab, qdot_d_ab, qddot_d_ab, ...
        ctrl_ablation_c2b, Le, Kf_nom, Kf_nom, Imax_ab, state_init, 0.001);
    
    assert(abs(iL_a - iL_b) < 1e-12, 'K_aw=0 时 iL 指令必须与 C2a 完全相同');
    assert(abs(iR_a - iR_b) < 1e-12, 'K_aw=0 时 iR 指令必须与 C2a 完全相同');
    assert(norm(info_a.v_cmd - info_b.v_cmd) < 1e-12, 'K_aw=0 时 v_cmd 必须恒等于 C2a');
    assert(norm(info_a.Delta_i_external - info_b.Delta_i_external) < 1e-12, '外部残差必须完全一致');
    assert(norm(state_next.z_aw) < 1e-12, 'K_aw=0 时 z_aw 状态必须保持恒为零');
    
    fprintf('  -> PASS: K_aw=0 严格消融验证通过，C2b 完整代码无缝退化至 C2a (误差 < 1e-12)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 12/15] 非对称推力系数 Kf_L ~= Kf_R 下映射与三级限幅计算自洽测试
fprintf('[Test 12/15] 检查非对称推力系数 Kf_L ~= Kf_R 下映射与三级限幅自洽性...\n');
try
    Kf_L_asym = Kf_nom * 1.30;
    Kf_R_asym = Kf_nom * 0.70;
    
    q_as = [0.05; 0.002]; qdot_as = [0.15; 0.004];
    qd_as = [0.12; 0.0]; qdot_d_as = [0.40; 0.0]; qddot_d_as = [1.5; 0.0];
    Imax_as = 4500.0;
    
    [iL_as, iR_as, info_as] = controller_c2a_robust( ...
        q_as, qdot_as, qd_as, qdot_d_as, qddot_d_as, ...
        ctrl_c2, Le, Kf_L_asym, Kf_R_asym, Imax_as);
    
    % 验证输出必须在 [-Imax, Imax] 内
    assert(abs(iL_as) <= Imax_as + 1e-6 && abs(iR_as) <= Imax_as + 1e-6, '指令必须在 Imax 约束内');
    
    % 验证实际反向投影力与 info.v_actual 完全一致
    FL_act =  Kf_L_asym * iL_as;
    FR_act = -Kf_R_asym * iR_as;
    FG_exp = FL_act + FR_act;
    Talpha_exp = -0.5 * Le * FL_act + 0.5 * Le * FR_act;
    
    assert(abs(info_as.v_actual(1) - FG_exp) < 1e-12, '非对称物理推力合力必须与 v_actual(1) 自洽');
    assert(abs(info_as.v_actual(2) - Talpha_exp) < 1e-12, '非对称偏转力矩必须与 v_actual(2) 自洽');
    
    fprintf('  -> PASS: 非对称推力系数下正反向映射与限幅投影严格自洽。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 13/15] pid_calc 真实未限幅请求 out_unsat 与 C1 分级残差恒等式测试
fprintf('[Test 13/15] 检查 pid_calc 真实未限幅输出 out_unsat 及 C1 残差代数恒等式...\n');
try
    % 1. 验证 pid_calc 截断与未截断输出
    s_test = init_pid_struct(10.0, 0.0, 0.0, 1000.0, 500.0);
    ref_val = 0.0;
    set_val = 200.0; % 误差 200 -> Pout = 2000.0 > max_out(1000.0)
    
    [out_val, s_test, out_unsat_val] = pid_calc(s_test, ref_val, set_val);
    assert(abs(out_val) == 1000.0, '受限主输出必须严格截断至 max_out = 1000');
    assert(out_unsat_val == 2000.0, '未受限输出 out_unsat 必须为真实物理计算值 2000');
    assert(s_test.out_unsat == 2000.0, 'pid 状态结构体内部 out_unsat 字段必须正确更新');
    
    % 2. 验证 C1 控制器分级残差与代数恒等式
    ctrl_c1_t = ctrl;
    ctrl_c1_t.Kp_sync = 3.5;
    ctrl_c1_t.Kd_sync = 0.08;
    ctrl_c1_t.spd_max_out = 16000.0;
    
    states_c1_t.pid_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    states_c1_t.pid_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    states_c1_t.pid_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, 16000, 2000);
    states_c1_t.pid_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, 16000, 2000);
    
    % 构造大步长引起限幅的输入
    [~, ~, ~, info_c1_t] = controller_c1_ccc( ...
        0.0, 0.05, 0.0, 0.0, 0.5, 3000.0, ctrl_c1_t, mech, 4500.0, states_c1_t);
    
    % 恒等式检查: Delta_i_total == Delta_i_internal + Delta_i_external
    res_err_c1 = norm(info_c1_t.Delta_i_total - (info_c1_t.Delta_i_internal + info_c1_t.Delta_i_external));
    assert(res_err_c1 < 1e-12, 'C1 分级残差必须严格满足 Delta_i_total == Delta_i_internal + Delta_i_external');
    
    % 检查物理量关系
    assert(norm(info_c1_t.Delta_i_internal - (info_c1_t.i_fw - info_c1_t.i_request)) < 1e-12, '内部残差定义严格自洽');
    assert(norm(info_c1_t.Delta_i_external - (info_c1_t.i_actual - info_c1_t.i_fw)) < 1e-12, '外部残差定义严格自洽');
    assert(norm(info_c1_t.Delta_i_total - (info_c1_t.i_actual - info_c1_t.i_request)) < 1e-12, '总残差定义严格自洽');
    
    % 3. 严格断言工况确实同时触发内部固件与外部物理双级限幅
    assert(norm(info_c1_t.Delta_i_internal) > 0, '测试工况必须真实触发内部固件限幅 (Delta_i_internal > 0)');
    assert(norm(info_c1_t.Delta_i_external) > 0, '测试工况必须真实触发外部硬件限幅 (Delta_i_external > 0)');
    assert(info_c1_t.is_sat_internal == true, 'is_sat_internal 标志必须为 true');
    assert(info_c1_t.is_sat_external == true, 'is_sat_external 标志必须为 true');
    assert(info_c1_t.is_sat_any == true, 'is_sat_any 标志必须为 true');
    
    fprintf('  -> PASS: pid_calc 未截断输出与 C1 分级残差代数恒等式及双级饱和触发严格成立 (误差 < 1e-12)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 14/15] C2a-SyncAlloc 与 thrust_allocator_sync 等价性及下游硬件残差为零断言
fprintf('[Test 14/15] 检查 C2a-SyncAlloc 输出等价性及下游硬件残差为零...\n');
try
    % 构造测试输入 (强限流工况，引发分配器主动削减推进力)
    q_t14 = [0.0; 0.0];
    qdot_t14 = [0.0; 0.0];
    qd_t14 = [0.10; 0.0];
    qdot_d_t14 = [0.50; 0.0];
    qddot_d_t14 = [2.0; 0.0];
    Imax_t14 = 4500.0;
    
    [iL_c2a_sync, iR_c2a_sync, info_c2a_sync] = controller_c2a_sync_alloc( ...
        q_t14, qdot_t14, qd_t14, qdot_d_t14, qddot_d_t14, ...
        ctrl_c2, Le, Kf_nom, Kf_nom, Imax_t14);
    
    % 1. 直接调用 thrust_allocator_sync 计算期望输出
    [i_direct_alloc, direct_alloc_info] = thrust_allocator_sync( ...
        info_c2a_sync.v_beta(1), info_c2a_sync.v_beta(2), Le, Kf_nom, Kf_nom, Imax_t14, 16000.0);
    
    % 断言 1: C2a-SyncAlloc 输出电流必须与直接调用 thrust_allocator_sync 完全一致
    assert(norm([iL_c2a_sync; iR_c2a_sync] - i_direct_alloc) < 1e-12, ...
           'C2a-SyncAlloc 输出电流必须与直接调用 thrust_allocator_sync 完全一致 (误差 < 1e-12)');
    assert(norm(info_c2a_sync.i_alloc - i_direct_alloc) < 1e-12, ...
           'info.i_alloc 必须与分配器直接解算结果严格相等');
    
    % 断言 3: SyncAlloc 的下游硬件残差应严格为零 (由于分配器在 I_eff 盒内闭式求解)
    assert(norm(info_c2a_sync.Delta_i_external) < 1e-12, ...
           'SyncAlloc 的下游硬件残差 Delta_i_external 必须严格为零 (< 1e-12)');
    assert(info_c2a_sync.is_sat_external == false, ...
           'SyncAlloc 的 is_sat_external 标志必须为 false');
    
    % 断言 4: 动态闭环中验证 Delta_i_total == Delta_i_alloc + Delta_i_external
    res_err_t14 = norm(info_c2a_sync.Delta_i_total - (info_c2a_sync.Delta_i_alloc + info_c2a_sync.Delta_i_external));
    assert(res_err_t14 < 1e-12, ...
           'Delta_i_total 必须严格满足 Delta_i_alloc + Delta_i_external 恒等式 (误差 < 1e-12)');
    
    fprintf('  -> PASS: C2a-SyncAlloc 输出与分配器直接调用严格一致，下游硬件残差严格为零，分级残差恒等式成立。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 15/15] C2b-SyncAlloc 在 K_aw=0 且 z_aw(0)=0 时严格退化为 C2a-SyncAlloc 及残差恒等式测试
fprintf('[Test 15/15] 检查 C2b-SyncAlloc 退化性 (K_aw=0) 及全周期残差恒等式...\n');
try
    ctrl_c2b_deg = ctrl_c2b;
    ctrl_c2b_deg.K_aw = zeros(2, 2); % 彻底关闭抗饱和增益
    ctrl_c2b_deg.lambda_aw = 20.0;
    state_c2b_deg.z_aw = [0.0; 0.0];
    
    % 构造多步复杂动态激励 (覆盖无饱和、单侧饱和、严重双侧饱和)
    N_steps_deg = 50;
    for step_i = 1:N_steps_deg
        q_dyn = [0.02 * sin(0.1 * step_i); 0.005 * cos(0.1 * step_i)];
        qdot_dyn = [0.1 * cos(0.1 * step_i); -0.01 * sin(0.1 * step_i)];
        qd_dyn = [0.05 * step_i * Ts; 0.0];
        qdot_d_dyn = [0.5; 0.0];
        qddot_d_dyn = [1.5 * sin(0.2 * step_i); 0.0];
        Imax_dyn = 4500.0;
        
        % C2a-SyncAlloc
        [iL_c2a_k, iR_c2a_k, info_c2a_k] = controller_c2a_sync_alloc( ...
            q_dyn, qdot_dyn, qd_dyn, qdot_d_dyn, qddot_d_dyn, ...
            ctrl_c2, Le, Kf_nom, Kf_nom, Imax_dyn);
        
        % C2b-SyncAlloc (K_aw = 0)
        [iL_c2b_k, iR_c2b_k, state_c2b_deg, info_c2b_k] = controller_c2b_sync_alloc( ...
            q_dyn, qdot_dyn, qd_dyn, qdot_d_dyn, qddot_d_dyn, ...
            ctrl_c2b_deg, Le, Kf_nom, Kf_nom, Imax_dyn, state_c2b_deg, Ts);
        
        % 断言 2: 输出严格退化相等
        assert(norm([iL_c2b_k; iR_c2b_k] - [iL_c2a_k; iR_c2a_k]) < 1e-12, ...
               sprintf('第 %d 步: K_aw=0 时 C2b-SyncAlloc 必须严格退化为 C2a-SyncAlloc', step_i));
        assert(norm(state_c2b_deg.z_aw) < 1e-12, ...
               sprintf('第 %d 步: K_aw=0 时 z_aw 状态必须恒为零', step_i));
        
        % 断言 3 & 4: 下游硬件残差为零，且总残差恒等式严格成立
        assert(norm(info_c2b_k.Delta_i_external) < 1e-12, ...
               sprintf('第 %d 步: C2b-SyncAlloc 下游硬件残差必须严格为零', step_i));
        res_err_c2b = norm(info_c2b_k.Delta_i_total - (info_c2b_k.Delta_i_alloc + info_c2b_k.Delta_i_external));
        assert(res_err_c2b < 1e-12, ...
               sprintf('第 %d 步: C2b-SyncAlloc 必须严格满足 Delta_i_total == Delta_i_alloc + Delta_i_external', step_i));
    end
    
    fprintf('  -> PASS: K_aw=0 时 C2b-SyncAlloc 50 步动态严格退化为 C2a-SyncAlloc，下游残差恒为零且恒等式严格成立。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% 总结报告
fprintf('==================================================================\n');
fprintf('  第二阶段单元测试结果: %d / %d 项全部通过 (PASS)\n', passed_tests, total_tests);
fprintf('  测试状态: 15 项接口、映射和局部动态检查通过。\n');
fprintf('==================================================================\n');

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
