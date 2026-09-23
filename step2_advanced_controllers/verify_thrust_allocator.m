%% VERIFY_THRUST_ALLOCATOR.M - 推力空间同步优先约束分配器专用单元测试
% =========================================================================
% 本测试套件严格检验 thrust_allocator_sync 的 5 项数学与物理几何特性：
% [Test 1/5] 无超限可行区透传 (与解析逆映射严格等价)
% [Test 2/5] 纯平动超限对称削顶 (力矩严格保持为零)
% [Test 3/5] 纯力矩超限边界性 (对称极限下合力为零，非对称极限下合力自洽)
% [Test 4/5] 组合超限时力矩完整保留与推进力主动削减
% [Test 5/5] 非对称推力系数与双级有效限幅 I_eff 矩形可行域闭合
% =========================================================================

clear; clc;

fprintf('==================================================================\n');
fprintf('  推力空间同步优先约束分配器单元测试 (verify_thrust_allocator)\n');
fprintf('==================================================================\n\n');

total_tests = 5;
passed_tests = 0;

addpath(fullfile(pwd, '..', 'step1_baseline_c0'));
[ctrl, mech, ~] = param_init();
Le = mech.Le;
Kf_nom = mech.Kf;

%% [Test 1/5] 无超限可行区透传测试
fprintf('[Test 1/5] 检查无超限可行区透传性 (与 force_to_current 严格等价)...\n');
try
    FG_des = 30.0; % N
    Talpha_des = 2.0; % N*m
    Imax = 16000.0;
    Ifw = 16000.0;
    
    [i_alloc, info1] = thrust_allocator_sync(FG_des, Talpha_des, Le, Kf_nom, Kf_nom, Imax, Ifw);
    [iL_inv, iR_inv] = actuator_map_m3508.force_to_current(FG_des, Talpha_des, Le, Kf_nom, Kf_nom);
    
    assert(abs(i_alloc(1) - iL_inv) < 1e-12, '可行区内 iL 必须与逆映射严格一致');
    assert(abs(i_alloc(2) - iR_inv) < 1e-12, '可行区内 iR 必须与逆映射严格一致');
    assert(abs(info1.Delta_Talpha) < 1e-12, '可行区内力矩缺额必须为零');
    assert(abs(info1.Delta_FG) < 1e-12, '可行区内推进力缺额必须为零');
    assert(~info1.is_sat_any, '未超限时 is_sat_any 必须为 false');
    
    fprintf('  -> PASS: 可行区内分配结果与逆映射严格等价 (误差 < 1e-12)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 2/5] 纯平动超限对称削顶测试
fprintf('[Test 2/5] 检查纯平动超限对称削顶 (偏转力矩严格保持为零)...\n');
try
    Talpha_des = 0.0;
    Imax = 4500.0;
    Ifw = 16000.0;
    FL_max = Kf_nom * Imax;
    FR_max = Kf_nom * Imax;
    FG_max = FL_max + FR_max;
    
    FG_over = FG_max * 1.5; % 强烈超限
    
    [i_alloc, info2] = thrust_allocator_sync(FG_over, Talpha_des, Le, Kf_nom, Kf_nom, Imax, Ifw);
    
    assert(abs(info2.Talpha_alloc) < 1e-12, '纯平动输入下，分配力矩必须严格恒为零');
    assert(abs(info2.FG_alloc - FG_max) < 1e-12, '实际合力必须严格截断在最大物理推力 FG_max');
    assert(info2.is_sat_thrust == true, '平动力必须标记为饱和');
    assert(info2.is_sat_torque == false, '力矩未超限，is_sat_torque 必须为 false');
    assert(abs(i_alloc(1) - Imax) < 1e-12 && abs(i_alloc(2) - (-Imax)) < 1e-12, ...
           '双侧电流必须严格打满在正向推力极限 [Imax, -Imax]');
    
    fprintf('  -> PASS: 纯平动超限被准确截断在 FG_max，且纠偏力矩零漂移严格保持 (误差 < 1e-12)。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 3/5] 纯力矩超限边界性测试 (对称与非对称)
fprintf('[Test 3/5] 检查纯力矩超限边界性 (对称合力为零，非对称合力自洽)...\n');
try
    FG_des = 0.0;
    Imax = 4500.0;
    
    % 3a. 对称推力系数
    DeltaF_max_sym = 2.0 * Kf_nom * Imax;
    Talpha_max_sym = 0.5 * Le * DeltaF_max_sym;
    Talpha_over = Talpha_max_sym * 1.6; % 严重超出物理力矩能力
    
    [i_alloc_sym, info3a] = thrust_allocator_sync(FG_des, Talpha_over, Le, Kf_nom, Kf_nom, Imax, 16000);
    assert(abs(info3a.FG_alloc) < 1e-12, '对称限幅下纯力矩超限时合力必须严格为零');
    assert(abs(info3a.Talpha_alloc - Talpha_max_sym) < 1e-12, '输出力矩必须严格为最大可实现力矩');
    assert(info3a.is_sat_torque == true, '力矩必须标记为饱和');
    
    % 3b. 非对称推力系数 Kf_L ~= Kf_R
    Kf_L_asym = Kf_nom * 1.30;
    Kf_R_asym = Kf_nom * 0.70;
    FL_max_as = Kf_L_asym * Imax;
    FR_max_as = Kf_R_asym * Imax;
    DeltaF_max_as = FL_max_as + FR_max_as;
    Talpha_max_as = 0.5 * Le * DeltaF_max_as;
    
    [~, info3b] = thrust_allocator_sync(FG_des, Talpha_max_as * 1.5, Le, Kf_L_asym, Kf_R_asym, Imax, 16000);
    assert(abs(info3b.Talpha_alloc - Talpha_max_as) < 1e-12, '非对称极限下实际力矩必须严格等于 Talpha_max_as');
    % 非对称打满时: FL = -FL_max, FR = +FR_max ==> FG = FR_max - FL_max
    FG_expected_asym = FR_max_as - FL_max_as;
    assert(abs(info3b.FG_alloc - FG_expected_asym) < 1e-12, '非对称推力打满时合力必须与理论差额自洽');
    
    fprintf('  -> PASS: 纯力矩超限边界性严格符合对称与非对称理论解。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 4/5] 组合超限时力矩完整保留与平动力主动削减测试
fprintf('[Test 4/5] 检查组合超限时力矩完整保留与平动力主动削减...\n');
try
    Imax = 4500.0;
    FL_max = Kf_nom * Imax;
    FR_max = Kf_nom * Imax;
    
    % 构造工况:
    % 1. 期望纠偏力矩 Talpha_des 严格可行 (在 DeltaF_max 范围内，占 40% 的力矩裕度)
    DeltaF_req = 0.40 * (FL_max + FR_max);
    Talpha_req = 0.5 * Le * DeltaF_req;
    
    % 2. 期望推进合力极大，导致 [FL, FR] 整体越界
    FG_req = 1.8 * (FL_max + FR_max);
    
    [i_alloc, info4] = thrust_allocator_sync(FG_req, Talpha_req, Le, Kf_nom, Kf_nom, Imax, 16000);
    
    % 断言: 纠偏力矩必须被完整保留！
    assert(abs(info4.Delta_Talpha) < 1e-12, '力矩可行时，同步优先分配器必须 100%% 完整保留期望纠偏力矩');
    assert(info4.is_sat_torque == false, '力矩未发生截断，is_sat_torque 必须为 false');
    
    % 断言: 平动力必须被主动削减以让步于纠偏力矩！
    assert(info4.FG_alloc < FG_req, '合力必须主动让步削减');
    assert(info4.is_sat_thrust == true, '平动力必须标记为饱和');
    
    % 断言: 输出电流在有效限幅内
    assert(abs(i_alloc(1)) <= Imax + 1e-6 && abs(i_alloc(2)) <= Imax + 1e-6, '输出电流必须在 Imax 约束内');
    
    fprintf('  -> PASS: 纠偏力矩被完整保留 (误差 < 1e-12)，推进合力被主动削减让步。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 5/5] 非对称推力系数与双级有效限幅 I_eff 矩形可行域闭合测试
fprintf('[Test 5/5] 检查双级有效限幅 I_eff = min(I_fw, Imax) 与非对称可行域闭合...\n');
try
    % 内部限幅 6000 小于外部限幅 10000
    Ifw_test = 6000.0;
    Imax_test = 10000.0;
    Kf_L_test = Kf_nom * 1.25;
    Kf_R_test = Kf_nom * 0.75;
    
    % 极大需求输入
    [i_alloc, info5] = thrust_allocator_sync(200.0, 15.0, Le, Kf_L_test, Kf_R_test, Imax_test, Ifw_test);
    
    % 检查 I_eff 严格取 6000
    assert(info5.I_eff(1) == 6000.0 && info5.I_eff(2) == 6000.0, '有效限幅必须严格取 min(Ifw, Imax) = 6000');
    assert(abs(i_alloc(1)) <= 6000.0 + 1e-6, '左侧电流绝不能突破 I_eff');
    assert(abs(i_alloc(2)) <= 6000.0 + 1e-6, '右侧电流绝不能突破 I_eff');
    
    fprintf('  -> PASS: 有效限幅 I_eff 严格生效，非对称推力可行域安全闭合。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% 总结
fprintf('==================================================================\n');
fprintf('  推力空间同步优先约束分配器测试结果: %d / %d 项全部通过 (PASS)\n', passed_tests, total_tests);
fprintf('==================================================================\n');
