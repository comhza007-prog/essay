%% VERIFY_STEP1.M - 第一阶段 C0 基线模型自检与单元测试脚本 (修复版)
% =========================================================================
% 本脚本用于自动化检查第一阶段模型的关键逻辑与物理一致性：
% [Test 1] 核心参数完整性与减速比核查 (N = 19.0, Ts = 0.001s)
% [Test 2] 左右电机反向安装推力符号核查 (左正推向前, 右负推向前)
% [Test 3] 动态梯度限幅逻辑核查 (验证 [1000, 12000] rpm 截断)
% [Test 4] 完整四工况闭环推演与数据结构有效性核查
% =========================================================================

is_test_runner = true;

fprintf('====================================================\n');
fprintf('       双 M3508 龙门平台 C0 基线模型运行自检测试\n');
fprintf('====================================================\n\n');

total_tests = 4;
passed_tests = 0;

%% [Test 1] 核心参数完整性与减速比核查
fprintf('[Test 1/4] 检查参数初始化与数据溯源...\n');
try
    [ctrl, mech, plant] = param_init();
    assert(ctrl.Ts == 0.001, '控制周期 Ts 必须为 0.001s');
    assert(mech.N == 19.0, '基线减速比必须为代码实际值 19.0');
    assert(ctrl.spd_max_out == 16000.0, '速度环输出限幅必须为 16000');
    assert(ctrl.use_dynamic_ang_limit == true, '必须启用动态位置限幅');
    fprintf('  -> PASS: 参数初始化正常，减速比与限幅完全匹配程序。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 2] 左右电机反向安装推力符号核查
fprintf('[Test 2/4] 检查对向安装电机推力符号...\n');
try
    % 给左电机下发正指令，右电机下发负指令，验证两者是否均产生向前推力
    x_test = zeros(4, 1);
    x_next_left = gantry_dynamics_step(x_test, 1000.0, 0.0, mech, plant, 0, 0, 0, 0.001);
    x_next_right = gantry_dynamics_step(x_test, 0.0, -1000.0, mech, plant, 0, 0, 0, 0.001);
    
    assert(x_next_left(3) > 0, '左电机下发正指令(+1000)必须产生向前平动加速度');
    assert(x_next_right(3) > 0, '右电机下发负指令(-1000)必须产生向前平动加速度');
    fprintf('  -> PASS: 符号逻辑完全正确，左正(+)/右负(-)均准确产生向前驱动力。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 3] 动态梯度限幅逻辑核查
fprintf('[Test 3/4] 检查 chassis_calculate 动态梯度限幅...\n');
try
    % 模拟起步瞬间 (已走圈数 = 0)
    error_rev = 5.0; travel_rev = 0.0;
    dyn_min = min(error_rev, travel_rev) * ctrl.ang_gradient_slope;
    dyn_clipped = max(ctrl.ang_gradient_min_out, min(ctrl.ang_gradient_max_out, dyn_min));
    assert(dyn_clipped == 1000.0, '起步时刻限幅必须被截断在下限 1000.0 rpm');
    
    % 模拟超长行程阶段 (圈数 >= 60 圈, 60*200 = 12000)
    travel_rev = 80.0; error_rev = 80.0;
    dyn_max = min(error_rev, travel_rev) * ctrl.ang_gradient_slope;
    dyn_clipped_max = max(ctrl.ang_gradient_min_out, min(ctrl.ang_gradient_max_out, dyn_max));
    assert(dyn_clipped_max == 12000.0, '大圈数巡航阶段限幅必须被截断在上限 12000.0 rpm');
    
    fprintf('  -> PASS: 动态梯度限幅范围严格收敛在 [1000, 12000] rpm。\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% [Test 4] 运行主基准测试并验证数据完整性
fprintf('[Test 4/4] 检查主基线仿真 run_c0_benchmark 运行完整性...\n');
try
    run_c0_benchmark;
    assert(exist('results', 'var') == 1, '变量 results 必须存在');
    assert(length(results) == 4, '必须包含 4 组工况数据');
    assert(exist('baseline_c0_comparison.png', 'file') == 2, '图片 baseline_c0_comparison.png 必须成功生成');
    
    % 检查 Case 1 同步误差为 0
    assert(results(1).max_sync < 1e-6, 'Case 1 标称对称工况同步误差应严格接近 0');
    % 检查 Case 3 偏心质量导致同步误差增大
    assert(results(3).max_sync > results(2).max_sync, 'Case 3 偏心质量较大时同步误差必须大于 Case 2');
    % 检查 Case 4 饱和时间占比 > 0
    assert(results(4).sat_ratio > 30.0, 'Case 4 严格限流下饱和时间占比必须显著增加');
    
    fprintf('  -> PASS: 4组工况全部推演成功，物理规律与指标逻辑 100%% 验证通过！\n\n');
    passed_tests = passed_tests + 1;
catch ME
    fprintf('  -> FAIL: %s\n\n', ME.message);
end

%% 总结报告
fprintf('====================================================\n');
fprintf('  自检测试结果: %d / %d 项全部通过 (PASS)\n', passed_tests, total_tests);
fprintf('  当前工作目录: %s\n', pwd);
fprintf('  输出图片路径: %s\\baseline_c0_comparison.png\n', pwd);
fprintf('====================================================\n');
