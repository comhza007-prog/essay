%% VERIFY_DYNAMICS_EQUIVALENCE.M - Step 2 动力学与公共动力学函数等价性单元测试
% =========================================================================
% 功能说明:
% 1. 随机生成 1000 组全面测试工况:
%    - 状态向量 x = [yG; alpha; yG_dot; alpha_dot]
%    - 控制电流指令 iL, iR (-16000 ~ 16000 counts)
%    - 偏载与偏心参数 delta_m (0 ~ 5 kg), d_load (-0.3 ~ 0.3 m)
%    - 导轨摩擦偏差 delta_fric (-0.5 ~ +0.5)
%    - 非对称推力系数 Kf_L, Kf_R (0.5*Kf ~ 1.5*Kf)
% 2. 分别通过原始 Step 2 解析公式与 common/gantry_dynamics_deriv 计算微分导数与 RK4 单步状态
% 3. 严格断言验收指标:
%    max(abs(dx_old - dx_new)) < 1e-12
%    max(abs(xnext_old - xnext_new)) < 1e-12
% =========================================================================

function [max_err_dx, max_err_xnext] = verify_dynamics_equivalence()
    script_dir = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, '..');
    
    addpath(fullfile(output_dir, 'tests', 'fixtures'));
    addpath(fullfile(output_dir, 'common'));
    addpath(fullfile(output_dir, 'step1_baseline_c0'));
    addpath(fullfile(output_dir, 'step2_advanced_controllers'));
    addpath(fullfile(output_dir, 'step3_adaptive_rls'));
    
    [~, mech, plant] = param_init();
    dt = 0.001;
    
    rng(20260923); % 固定随机种子保证结果可复现
    N_tests = 1000;
    
    max_err_dx = 0.0;
    max_err_xnext = 0.0;
    
    fprintf('=========================================================================\n');
    fprintf('   正在执行动力学等价性检验: 冻结旧版参考夹具 vs 公共动力学函数 (1000 组工况)   \n');
    fprintf('=========================================================================\n');
    
    for i = 1:N_tests
        % 随机生成物理状态与输入
        yG        = -0.5 + 1.0 * rand();        % -0.5 ~ +0.5 m
        alpha     = -0.01 + 0.02 * rand();      % -10 ~ +10 mrad
        yG_dot    = -1.5 + 3.0 * rand();        % -1.5 ~ +1.5 m/s
        alpha_dot = -0.1 + 0.2 * rand();        % -0.1 ~ +0.1 rad/s
        x = [yG; alpha; yG_dot; alpha_dot];
        
        iL = -16000 + 32000 * rand();
        iR = -16000 + 32000 * rand();
        
        delta_m    = 5.0 * rand();              % 0 ~ 5.0 kg
        d_load     = -0.3 + 0.6 * rand();       % -0.3 ~ +0.3 m
        delta_fric = -0.5 + 1.0 * rand();       % -50% ~ +50%
        Kf_L       = mech.Kf * (0.6 + 0.8 * rand());
        Kf_R       = mech.Kf * (0.6 + 0.8 * rand());
        
        % 1. 调用冻结的重构前 Step 2 原始参考夹具 (tests/fixtures/)
        dx_legacy    = gantry_dynamics_deriv_legacy(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
        xnext_legacy = gantry_dynamics_step_legacy(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R);
        
        % 2. 调用公共底层动力学核与单步推演函数 (common/ 与 step3_adaptive_rls/)
        [dx_common, ~] = gantry_dynamics_deriv(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
        xnext_common   = gantry_dynamics_step_step3b(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R);
        
        err_dx = max(abs(dx_legacy - dx_common));
        err_xnext = max(abs(xnext_legacy - xnext_common));
        
        if err_dx > max_err_dx
            max_err_dx = err_dx;
        end
        if err_xnext > max_err_xnext
            max_err_xnext = err_xnext;
        end
    end
    
    fprintf('1000 组工况扫描完成:\n');
    fprintf('  微分导数最大残差 max(abs(dx_old - dx_new))       = %.2e (阈值: 1e-12)\n', max_err_dx);
    fprintf('  RK4单步推演最大残差 max(abs(xnext_old - xnext_new)) = %.2e (阈值: 1e-12)\n', max_err_xnext);
    
    assert(max_err_dx < 1e-12, '微分导数等价性测试未通过！');
    assert(max_err_xnext < 1e-12, 'RK4单步推演等价性测试未通过！');
    
    fprintf('>>> 动力学等价性测试 100%% PASS！公共动力学函数与冻结参考夹具完全一致！\n');
    fprintf('=========================================================================\n\n');
end
