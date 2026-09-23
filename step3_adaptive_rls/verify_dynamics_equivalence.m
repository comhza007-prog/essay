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
    fprintf('          正在执行动力学等价性检验: 1000 组随机工况扫描          \n');
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
        
        % 1. 原 Step 2 独立原生解析计算 (对照组原始代码基准)
        dx_old = orig_step2_eval_deriv(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
        xnext_old = orig_step2_rk4(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R);
        
        % 2. 公共底层动力学核计算 (实验组公共函数)
        [dx_new, ~] = gantry_dynamics_deriv(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
        xnext_new = gantry_dynamics_step_step3b(x, iL, iR, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R);
        
        err_dx = max(abs(dx_old - dx_new));
        err_xnext = max(abs(xnext_old - xnext_new));
        
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
    
    fprintf('>>> 动力学等价性测试 100%% PASS！彻底消除代码复写风险！\n');
    fprintf('=========================================================================\n\n');
end

%% 原 Step 2 原始解析微分函数基准 (严格内联备份以作等价对照)
function dxdt = orig_step2_eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R)
    yG        = x(1);
    alpha     = x(2);
    yG_dot    = x(3);
    alpha_dot = x(4);
    Le = mech.Le;
    
    vL = yG_dot - 0.5 * Le * alpha_dot;
    vR = yG_dot + 0.5 * Le * alpha_dot;
    FL =  Kf_L * iL_cmd;
    FR = -Kf_R * iR_cmd;
    
    bL = plant.b_nom;
    fcL = plant.fc_nom;
    bR = plant.b_nom * (1.0 + delta_fric);
    fcR = plant.fc_nom * (1.0 + delta_fric);
    
    F_fric_L = bL * vL + fcL * tanh(100.0 * vL);
    F_fric_R = bR * vR + fcR * tanh(100.0 * vR);
    
    M_tot = mech.mG_nom + delta_m;
    J_tot = mech.J_alpha_nom + delta_m * (d_load^2);
    coupling_m = delta_m * d_load;
    
    F_total = (FL + FR) - (F_fric_L + F_fric_R);
    Tau_total = (0.5 * Le) * (FR - FL) - (0.5 * Le) * (F_fric_R - F_fric_L) ...
                - plant.K_alpha * alpha - plant.B_alpha * alpha_dot;
            
    detM = M_tot * J_tot - (coupling_m^2);
    yG_ddot    = ( J_tot * F_total - coupling_m * Tau_total) / detM;
    alpha_ddot = (-coupling_m * F_total + M_tot * Tau_total) / detM;
    
    dxdt = [yG_dot; alpha_dot; yG_ddot; alpha_ddot];
end

%% 原 Step 2 原始 RK4 单步推演基准
function x_next = orig_step2_rk4(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, dt, Kf_L, Kf_R)
    k1 = orig_step2_eval_deriv(x, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k2 = orig_step2_eval_deriv(x + 0.5 * dt * k1, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k3 = orig_step2_eval_deriv(x + 0.5 * dt * k2, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    k4 = orig_step2_eval_deriv(x + dt * k3, iL_cmd, iR_cmd, mech, plant, delta_m, d_load, delta_fric, Kf_L, Kf_R);
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end
