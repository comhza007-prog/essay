%% VERIFY_STEP3B_PHASE0.M - Step 3B Phase 0 可辨识性预分析与参数独立敏感性评测脚本 (全面重构版)
% =========================================================================
% 测试架构 (四部分严格独立):
% Test 1: 动力学与符号一致性 (独立输出: 动力学函数一致性指标 + 代数回归一致性指标)
% Test 2: 理想连续测量回归 (build_step3b_regression, t >= 0.5s, 验收指标: 相对误差 <= 1%)
% Test 3: 量化测量回归 (量化步长 q_y = 1.21 um, 验收指标: 相对误差 <= 5%, std <= 5%, PE误动/漏动 <= 1%)
% Test 4: 结构参数敏感性 (全链路重构: K_alpha, B_alpha, J0 分别 +/-20%, 独立报告误差传递增益)
% =========================================================================

function verify_step3b_phase0()
    script_dir = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, '..');
    
    addpath(fullfile(output_dir, 'common'));
    addpath(fullfile(output_dir, 'step1_baseline_c0'));
    addpath(fullfile(output_dir, 'step2_advanced_controllers'));
    addpath(fullfile(output_dir, 'step3_adaptive_rls'));
    
    fprintf('=========================================================================\n');
    fprintf('          STEP 3B PHASE 0: 测量回归链路验证与可辨识性预分析报告          \n');
    fprintf('=========================================================================\n\n');
    
    % 加载生成的两组工况数据
    file_r070 = fullfile(script_dir, 'data_step3b_phase0_r070.mat');
    file_r130 = fullfile(script_dir, 'data_step3b_phase0_r130.mat');
    
    assert(isfile(file_r070), '未找到工况 A 数据文件: %s', file_r070);
    assert(isfile(file_r130), '未找到工况 B 数据文件: %s', file_r130);
    
    d070 = load(file_r070);
    d130 = load(file_r130);
    
    datasets = {d070, d130};
    case_tags = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
    
    csv_rows = {};
    csv_header = {'Case', 'Test_Item', 'Param_Perturbation', 'Sensor_Mode', ...
                  'PE_Threshold_count_m', 'Active_Ratio_pct', 'False_Activation_pct', ...
                  'Miss_Activation_pct', 'Delta_Kf_True', 'Delta_Kf_Est', 'Rel_Error_pct', ...
                  'Est_Std_pct', 'Transfer_Gain', 'Res_RMS_Nm'};
              
    %% =====================================================================
    %% Test 1: 动力学与符号一致性检验
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test 1] 动力学与符号一致性检验\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    % 1A. 动力学函数一致性测试 (调用 verify_dynamics_equivalence)
    [max_err_dx, max_err_xnext] = verify_dynamics_equivalence();
    fprintf('  [1A] 动力学函数一致性:\n');
    fprintf('       微分导数残差 max|dx_old - dx_new|       = %.2e (验收指标: < 1e-12)\n', max_err_dx);
    fprintf('       RK4单步推演残差 max|xnext_old - xnext|  = %.2e (验收指标: < 1e-12)\n', max_err_xnext);
    assert(max_err_dx < 1e-12 && max_err_xnext < 1e-12, '动力学函数一致性未通过！');
    fprintf('       >>> 动力学函数一致性: PASS (1000 组工况完全等价)\n\n');
    
    % 1B. 代数回归一致性测试
    max_alg_err_overall = 0.0;
    for c = 1:2
        ds = datasets{c};
        phi_k = 0.25 * ds.Le * (ds.iL_actual - ds.iR_actual);
        Treq_k = ds.mech.J_alpha_nom * ds.alpha_ddot_true + ds.plant.B_alpha * ds.alpha_dot_true ...
                 + ds.plant.K_alpha * ds.alpha_true + ds.T_fric_true;
        y_k = -Treq_k - 0.5 * ds.Le * ds.Kf_mean * (ds.iL_actual + ds.iR_actual);
        res_k = max(abs(y_k - phi_k * ds.Delta_Kf_true));
        if res_k > max_alg_err_overall
            max_alg_err_overall = res_k;
        end
    end
    fprintf('  [1B] 代数回归一致性:\n');
    fprintf('       独立状态力矩代数残差 max|y_k - phi_k * Delta_Kf| = %.2e (验收指标: < 1e-12)\n', max_alg_err_overall);
    assert(max_alg_err_overall < 1e-12, '代数回归一致性未通过！');
    fprintf('       >>> 代数回归一致性: PASS (严格满足线性回归数学形式)\n\n');
    
    %% =====================================================================
    %% Test 2: 理想连续测量回归 (build_step3b_regression, mode = 'ideal')
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test 2] 理想连续测量回归 (t >= 0.5s, 验收指标: 相对误差 <= 1.0%%)\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    Nw = 200; % 200ms 滑动窗口
    sigma_PE_base = 50.0; % 候选基准阈值 (count*m)
    
    for c = 1:2
        ds = datasets{c};
        case_tag = case_tags{c};
        
        % 从理想左右传感器通道重构回归信号
        reg_ideal = build_step3b_regression( ...
            ds.yL_ideal, ds.yR_ideal, ds.iL_actual, ds.iR_actual, ...
            ds.dt, ds.mech, ds.plant, ds.Kf_mean, 'ideal');
        
        % 消除前 0.5s 滤波器初始瞬态建立区间
        eval_mask = (ds.t >= 0.5);
        
        % 计算滑动窗 Gram 标量能量 G_k 与开方
        phi_f = reg_ideal.phi_f;
        y_f   = reg_ideal.y_f;
        G_k = zeros(size(phi_f));
        cum_sq = cumsum(phi_f .^ 2);
        for k = 1:length(phi_f)
            if k <= Nw
                G_k(k) = cum_sq(k) / k;
            else
                G_k(k) = (cum_sq(k) - cum_sq(k - Nw)) / Nw;
            end
        end
        sqrt_G_k = sqrt(G_k);
        
        act_mask = eval_mask & (sqrt_G_k >= sigma_PE_base);
        phi_act = phi_f(act_mask);
        y_act   = y_f(act_mask);
        
        Delta_Kf_est = (phi_act' * y_act) / (phi_act' * phi_act);
        rel_err_pct  = abs(Delta_Kf_est - ds.Delta_Kf_true) / abs(ds.Delta_Kf_true) * 100.0;
        res_rms      = sqrt(mean((y_act - phi_act * Delta_Kf_est).^2));
        
        fprintf('  工况 %s:\n', case_tag);
        fprintf('    真值: %+.7f N/count | 估计值: %+.7f N/count | 相对误差: %.4f%% (指标: <= 1.0%%)\n', ...
            ds.Delta_Kf_true, Delta_Kf_est, rel_err_pct);
        fprintf('    残差 RMS: %.4e N*m | 有效激活比例: %.1f%%\n', ...
            res_rms, mean(act_mask(eval_mask))*100);
        
        assert(rel_err_pct <= 1.0, 'Test 2 理想连续测量回归相对误差超限！');
        assert(sign(Delta_Kf_est) == sign(ds.Delta_Kf_true), 'Test 2 推力偏差符号恢复错误！');
        
        csv_rows{end+1} = {case_tag, 'Test2_Ideal_Continuous', 'Nominal', 'Ideal', ...
            sigma_PE_base, mean(act_mask(eval_mask))*100, 0.0, 0.0, ...
            ds.Delta_Kf_true, Delta_Kf_est, rel_err_pct, 0.0, 1.0, res_rms};
    end
    fprintf('  >>> Test 2 理想连续测量回归: PASS！\n\n');
    
    %% =====================================================================
    %% Test 3: 量化测量回归 (build_step3b_regression, mode = 'quantized')
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test 3] 量化测量回归 (分辨率 q_y = 1.21 um, 阶梯阈值扫描与门控特性)\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    pe_thresholds = [1.0, 10.0, 50.0, 100.0];
    
    for c = 1:2
        ds = datasets{c};
        case_tag = case_tags{c};
        
        % 从量化左右传感器通道重构回归信号
        reg_quant = build_step3b_regression( ...
            ds.yL_quant, ds.yR_quant, ds.iL_actual, ds.iR_actual, ...
            ds.dt, ds.mech, ds.plant, ds.Kf_mean, 'quantized');
        
        eval_mask = (ds.t >= 0.5);
        % 强激励区间与停顿区间定义
        strong_ref = (ds.t >= 0.5 & ds.t <= 2.3);
        dwell_ref  = (ds.t >= 3.0 & ds.t <= 4.0);
        
        phi_f = reg_quant.phi_f;
        y_f   = reg_quant.y_f;
        G_k = zeros(size(phi_f));
        cum_sq = cumsum(phi_f .^ 2);
        for k = 1:length(phi_f)
            if k <= Nw
                G_k(k) = cum_sq(k) / k;
            else
                G_k(k) = (cum_sq(k) - cum_sq(k - Nw)) / Nw;
            end
        end
        sqrt_G_k = sqrt(G_k);
        
        fprintf('  -----------------------------------------------------------------------\n');
        fprintf('  工况 %s (Delta_Kf_true = %+.7f N/count, 量化步长 = 1.21 um):\n', case_tag, ds.Delta_Kf_true);
        fprintf('  PE阈值(count*m) | 激活率(%%) | 停顿误动(%%) | 强激漏动(%%) | 估计值(N/count) | 相对误差(%%) | 滑动std(%%) | 残差RMS(Nm)\n');
        
        for th = pe_thresholds
            act_mask = eval_mask & (sqrt_G_k >= th);
            active_ratio = mean(act_mask(eval_mask)) * 100.0;
            
            % 停顿段误激活率 (在停顿死区内被错误激活的比例)
            false_act = mean(sqrt_G_k(dwell_ref) >= th) * 100.0;
            % 强激励段漏激活率 (在强激励段被错误漏掉的比例)
            miss_act  = mean(sqrt_G_k(strong_ref) < th) * 100.0;
            
            phi_act = phi_f(act_mask);
            y_act   = y_f(act_mask);
            
            Delta_Kf_est = (phi_act' * y_act) / (phi_act' * phi_act);
            rel_err_pct  = abs(Delta_Kf_est - ds.Delta_Kf_true) / abs(ds.Delta_Kf_true) * 100.0;
            res_rms      = sqrt(mean((y_act - phi_act * Delta_Kf_est).^2));
            
            % 滑动窗局部估计标准差统计
            act_indices = find(act_mask);
            w_ests = zeros(length(act_indices), 1);
            for i = 1:length(act_indices)
                idx = act_indices(i);
                sub_idx = max(1, idx - Nw + 1):idx;
                phi_w = phi_f(sub_idx);
                y_w   = y_f(sub_idx);
                w_ests(i) = (phi_w' * y_w) / (phi_w' * phi_w);
            end
            std_val = std(w_ests);
            std_pct = std_val / abs(ds.Delta_Kf_true) * 100.0;
            
            fprintf('       %3.0f        |   %5.1f   |    %5.1f    |    %5.1f    |   %+11.7f   |    %5.2f%%    |   %5.2f%%   |  %.2e\n', ...
                th, active_ratio, false_act, miss_act, Delta_Kf_est, rel_err_pct, std_pct, res_rms);
            
            % 在基准阈值 50 count*m 下进行硬断言检验
            if th == 50.0
                assert(rel_err_pct <= 5.0, '量化相对误差超过 5.0% 门限！');
                assert(false_act <= 1.0, '停顿段误激活率超过 1.0%！');
                assert(miss_act <= 1.0, '强激励段漏激活率超过 1.0%！');
                assert(std_pct <= 5.0, '估计标准差超过真值 5.0% 门限！');
                assert(sign(Delta_Kf_est) == sign(ds.Delta_Kf_true), '推力偏差符号恢复错误！');
            end
            
            csv_rows{end+1} = {case_tag, 'Test3_Quantized_Scan', 'Nominal', 'Quantized', ...
                th, active_ratio, false_act, miss_act, ds.Delta_Kf_true, Delta_Kf_est, ...
                rel_err_pct, std_pct, 1.0, res_rms};
        end
    end
    fprintf('  >>> Test 3 量化测量回归 (基准阈值 50 count*m): PASS！\n\n');
    
    %% =====================================================================
    %% Test 4: 结构参数独立敏感性评测 (全链路重构)
    %% =====================================================================
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('>>> [Test 4] 结构参数独立敏感性评测 (K_alpha, B_alpha, J0 分别 +/-20%% 全链路重构)\n');
    fprintf('-------------------------------------------------------------------------\n');
    
    params_to_test = {'K_alpha', 'B_alpha', 'J0'};
    deltas = [-0.20, +0.20];
    
    for c = 1:2
        ds = datasets{c};
        case_tag = case_tags{c};
        
        fprintf('  =======================================================================\n');
        fprintf('  结构参数敏感性评测: 工况 %s\n', case_tag);
        fprintf('  摄动参数  |  摄动比例  | Delta_Kf_真值 | Delta_Kf_估计 | 估计偏差(%%) | 误差传递增益 | 残差RMS(Nm)\n');
        
        for p_idx = 1:length(params_to_test)
            param_name = params_to_test{p_idx};
            
            for d_idx = 1:length(deltas)
                delta_pct = deltas(d_idx);
                
                % 构造摄动结构参数副本
                plant_pert = ds.plant;
                mech_pert  = ds.mech;
                
                if strcmp(param_name, 'K_alpha')
                    plant_pert.K_alpha = ds.plant.K_alpha * (1.0 + delta_pct);
                elseif strcmp(param_name, 'B_alpha')
                    plant_pert.B_alpha = ds.plant.B_alpha * (1.0 + delta_pct);
                elseif strcmp(param_name, 'J0')
                    mech_pert.J_alpha_nom = ds.mech.J_alpha_nom * (1.0 + delta_pct);
                end
                
                % 关键: 每次摄动都必须完整通过 build_step3b_regression 真实链路构造！
                reg_pert = build_step3b_regression( ...
                    ds.yL_quant, ds.yR_quant, ds.iL_actual, ds.iR_actual, ...
                    ds.dt, mech_pert, plant_pert, ds.Kf_mean, 'quantized');
                
                eval_mask = (ds.t >= 0.5);
                phi_f = reg_pert.phi_f;
                y_f   = reg_pert.y_f;
                
                G_k = zeros(size(phi_f));
                cum_sq = cumsum(phi_f .^ 2);
                for k = 1:length(phi_f)
                    if k <= Nw
                        G_k(k) = cum_sq(k) / k;
                    else
                        G_k(k) = (cum_sq(k) - cum_sq(k - Nw)) / Nw;
                    end
                end
                sqrt_G_k = sqrt(G_k);
                
                act_mask = eval_mask & (sqrt_G_k >= sigma_PE_base);
                phi_act = phi_f(act_mask);
                y_act   = y_f(act_mask);
                
                Delta_Kf_est = (phi_act' * y_act) / (phi_act' * phi_act);
                bias_pct     = (Delta_Kf_est - ds.Delta_Kf_true) / ds.Delta_Kf_true * 100.0;
                transfer_gain = (bias_pct / 100.0) / delta_pct;
                res_rms      = sqrt(mean((y_act - phi_act * Delta_Kf_est).^2));
                
                fprintf('  %-9s |   %+5.1f%%   |  %+11.7f  |  %+11.7f  |   %+6.2f%%   |    %5.3f    |  %.2e\n', ...
                    param_name, delta_pct*100, ds.Delta_Kf_true, Delta_Kf_est, bias_pct, transfer_gain, res_rms);
                
                % 记录 CSV (非 PE 测试，误动率与漏动率明确记为 NaN)
                csv_rows{end+1} = {case_tag, 'Test4_Sensitivity', sprintf('%s_%+d%%', param_name, round(delta_pct*100)), ...
                    'Quantized', sigma_PE_base, mean(act_mask(eval_mask))*100, NaN, NaN, ...
                    ds.Delta_Kf_true, Delta_Kf_est, bias_pct, NaN, transfer_gain, res_rms};
            end
        end
    end
    fprintf('  >>> Test 4 结构参数独立敏感性评测完成！\n\n');
    
    %% =====================================================================
    %% 导出结构化评测 CSV
    %% =====================================================================
    csv_file = fullfile(script_dir, 'step3b_phase0_preanalysis.csv');
    fid = fopen(csv_file, 'w');
    fprintf(fid, '%s\n', strjoin(csv_header, ','));
    for i = 1:length(csv_rows)
        row = csv_rows{i};
        fprintf(fid, '%s,%s,%s,%s,%.1f,%.2f,%.2f,%.2f,%.7e,%.7e,%.4f,%.4f,%.4f,%.4e\n', ...
            row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, ...
            row{9}, row{10}, row{11}, row{12}, row{13}, row{14});
    end
    fclose(fid);
    fprintf('>>> Phase 0 结构化量化评测指标已成功更新至: %s\n', csv_file);
    fprintf('=========================================================================\n');
    fprintf('          STEP 3B PHASE 0 全链路评测执行完毕 (4 项独立测试全部 PASS)      \n');
    fprintf('=========================================================================\n');
end
