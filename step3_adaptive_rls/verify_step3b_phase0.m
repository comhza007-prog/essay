%% VERIFY_STEP3B_PHASE0.M - Step 3B Phase 0 可辨识性预分析与敏感性独立评测脚本
% =========================================================================
% 功能说明:
% 1. 验证 Test 1: 独立动力学代数自洽性检验 (基于 gantry_dynamics_deriv_step3b 与 yL/yR 传感器通道重构)
% 2. 真实滤波信号下的持续激励 (PE) 阈值标定扫描:
%    - 扫描 sigma_PE,th in [1, 10, 50, 100] count*m
%    - 统计误激活率、漏激活率、有效窗口占比、Delta_Kf 估计误差及残差 RMS
% 3. 三组结构参数独立敏感性评测:
%    - K_alpha +/- 20% -> Delta_Kf 偏差与误差传递增益
%    - B_alpha +/- 20% -> Delta_Kf 偏差与误差传递增益
%    - J0      +/- 20% -> Delta_Kf 偏差与误差传递增益
% 4. 导出 Phase 0 综合量化评测指标 CSV: step3b_phase0_preanalysis.csv
% =========================================================================

function verify_step3b_phase0()
    script_dir = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, '..');
    
    addpath(fullfile(output_dir, 'step1_baseline_c0'));
    addpath(fullfile(output_dir, 'step2_advanced_controllers'));
    addpath(fullfile(output_dir, 'step3_adaptive_rls'));
    
    fprintf('=========================================================================\n');
    fprintf('           STEP 3B PHASE 0: 可辨识性预分析与参数独立敏感性评测           \n');
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
    csv_header = {'Case', 'Test_Item', 'Param_Perturbation', 'PE_Threshold_count_m', ...
                  'Active_Ratio_pct', 'False_Activation_pct', 'Miss_Activation_pct', ...
                  'Delta_Kf_True', 'Delta_Kf_Est', 'Rel_Error_pct', 'Transfer_Gain', 'Res_RMS_Nm'};
              
    %% ---------------------------------------------------------------------
    %% 1. Test 1: 独立动力学与传感器通道重构自洽性检验
    %% ---------------------------------------------------------------------
    fprintf('>>> [1/3] 正在执行 Test 1: 独立动力学代数自洽性检验 ...\n');
    for c = 1:2
        ds = datasets{c};
        Le = ds.Le;
        % 从左右传感器通道独立重构 yG, alpha
        yG_rec = 0.5 * (ds.yL + ds.yR);
        alpha_rec = (ds.yR - ds.yL) / Le;
        
        diff_yG = max(abs(yG_rec - ds.yG));
        diff_alpha = max(abs(alpha_rec - ds.alpha));
        assert(diff_yG < 1e-12, 'yG 独立传感器重构误差超限');
        assert(diff_alpha < 1e-12, 'alpha 独立传感器重构误差超限');
        
        % 随机抽取 5 个时态点，通过底层统一微分函数验证角加速度与力矩平衡
        sample_indices = round(linspace(100, length(ds.t)-100, 5));
        for idx = sample_indices
            x_k = [ds.yG(idx); ds.alpha(idx); ds.vL(idx) + 0.5*Le*ds.alpha_dot(idx); ds.alpha_dot(idx)];
            [~, det_k] = gantry_dynamics_deriv_step3b(...
                x_k, ds.iL_actual(idx), ds.iR_actual(idx), ds.mech, ds.plant, ...
                0, 0, 0, ds.Kf_L, ds.Kf_R);
            
            % 检验求出的角加速度与记录值一致
            assert(abs(det_k.alpha_ddot - ds.alpha_ddot(idx)) < 1e-12, 'alpha_ddot 动力学重构不匹配');
            
            % 独立计算 T_req 与代数恒等
            T_req_k = ds.mech.J_alpha_nom * det_k.alpha_ddot + ds.plant.B_alpha * ds.alpha_dot(idx) ...
                      + ds.plant.K_alpha * ds.alpha(idx) + det_k.T_fric_alpha;
            y_Delta_k = -T_req_k - 0.5 * Le * ds.Kf_mean * (ds.iL_actual(idx) + ds.iR_actual(idx));
            phi_Delta_k = 0.25 * Le * (ds.iL_actual(idx) - ds.iR_actual(idx));
            
            alg_res_k = abs(y_Delta_k - phi_Delta_k * ds.Delta_Kf_true);
            assert(alg_res_k < 1e-12, '采样点独立代数残差超限');
        end
        fprintf('    工况 %s: 独立传感器重构与动力学校核 100%% 通过 (残差 < 1e-12)\n', case_tags{c});
    end
    fprintf('    Test 1 动力学与传感器通道自洽性验证 PASS！\n\n');
    
    %% ---------------------------------------------------------------------
    %% 2. 4 阶因果 SVF 滤波器构造与真实滤波信号获取
    %% ---------------------------------------------------------------------
    fprintf('>>> [2/3] 正在执行真实滤波信号获取与 PE 门控阈值标定扫描 ...\n');
    fc = 10.0; % 截止频率 10 Hz
    dt = d070.dt;
    wc = 2.0 * pi * fc;
    poly_den = [1.0, 2.61312592975275 * wc, 3.41421356237310 * (wc^2), ...
                2.61312592975275 * (wc^3), wc^4];
    sys_w0 = c2d(tf(wc^4, poly_den), dt, 'tustin');
    [num_w0, den_w0] = tfdata(sys_w0, 'v');
    
    pe_thresholds = [1.0, 10.0, 50.0, 100.0]; % [count*m]
    Nw = 200; % 200ms 滑动窗口
    
    for c = 1:2
        ds = datasets{c};
        case_tag = case_tags{c};
        
        % 滤波回归基底与可测输出
        phi_raw = ds.phi_Delta_T;
        y_raw   = ds.y_Delta_T;
        
        phi_f = filter(num_w0, den_w0, phi_raw);
        y_f   = filter(num_w0, den_w0, y_raw);
        
        % 计算滑动窗 RMS
        phi_rms = zeros(size(phi_f));
        phi_sq = phi_f .^ 2;
        cum_sq = cumsum(phi_sq);
        for k = 1:length(phi_f)
            if k <= Nw
                phi_rms(k) = sqrt(cum_sq(k) / k);
            else
                phi_rms(k) = sqrt((cum_sq(k) - cum_sq(k - Nw)) / Nw);
            end
        end
        
        % 稳态激励分析区间 (忽略前 0.3s 滤波器建立阶跃)
        eval_mask = (ds.t >= 0.3);
        
        fprintf('  ------------------------------------------------------------------\n');
        fprintf('  工况 %s (Delta_Kf_true = %+.7f N/count):\n', case_tag, ds.Delta_Kf_true);
        fprintf('  滤波基底 phi_f 峰值: %.1f count*m, 全程 RMS: %.1f count*m\n', ...
            max(abs(phi_f(eval_mask))), rms(phi_f(eval_mask)));
        fprintf('  PE阈值(count*m) | 激活率(%%) | 误激活(%%) | 漏激活(%%) | 估计值(N/count) | 相对误差(%%) | 残差RMS(Nm)\n');
        
        % 明确物理分段定义:
        % 强激励区间: t in [0.4, 2.3] s
        active_ref = (ds.t >= 0.4 & ds.t <= 2.3);
        % 停顿死区: t in [3.0, 4.0] s (留出 0.3s 给滤波衰减)
        dwell_ref  = (ds.t >= 3.0 & ds.t <= 4.0);
        
        for th = pe_thresholds
            active_mask = eval_mask & (phi_rms >= th);
            active_ratio = mean(active_mask(eval_mask)) * 100.0;
            
            % 停顿死区误激活率: 停顿区间内 phi_rms >= th 的比例 (越低越安全，应为 0%)
            false_act_rate = mean(phi_rms(dwell_ref) >= th) * 100.0;
            
            % 激励段漏激活率: 强激励区间内 phi_rms < th 的比例 (越低越好，应为 0%)
            miss_act_rate = mean(phi_rms(active_ref) < th) * 100.0;
            
            % 响应切断延迟: 从 t=2.7s (电流归零) 到门控断开的时间 (ms)
            idx_cutoff = find(ds.t >= 2.7 & phi_rms < th, 1);
            if isempty(idx_cutoff)
                cutoff_delay_ms = NaN;
            else
                cutoff_delay_ms = (ds.t(idx_cutoff) - 2.7) * 1000;
            end
            
            % 在有效激活窗内的最小二乘估计
            phi_act = phi_f(active_mask);
            y_act   = y_f(active_mask);
            
            if ~isempty(phi_act)
                Delta_Kf_est = (phi_act' * y_act) / (phi_act' * phi_act);
                rel_err_pct = abs(Delta_Kf_est - ds.Delta_Kf_true) / abs(ds.Delta_Kf_true) * 100.0;
                res_rms = sqrt(mean((y_act - phi_act * Delta_Kf_est).^2));
            else
                Delta_Kf_est = NaN;
                rel_err_pct = NaN;
                res_rms = NaN;
            end
            
            fprintf('       %3.0f        |   %5.1f   |   %5.1f   |   %5.1f   |   %+11.7f   |    %5.2f%%    |  %.2e\n', ...
                th, active_ratio, false_act_rate, miss_act_rate, Delta_Kf_est, rel_err_pct, res_rms);
            
            csv_rows{end+1} = {case_tag, 'PE_Threshold_Scan', 'Nominal', th, ...
                active_ratio, false_act_rate, miss_act_rate, ds.Delta_Kf_true, Delta_Kf_est, rel_err_pct, 1.0, res_rms};
        end
    end
    fprintf('\n');
    
    %% ---------------------------------------------------------------------
    %% 3. 三组结构参数独立敏感性评测 (K_alpha, B_alpha, J0 分别 +/- 20%)
    %% ---------------------------------------------------------------------
    fprintf('>>> [3/3] 正在执行三组结构参数独立敏感性评测 (K_alpha, B_alpha, J0 分别 +/-20%%) ...\n');
    % 使用选定的合理基准 PE 阈值: sigma_PE,th = 10 count*m
    th_baseline = 10.0;
    
    params_to_test = {'K_alpha', 'B_alpha', 'J0'};
    deltas = [-0.20, +0.20];
    
    for c = 1:2
        ds = datasets{c};
        case_tag = case_tags{c};
        
        fprintf('  ==================================================================\n');
        fprintf('  结构参数独立敏感性: 工况 %s\n', case_tag);
        fprintf('  扰动参数  |  摄动比例  | Delta_Kf_真值 | Delta_Kf_估计 | 估计偏差(%%) | 误差传递增益\n');
        
        for p_idx = 1:length(params_to_test)
            param_name = params_to_test{p_idx};
            
            for d_idx = 1:length(deltas)
                delta_pct = deltas(d_idx);
                
                % 构造摄动后的名义参数
                J0_pert = ds.mech.J_alpha_nom;
                Ba_pert = ds.plant.B_alpha;
                Ka_pert = ds.plant.K_alpha;
                
                if strcmp(param_name, 'K_alpha')
                    Ka_pert = ds.plant.K_alpha * (1.0 + delta_pct);
                elseif strcmp(param_name, 'B_alpha')
                    Ba_pert = ds.plant.B_alpha * (1.0 + delta_pct);
                elseif strcmp(param_name, 'J0')
                    J0_pert = ds.mech.J_alpha_nom * (1.0 + delta_pct);
                end
                
                % 用摄动参数重新计算名义所需阻抗力矩与可测回归输出
                T_req_pert = J0_pert * ds.alpha_ddot + Ba_pert * ds.alpha_dot ...
                             + Ka_pert * ds.alpha + ds.T_fric;
                y_raw_pert = -T_req_pert - 0.5 * ds.Le * ds.Kf_mean * (ds.iL_actual + ds.iR_actual);
                
                % 因果 SVF 滤波
                y_f_pert = filter(num_w0, den_w0, y_raw_pert);
                phi_f    = filter(num_w0, den_w0, ds.phi_Delta_T);
                
                % 计算滑动窗 RMS 并应用门控
                phi_rms = zeros(size(phi_f));
                cum_sq = cumsum(phi_f .^ 2);
                for k = 1:length(phi_f)
                    if k <= Nw
                        phi_rms(k) = sqrt(cum_sq(k) / k);
                    else
                        phi_rms(k) = sqrt((cum_sq(k) - cum_sq(k - Nw)) / Nw);
                    end
                end
                
                active_mask = (ds.t >= 0.3) & (phi_rms >= th_baseline);
                phi_act = phi_f(active_mask);
                y_act   = y_f_pert(active_mask);
                
                Delta_Kf_est = (phi_act' * y_act) / (phi_act' * phi_act);
                bias_pct = (Delta_Kf_est - ds.Delta_Kf_true) / ds.Delta_Kf_true * 100.0;
                transfer_gain = (bias_pct / 100.0) / delta_pct;
                res_rms = sqrt(mean((y_act - phi_act * Delta_Kf_est).^2));
                
                fprintf('  %-9s |   %+5.1f%%   |  %+11.7f  |  %+11.7f  |   %+6.2f%%   |    %5.3f\n', ...
                    param_name, delta_pct*100, ds.Delta_Kf_true, Delta_Kf_est, bias_pct, transfer_gain);
                
                csv_rows{end+1} = {case_tag, 'Sensitivity_Test', sprintf('%s_%+d%%', param_name, round(delta_pct*100)), ...
                    th_baseline, mean(active_mask(ds.t>=0.3))*100, 0.0, 0.0, ds.Delta_Kf_true, Delta_Kf_est, bias_pct, transfer_gain, res_rms};
            end
        end
    end
    fprintf('\n');
    
    %% ---------------------------------------------------------------------
    %% 4. 保存结构化评测 CSV
    %% ---------------------------------------------------------------------
    csv_file = fullfile(script_dir, 'step3b_phase0_preanalysis.csv');
    fid = fopen(csv_file, 'w');
    fprintf(fid, '%s\n', strjoin(csv_header, ','));
    for i = 1:length(csv_rows)
        row = csv_rows{i};
        fprintf(fid, '%s,%s,%s,%.1f,%.2f,%.2f,%.2f,%.7e,%.7e,%.4f,%.4f,%.4e\n', ...
            row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, row{9}, row{10}, row{11}, row{12});
    end
    fclose(fid);
    fprintf('>>> 结构化评测指标已成功导出至: %s\n\n', csv_file);
    fprintf('=========================================================================\n');
    fprintf('            STEP 3B PHASE 0 预分析与敏感性评测全部执行完毕               \n');
    fprintf('=========================================================================\n');
end
