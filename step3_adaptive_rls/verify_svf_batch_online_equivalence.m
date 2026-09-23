%% VERIFY_SVF_BATCH_ONLINE_EQUIVALENCE.M - 批处理 SVF 与在线逐点递推 SVF 点对点数值等价性检验
% =========================================================================
% 功能说明:
% 1. 加载 Phase 0 两组工况 (r070 与 r130) 的全部真实测量序列 (yL, yR, iL, iR)
% 2. 分别执行:
%    - 离线批处理: build_step3b_regression (基于 MATLAB 内置 filter)
%    - 在线逐点递推: rls_filter_svf_step3b (基于 Direct Form II Transposed 状态机)
% 3. 全程逐点逐通道对比:
%    - alpha_f, alpha_dot_f, alpha_ddot_f, yG_f, Tfric_f, Treq_f, phi_f, y_f
% 4. 严格统计并报告:
%    - 全程 (0 <= t <= 4.0s) 最大绝对残差
%    - 初始瞬态阶段 (0 <= t < 0.5s) 最大绝对残差 (严禁隐藏，单独报告)
%    - 评估工作区间 (t >= 0.5s) 最大绝对残差
% 5. 严格断言指标: 全程误差与稳定段误差均满足 max_err < 1e-12
% =========================================================================

function [is_pass, max_errors] = verify_svf_batch_online_equivalence()
    script_dir = fileparts(mfilename('fullpath'));
    output_dir = fullfile(script_dir, '..');
    
    addpath(fullfile(output_dir, 'common'));
    addpath(fullfile(output_dir, 'step1_baseline_c0'));
    addpath(fullfile(output_dir, 'step2_advanced_controllers'));
    addpath(fullfile(output_dir, 'step3_adaptive_rls'));
    
    fprintf('=========================================================================\n');
    fprintf('     STEP 3B: 批处理 SVF 与在线递推 SVF 点对点数值等价性验证 (两组工况)    \n');
    fprintf('=========================================================================\n\n');
    
    file_r070 = fullfile(script_dir, 'data_step3b_phase0_r070.mat');
    file_r130 = fullfile(script_dir, 'data_step3b_phase0_r130.mat');
    
    assert(isfile(file_r070), '未找到工况 A 数据文件: %s', file_r070);
    assert(isfile(file_r130), '未找到工况 B 数据文件: %s', file_r130);
    
    d070 = load(file_r070);
    d130 = load(file_r130);
    
    datasets = {d070, d130};
    case_tags = {'r070 (Delta_Kf < 0)', 'r130 (Delta_Kf > 0)'};
    modes = {'ideal', 'quantized'};
    
    channels = {'alpha_f', 'alpha_dot_f', 'alpha_ddot_f', 'yG_f', ...
                'Tfric_f', 'Treq_f', 'phi_f', 'y_f'};
            
    max_errors = struct();
    for ch = 1:length(channels)
        max_errors.(channels{ch}) = struct('all', 0.0, 'transient', 0.0, 'steady', 0.0);
    end
    
    is_pass = true;
    
    for c = 1:2
        ds = datasets{c};
        case_tag = case_tags{c};
        
        for m = 1:length(modes)
            mode = modes{m};
            fprintf('-------------------------------------------------------------------------\n');
            fprintf('>>> 正在验证: 工况 %s | 传感器通道: %s\n', case_tag, mode);
            fprintf('-------------------------------------------------------------------------\n');
            
            if strcmp(mode, 'ideal')
                yL = ds.yL_ideal;
                yR = ds.yR_ideal;
            else
                yL = ds.yL_quant;
                yR = ds.yR_quant;
            end
            iL = ds.iL_actual;
            iR = ds.iR_actual;
            N  = length(ds.t);
            
            % 1. 离线批处理
            reg_batch = build_step3b_regression( ...
                yL, yR, iL, iR, ds.dt, ds.mech, ds.plant, ds.Kf_mean, mode);
            
            % 2. 在线逐点递推
            flt_online = rls_filter_svf_step3b(10.0, ds.dt);
            
            alpha_f_online      = zeros(N, 1);
            alpha_dot_f_online  = zeros(N, 1);
            alpha_ddot_f_online = zeros(N, 1);
            yG_f_online         = zeros(N, 1);
            Tfric_f_online      = zeros(N, 1);
            Treq_f_online       = zeros(N, 1);
            phi_f_online        = zeros(N, 1);
            y_f_online          = zeros(N, 1);
            
            for k = 1:N
                [flt_online, phi_k, y_k, diag_k] = flt_online.step( ...
                    yL(k), yR(k), iL(k), iR(k), ds.mech, ds.plant, ds.Kf_mean);
                
                phi_f_online(k)        = phi_k;
                y_f_online(k)          = y_k;
                alpha_f_online(k)      = diag_k.alpha_f;
                alpha_dot_f_online(k)  = diag_k.alpha_dot_f;
                alpha_ddot_f_online(k) = diag_k.alpha_ddot_f;
                yG_f_online(k)         = diag_k.yG_f;
                Tfric_f_online(k)      = diag_k.Tfric_f;
                Treq_f_online(k)       = diag_k.Treq_f;
            end
            
            % 3. 区间划分 (瞬态区间 0~0.5s, 稳定区间 >=0.5s)
            trans_mask  = (ds.t < 0.5);
            steady_mask = (ds.t >= 0.5);
            
            fprintf('  %-14s | 全程最大误差 (0~4s) | 瞬态最大误差 (<0.5s) | 稳定段最大误差 (>=0.5s)\n', '回归通道名称');
            fprintf('  ---------------+--------------------+--------------------+--------------------\n');
            
            online_data = struct( ...
                'alpha_f', alpha_f_online, ...
                'alpha_dot_f', alpha_dot_f_online, ...
                'alpha_ddot_f', alpha_ddot_f_online, ...
                'yG_f', yG_f_online, ...
                'Tfric_f', Tfric_f_online, ...
                'Treq_f', Treq_f_online, ...
                'phi_f', phi_f_online, ...
                'y_f', y_f_online);
            
            for ch = 1:length(channels)
                c_name = channels{ch};
                diff_vec = abs(reg_batch.(c_name) - online_data.(c_name));
                
                err_all    = max(diff_vec);
                err_trans  = max(diff_vec(trans_mask));
                err_steady = max(diff_vec(steady_mask));
                
                max_errors.(c_name).all       = max(max_errors.(c_name).all, err_all);
                max_errors.(c_name).transient = max(max_errors.(c_name).transient, err_trans);
                max_errors.(c_name).steady    = max(max_errors.(c_name).steady, err_steady);
                
                fprintf('  %-14s |      %.2e      |      %.2e      |      %.2e\n', ...
                    c_name, err_all, err_trans, err_steady);
                
                % 严格断言
                if err_all > 1e-12 || err_steady > 1e-12
                    is_pass = false;
                end
            end
            fprintf('\n');
        end
    end
    
    fprintf('=========================================================================\n');
    fprintf('综合汇总所有工况与传感器通道的最大绝对残差:\n');
    for ch = 1:length(channels)
        c_name = channels{ch};
        fprintf('  %-14s: 全程 = %.2e | 瞬态 = %.2e | 稳定段 = %.2e (阈值: 1e-12)\n', ...
            c_name, max_errors.(c_name).all, max_errors.(c_name).transient, max_errors.(c_name).steady);
    end
    
    assert(is_pass, '批处理 SVF 与在线递推 SVF 点对点数值等价性检验失败！残差超过 1e-12！');
    fprintf('\n>>> 批处理 SVF 与在线递推 SVF 100%% PASS！全时间段所有回归通道数值完全等价！\n');
    fprintf('=========================================================================\n\n');
end
