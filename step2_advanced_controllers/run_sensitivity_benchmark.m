%% RUN_SENSITIVITY_BENCHMARK.M - 参数敏感性与工况拓展全面评测脚本 (V2.1 审阅重构终极版)
% =========================================================================
% 本脚本执行 6 大维度的参数敏感性扫描与工况拓展仿真：
% 维度 1A: 偏载质量扫描 (delta_m = 0.0 ~ 6.0 kg, d = 0.28 m)
% 维度 1B: 偏心距扫描 (d = 0.00 ~ 0.30 m, delta_m = 3.0 kg, 限制在半跨距内)
% 维度 2:  左右导轨摩擦非对称性扫描 (delta_fric = 0% ~ 70%)
% 维度 3:  执行器限流能力分级扫描 (Imax = 3500 ~ 16000 counts)
% 维度 4:  平均推力能力恒定条件下的左右推力增益非对称扫描:
%          - 4A: 失配退化 (Blind Mismatch: 控制器用标称 Kf, 真实物理退化, 评估真实容错)
%          - 4B: 已知校准 (Matched Calibration: 控制器已知退化 Kf, 作为 oracle 对照)
% 维度 5:  动态抗饱和参数空间网格扫描 (K_aw = 0~80, lambda_aw = 5~80, 含 Kaw=0 基准)
% 维度 6:  往返分段载荷工况 (正向 4.5kg 载荷, 3.3s 静止保持期间切换为 0.5kg 空载, 3.5s 启动返程)
%
% 统一对比 2x2 完全因子消融矩阵:
%   1: C2a          (独立截断, 无 AW)
%   2: C2a-SyncAlloc (同步优先, 无 AW)
%   3: C2b          (独立截断, 动态 AW)
%   4: C2b-SyncAlloc (同步优先, 动态 AW - 提出复合方案)
% =========================================================================

clear; clc; close all;
diary('sensitivity_run.log');

fprintf('========================================================================================\n');
fprintf('        Step 2.6: 参数敏感性与工况拓展全面评测 (V2.1 审阅重构终极版)\n');
fprintf('========================================================================================\n\n');

%% 1. 环境初始化与标称参数
addpath(fullfile(pwd, '..', 'step1_baseline_c0'));
[ctrl, mech, plant] = param_init();

Ts = 0.001; % 1 ms 离散步长
traj = trajectory_reciprocating(7.0, Ts, 1.0, 0.6, 1.5);
N_steps = length(traj.t);
t_half_idx = find(traj.t >= traj.t_half, 1);

% C2 基础参数 (名义模型注入 5%~10% 失配，完全统一基准)
ctrl_c2_base.M_hat = 0.95 * diag([mech.mG_nom, mech.J_alpha_nom]);
ctrl_c2_base.B_hat = 0.90 * diag([plant.b_nom * 2.0, plant.B_alpha]);
ctrl_c2_base.K_hat = diag([0.0, plant.K_alpha]);
ctrl_c2_base.A_hat = diag([plant.fc_nom * 2.0, 0.0]);

ctrl_c2_base.Gamma = diag([30.0, 20.0]);
ctrl_c2_base.K1 = diag([45.0, 30.0]);
ctrl_c2_base.K2 = diag([12.0, 6.0]);
ctrl_c2_base.phi = [0.03; 0.03];
ctrl_c2_base.eps_v = 0.01;
ctrl_c2_base.eps_alpha = 0.01;
ctrl_c2_base.I_fw_limit = 16000.0;
ctrl_c2_base.enable_ff = true;

% C2b 抗饱和增益
ctrl_c2b_base = ctrl_c2_base;
ctrl_c2b_base.K_aw = 20.0 * eye(2);
ctrl_c2b_base.lambda_aw = 20.0;
ctrl_c2b_base.aw_mode = 'external';

% C1 传统交叉耦合参数
ctrl_c1_base = ctrl;
ctrl_c1_base.Kp_sync = 3.5;
ctrl_c1_base.Kd_sync = 0.08;
ctrl_c1_base.spd_max_out = 16000.0;

ctrl_names = {'C2a (独立截断)', 'C2a-SyncAlloc (同步优先)', 'C2b (独立截断)', 'C2b-SyncAlloc (提出复合方案)'};
ctrl_colors = {[0.2, 0.4, 0.8], [0.85, 0.45, 0.1], [0.2, 0.7, 0.3], [0.8, 0.1, 0.1]};
ctrl_lines = {'--', ':', '-.', '-'};
ctrl_widths = [1.6, 1.8, 1.6, 2.2];

csv_records = {};

%% =========================================================================
%% 维度 1A: 偏载质量扫描 (delta_m = 0.0, 1.5, 3.0, 4.5, 6.0 kg, 固定 d = 0.28 m)
%% =========================================================================
fprintf('>>> 正在执行 维度 1A: 偏载质量扫描 (delta_m = 0.0 ~ 6.0 kg) ...\n');
dm_list = [0.0, 1.5, 3.0, 4.5, 6.0];
dim1a_results = struct();
Imax_dim1 = 4500.0;

for m_idx = 1:length(dm_list)
    dm_val = dm_list(m_idx);
    env1a.delta_m = dm_val;
    env1a.d_load = 0.28;
    env1a.delta_fric = 0.30;
    env1a.Kf_L = mech.Kf;
    env1a.Kf_R = mech.Kf;
    
    for c_id = 1:4
        res = simulate_controller(c_id, traj, env1a, Imax_dim1, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, mech.Kf, mech.Kf, t_half_idx);
        dim1a_results(m_idx, c_id).res = res;
        dim1a_results(m_idx, c_id).dm = dm_val;
        dim1a_results(m_idx, c_id).c_id = c_id;
        
        csv_records{end+1} = {'Dim1A_EccentricMass', sprintf('dm=%.1fkg', dm_val), Imax_dim1, ctrl_names{c_id}, ...
            res.max_sync, res.rmse_sync, res.rmse_yG, res.overshoot_fwd, res.overshoot_rev, res.settle_fwd, res.settle_rev, ...
            res.int_delta_alloc, res.int_delta_ext, res.int_delta_tot, res.max_delta_FG, res.max_delta_Talpha_phys, res.max_delta_Talpha_model, ...
            res.max_z_aw_norm, res.tv_iL, res.tv_iR, res.tv_total, res.fwd_reverse_samples, res.rev_reverse_samples, NaN, NaN};
    end
end

%% =========================================================================
%% 维度 1B: 偏心距扫描 (d = 0.00, 0.14, 0.28, 0.30 m, 固定 delta_m = 3.0 kg)
%% =========================================================================
fprintf('>>> 正在执行 维度 1B: 偏心距扫描 (d = 0.00 ~ 0.30 m, 限制在半跨距内) ...\n');
d_list = [0.00, 0.14, 0.28, 0.30];
dim1b_results = struct();

for d_idx = 1:length(d_list)
    d_val = d_list(d_idx);
    env1b.delta_m = 3.0;
    env1b.d_load = d_val;
    env1b.delta_fric = 0.30;
    env1b.Kf_L = mech.Kf;
    env1b.Kf_R = mech.Kf;
    
    for c_id = 1:4
        res = simulate_controller(c_id, traj, env1b, Imax_dim1, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, mech.Kf, mech.Kf, t_half_idx);
        dim1b_results(d_idx, c_id).res = res;
        dim1b_results(d_idx, c_id).d = d_val;
        dim1b_results(d_idx, c_id).c_id = c_id;
        
        csv_records{end+1} = {'Dim1B_EccentricDist', sprintf('d=%.2fm', d_val), Imax_dim1, ctrl_names{c_id}, ...
            res.max_sync, res.rmse_sync, res.rmse_yG, res.overshoot_fwd, res.overshoot_rev, res.settle_fwd, res.settle_rev, ...
            res.int_delta_alloc, res.int_delta_ext, res.int_delta_tot, res.max_delta_FG, res.max_delta_Talpha_phys, res.max_delta_Talpha_model, ...
            res.max_z_aw_norm, res.tv_iL, res.tv_iR, res.tv_total, res.fwd_reverse_samples, res.rev_reverse_samples, NaN, NaN};
    end
end

%% =========================================================================
%% 维度 2: 左右导轨摩擦非对称性扫描 (delta_fric: 0%, 15%, 30%, 50%, 70%)
%% =========================================================================
fprintf('>>> 正在执行 维度 2: 左右导轨摩擦非对称性扫描 (delta_fric = 0%% ~ 70%%) ...\n');
fric_list = [0.0, 0.15, 0.30, 0.50, 0.70];
dim2_results = struct();
Imax_dim2 = 4500.0;

for f_idx = 1:length(fric_list)
    fric_val = fric_list(f_idx);
    env2.delta_m = 3.0;
    env2.d_load = 0.28;
    env2.delta_fric = fric_val;
    env2.Kf_L = mech.Kf;
    env2.Kf_R = mech.Kf;
    
    for c_id = 1:4
        res = simulate_controller(c_id, traj, env2, Imax_dim2, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, mech.Kf, mech.Kf, t_half_idx);
        dim2_results(f_idx, c_id).res = res;
        dim2_results(f_idx, c_id).fric = fric_val;
        dim2_results(f_idx, c_id).c_id = c_id;
        
        csv_records{end+1} = {'Dim2_FrictionAsym', sprintf('fric=%.0f%%', fric_val*100), Imax_dim2, ctrl_names{c_id}, ...
            res.max_sync, res.rmse_sync, res.rmse_yG, res.overshoot_fwd, res.overshoot_rev, res.settle_fwd, res.settle_rev, ...
            res.int_delta_alloc, res.int_delta_ext, res.int_delta_tot, res.max_delta_FG, res.max_delta_Talpha_phys, res.max_delta_Talpha_model, ...
            res.max_z_aw_norm, res.tv_iL, res.tv_iR, res.tv_total, res.fwd_reverse_samples, res.rev_reverse_samples, NaN, NaN};
    end
end

%% =========================================================================
%% 维度 3: 执行器限流能力分级扫描 (Imax: 3500, 4500, 6000, 8000, 12000, 16000)
%% =========================================================================
fprintf('>>> 正在执行 维度 3: 执行器限流能力分级扫描 (Imax = 3500 ~ 16000 counts) ...\n');
imax_list = [3500, 4500, 6000, 8000, 12000, 16000];
dim3_results = struct();

for i_idx = 1:length(imax_list)
    imax_val = imax_list(i_idx);
    env3.delta_m = 3.0;
    env3.d_load = 0.28;
    env3.delta_fric = 0.30;
    env3.Kf_L = mech.Kf;
    env3.Kf_R = mech.Kf;
    
    for c_id = 1:4
        res = simulate_controller(c_id, traj, env3, imax_val, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, mech.Kf, mech.Kf, t_half_idx);
        dim3_results(i_idx, c_id).res = res;
        dim3_results(i_idx, c_id).Imax = imax_val;
        dim3_results(i_idx, c_id).c_id = c_id;
        
        csv_records{end+1} = {'Dim3_ImaxEscalation', sprintf('Imax=%.0f', imax_val), imax_val, ctrl_names{c_id}, ...
            res.max_sync, res.rmse_sync, res.rmse_yG, res.overshoot_fwd, res.overshoot_rev, res.settle_fwd, res.settle_rev, ...
            res.int_delta_alloc, res.int_delta_ext, res.int_delta_tot, res.max_delta_FG, res.max_delta_Talpha_phys, res.max_delta_Talpha_model, ...
            res.max_z_aw_norm, res.tv_iL, res.tv_iR, res.tv_total, res.fwd_reverse_samples, res.rev_reverse_samples, NaN, NaN};
    end
end

%% =========================================================================
%% 维度 4: 平均推力能力恒定条件下的左右推力增益非对称扫描 (4A: 失配退化 vs 4B: 已知校准)
%% =========================================================================
fprintf('>>> 正在执行 维度 4: 左右推力非对称扫描 (4A: 失配退化 vs 4B: 已知校准) ...\n');
ratio_list = [0.70, 0.85, 1.00, 1.15, 1.30];
dim4a_results = struct();
dim4b_results = struct();
Kf_mean = mech.Kf;
Imax_dim4 = 4500.0;

for r_idx = 1:length(ratio_list)
    r_val = ratio_list(r_idx);
    % 严格保持平均总驱动推力恒定: (Kf_L + Kf_R)/2 = Kf_mean
    Kf_L_r = 2.0 * Kf_mean * r_val / (1.0 + r_val);
    Kf_R_r = 2.0 * Kf_mean / (1.0 + r_val);
    
    env4.delta_m = 3.0;
    env4.d_load = 0.28;
    env4.delta_fric = 0.30;
    env4.Kf_L = Kf_L_r;
    env4.Kf_R = Kf_R_r;
    
    % 4A: 失配测试 (控制器使用标称对称 Kf_nom, 真实物理退化, 评估真实容错)
    for c_id = 1:4
        res_blind = simulate_controller(c_id, traj, env4, Imax_dim4, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, Kf_mean, Kf_mean, t_half_idx);
        dim4a_results(r_idx, c_id).res = res_blind;
        dim4a_results(r_idx, c_id).ratio = r_val;
        dim4a_results(r_idx, c_id).c_id = c_id;
        
        csv_records{end+1} = {'Dim4A_ThrustAsym_Blind', sprintf('ratio=%.2f', r_val), Imax_dim4, ctrl_names{c_id}, ...
            res_blind.max_sync, res_blind.rmse_sync, res_blind.rmse_yG, res_blind.overshoot_fwd, res_blind.overshoot_rev, res_blind.settle_fwd, res_blind.settle_rev, ...
            res_blind.int_delta_alloc, res_blind.int_delta_ext, res_blind.int_delta_tot, res_blind.max_delta_FG, res_blind.max_delta_Talpha_phys, res_blind.max_delta_Talpha_model, ...
            res_blind.max_z_aw_norm, res_blind.tv_iL, res_blind.tv_iR, res_blind.tv_total, res_blind.fwd_reverse_samples, res_blind.rev_reverse_samples, NaN, NaN};
    end
    
    % 4B: 已知校准测试 (控制器已知真实退化参数, 作为 oracle 对照)
    for c_id = 1:4
        res_matched = simulate_controller(c_id, traj, env4, Imax_dim4, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, Kf_L_r, Kf_R_r, t_half_idx);
        dim4b_results(r_idx, c_id).res = res_matched;
        dim4b_results(r_idx, c_id).ratio = r_val;
        dim4b_results(r_idx, c_id).c_id = c_id;
        
        csv_records{end+1} = {'Dim4B_ThrustAsym_Matched', sprintf('ratio=%.2f', r_val), Imax_dim4, ctrl_names{c_id}, ...
            res_matched.max_sync, res_matched.rmse_sync, res_matched.rmse_yG, res_matched.overshoot_fwd, res_matched.overshoot_rev, res_matched.settle_fwd, res_matched.settle_rev, ...
            res_matched.int_delta_alloc, res_matched.int_delta_ext, res_matched.int_delta_tot, res_matched.max_delta_FG, res_matched.max_delta_Talpha_phys, res_matched.max_delta_Talpha_model, ...
            res_matched.max_z_aw_norm, res_matched.tv_iL, res_matched.tv_iR, res_matched.tv_total, res_matched.fwd_reverse_samples, res_matched.rev_reverse_samples, NaN, NaN};
    end
end

%% =========================================================================
%% 维度 5: 动态抗饱和参数网格扫描 (K_aw: 0~80, lambda_aw: 5~80, 含 Kaw=0 基准)
%% =========================================================================
fprintf('>>> 正在执行 维度 5: 动态抗饱和参数网格扫描 (K_aw x lambda_aw, 含 Kaw=0 基准) ...\n');
kaw_list = [0.0, 5.0, 10.0, 20.0, 40.0, 80.0];
lam_list = [5.0, 10.0, 20.0, 40.0, 80.0];
dim5_grid = struct();

env5.delta_m = 3.0;
env5.d_load = 0.28;
env5.delta_fric = 0.30;
env5.Kf_L = mech.Kf;
env5.Kf_R = mech.Kf;
Imax_dim5 = 4500.0;

for k_i = 1:length(kaw_list)
    for l_i = 1:length(lam_list)
        ctrl_c2b_grid = ctrl_c2b_base;
        ctrl_c2b_grid.K_aw = kaw_list(k_i) * eye(2);
        ctrl_c2b_grid.lambda_aw = lam_list(l_i);
        
        res_grid = simulate_controller(4, traj, env5, Imax_dim5, ctrl_c2_base, ctrl_c2b_grid, mech, plant, Ts, mech.Kf, mech.Kf, t_half_idx);
        dim5_grid(k_i, l_i).res = res_grid;
        dim5_grid(k_i, l_i).kaw = kaw_list(k_i);
        dim5_grid(k_i, l_i).lam = lam_list(l_i);
        
        csv_records{end+1} = {'Dim5_AW_Tuning', sprintf('Kaw=%.0f_lam=%.0f', kaw_list(k_i), lam_list(l_i)), Imax_dim5, 'C2b-SyncAlloc', ...
            res_grid.max_sync, res_grid.rmse_sync, res_grid.rmse_yG, res_grid.overshoot_fwd, res_grid.overshoot_rev, res_grid.settle_fwd, res_grid.settle_rev, ...
            res_grid.int_delta_alloc, res_grid.int_delta_ext, res_grid.int_delta_tot, res_grid.max_delta_FG, res_grid.max_delta_Talpha_phys, res_grid.max_delta_Talpha_model, ...
            res_grid.max_z_aw_norm, res_grid.tv_iL, res_grid.tv_iR, res_grid.tv_total, res_grid.fwd_reverse_samples, res_grid.rev_reverse_samples, NaN, NaN};
    end
end

%% =========================================================================
%% 维度 6: 往返分段载荷工况 (正向 4.5kg 载荷, 3.3s 静止保持期间切换为 0.5kg 空载, 3.5s 启动返程)
%% =========================================================================
fprintf('>>> 正在执行 维度 6: 往返分段载荷工况 (Segmented Load Dwell Switch) ...\n');
dim6_results = struct();
Imax_dim6 = 4500.0;
dim6_ctrl_names = [ctrl_names, {'C0 (独立级联 PID)', 'C1 (传统交叉耦合 CCC)'}];

% 运行 C2a, C2a-SyncAlloc, C2b, C2b-SyncAlloc (c_id = 1~4)
for c_id = 1:4
    res_lt = simulate_load_transfer(c_id, traj, Imax_dim6, ctrl, ctrl_c1_base, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, t_half_idx);
    dim6_results(c_id).res = res_lt;
    dim6_results(c_id).name = dim6_ctrl_names{c_id};
    
    csv_records{end+1} = {'Dim6_LoadTransfer', 'Fwd4.5kg_Sw3.3s_Rev0.5kg', Imax_dim6, dim6_ctrl_names{c_id}, ...
        res_lt.max_sync, res_lt.rmse_sync, res_lt.rmse_yG, res_lt.overshoot_fwd, res_lt.overshoot_rev, res_lt.settle_fwd, res_lt.settle_rev, ...
        res_lt.int_delta_alloc, res_lt.int_delta_ext, res_lt.int_delta_tot, res_lt.max_delta_FG, res_lt.max_delta_Talpha_phys, res_lt.max_delta_Talpha_model, ...
            res_lt.max_z_aw_norm, res_lt.tv_iL, res_lt.tv_iR, res_lt.tv_total, res_lt.fwd_reverse_samples, res_lt.rev_reverse_samples, res_lt.v_switch, res_lt.omega_switch};
end

% 补充 C0 与 C1 作为基准参考 (c_id = 5, 6)
for c_id = 5:6
    res_lt = simulate_load_transfer(c_id, traj, Imax_dim6, ctrl, ctrl_c1_base, ctrl_c2_base, ctrl_c2b_base, mech, plant, Ts, t_half_idx);
    dim6_results(c_id).res = res_lt;
    dim6_results(c_id).name = dim6_ctrl_names{c_id};
    
    csv_records{end+1} = {'Dim6_LoadTransfer', 'Fwd4.5kg_Sw3.3s_Rev0.5kg', Imax_dim6, dim6_ctrl_names{c_id}, ...
        res_lt.max_sync, res_lt.rmse_sync, res_lt.rmse_yG, res_lt.overshoot_fwd, res_lt.overshoot_rev, res_lt.settle_fwd, res_lt.settle_rev, ...
        res_lt.int_delta_alloc, res_lt.int_delta_ext, res_lt.int_delta_tot, res_lt.max_delta_FG, res_lt.max_delta_Talpha_phys, res_lt.max_delta_Talpha_model, ...
            res_lt.max_z_aw_norm, res_lt.tv_iL, res_lt.tv_iR, res_lt.tv_total, res_lt.fwd_reverse_samples, res_lt.rev_reverse_samples, res_lt.v_switch, res_lt.omega_switch};
end

%% 4. 数据保存: MAT 文件与 CSV 报告
save('sensitivity_results.mat', 'dim1a_results', 'dim1b_results', 'dim2_results', 'dim3_results', 'dim4a_results', 'dim4b_results', 'dim5_grid', 'dim6_results');

total_records = length(csv_records);
csv_fid = fopen('sensitivity_summary.csv', 'w');
fprintf(csv_fid, 'Dimension,SweepParameter,Imax,Controller,Max_Sync_mm,RMSE_sync_mm,RMSE_yG_mm,Overshoot_Fwd_mm,Overshoot_Rev_mm,Settle_Fwd_s,Settle_Rev_s,Int_Delta_Alloc,Int_Delta_Ext,Int_Delta_Tot,Max_Delta_FG_N,Max_Delta_Talpha_Phys_Nm,Max_Delta_Talpha_Model_Nm,Max_z_aw_norm,TV_iL,TV_iR,TV_total,Fwd_Rev_Samples,Rev_Rev_Samples,V_Switch_mps,Omega_Switch_radps\n');
assert(all(cellfun(@(row) numel(row) == 25, csv_records)), 'CSV 记录字段数不一致');
for r_i = 1:total_records
    row = csv_records{r_i};
    fprintf(csv_fid, '%s,%s,%.1f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%d,%d,%.6e,%.6e\n', ...
        row{1}, row{2}, row{3}, row{4}, row{5}, row{6}, row{7}, row{8}, row{9}, row{10}, ...
        row{11}, row{12}, row{13}, row{14}, row{15}, row{16}, row{17}, row{18}, row{19}, row{20}, row{21}, row{22}, row{23}, row{24}, row{25});
end
fclose(csv_fid);
fprintf('\n[OK] 敏感性评测原始数据已存入 sensitivity_results.mat 与 sensitivity_summary.csv (共 %d 条实验记录)\n', total_records);
fprintf('[OK] CSV schema: %d fields per record, %d data rows\n', numel(csv_records{1}), total_records);
dimension_names = unique(cellfun(@(row) row{1}, csv_records, 'UniformOutput', false));
for d_i = 1:numel(dimension_names)
    fprintf('[OK] %s: %d records\n', dimension_names{d_i}, sum(cellfun(@(row) strcmp(row{1}, dimension_names{d_i}), csv_records)));
end

%% 5. 自动导出精确 Markdown 表格供报告嵌入 (避免人工抄写失误)
export_markdown_tables(dim1a_results, dim1b_results, dim2_results, dim3_results, ...
    dim4a_results, dim4b_results, dim5_grid, dim6_results, ...
    dm_list, d_list, fric_list, imax_list, ratio_list, kaw_list, lam_list, ctrl_names, dim6_ctrl_names);
fprintf('[OK] Markdown 表格已自动导出至 sensitivity_tables.md\n');

%% 6. 绘图与可视化展示
fprintf('>>> 正在生成敏感性与工况拓展高分辨率学术图表 ...\n');

% --- 图 1: 偏载敏感性扫描 (维度 1A 偏载质量 & 维度 1B 偏心距) ---
fig1 = figure('Visible', 'off', 'Color', 'w', 'Position', [60, 60, 1100, 750]);

subplot(2, 2, 1); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(m) dim1a_results(m, c_id).res.max_sync, 1:length(dm_list));
    plot(dm_list, vals, 'Marker', 'o', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('偏载质量 \Delta m (kg)'); ylabel('全程峰值同步误差 (mm)');
title('(a) 最大同步误差随偏载质量变化曲线'); legend('Location', 'Northwest', 'FontSize', 7.5);

subplot(2, 2, 2); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(m) dim1a_results(m, c_id).res.rmse_yG, 1:length(dm_list));
    plot(dm_list, vals, 'Marker', 's', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('偏载质量 \Delta m (kg)'); ylabel('质心位移跟踪 RMSE (mm)');
title('(b) 质心跟踪 RMSE 随偏载质量变化 (平动让步代价)'); legend('Location', 'Northwest', 'FontSize', 7.5);

subplot(2, 2, 3); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(d) dim1b_results(d, c_id).res.max_sync, 1:length(d_list));
    plot(d_list, vals, 'Marker', '^', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('偏心距 d (m)'); ylabel('全程峰值同步误差 (mm)');
title('(c) 最大同步误差随偏心距变化曲线 (\Delta m=3.0kg)'); legend('Location', 'Northwest', 'FontSize', 7.5);

subplot(2, 2, 4); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(d) dim1b_results(d, c_id).res.rmse_yG, 1:length(d_list));
    plot(d_list, vals, 'Marker', 'd', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('偏心距 d (m)'); ylabel('质心位移跟踪 RMSE (mm)');
title('(d) 质心跟踪 RMSE 随偏心距变化'); legend('Location', 'Northwest', 'FontSize', 7.5);

saveas(fig1, 'sensitivity_eccentric_load.png');
close(fig1);

% --- 图 2: 摩擦与推力非对称性响应 (维度 2 & 维度 4) ---
fig2 = figure('Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1100, 750]);

subplot(2, 2, 1); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(f) dim2_results(f, c_id).res.max_sync, 1:length(fric_list));
    plot(fric_list * 100, vals, 'Marker', 'o', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('左右摩擦偏差比例 \Delta f_{fric} (%)'); ylabel('峰值同步误差 (mm)');
title('(a) 导轨摩擦差异对同步精度的影响'); legend('Location', 'Northwest', 'FontSize', 7.5);

subplot(2, 2, 2); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(f) dim2_results(f, c_id).res.tv_total, 1:length(fric_list));
    plot(fric_list * 100, vals, 'Marker', 's', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('左右摩擦偏差比例 \Delta f_{fric} (%)'); ylabel('双执行器控制量总变差 TV_{total}');
title('(b) 双电机动作平滑度随摩擦差异变化'); legend('Location', 'Northwest', 'FontSize', 7.5);

subplot(2, 2, 3); hold on; grid on; box on;
% 4A 失配退化对比
for c_id = [1, 4]
    vals = arrayfun(@(r) dim4a_results(r, c_id).res.max_sync, 1:length(ratio_list));
    plot(ratio_list, vals, 'Marker', '^', 'Color', ctrl_colors{c_id}, 'LineStyle', '--', ...
        'LineWidth', 1.8, 'DisplayName', sprintf('%s (失配退化)', ctrl_names{c_id}));
end
% 4B 理想已知退化对比
for c_id = [1, 4]
    vals = arrayfun(@(r) dim4b_results(r, c_id).res.max_sync, 1:length(ratio_list));
    plot(ratio_list, vals, 'Marker', 'o', 'Color', ctrl_colors{c_id}, 'LineStyle', '-', ...
        'LineWidth', 2.0, 'DisplayName', sprintf('%s (已知校准)', ctrl_names{c_id}));
end
xlabel('推力系数非对称比值 K_{f,L} / K_{f,R}'); ylabel('峰值同步误差 (mm)');
title('(c) 推力增益非对称扫描 (失配 vs 已知校准)'); legend('Location', 'Northwest', 'FontSize', 7);

subplot(2, 2, 4); hold on; grid on; box on;
for c_id = [1, 4]
    vals_blind = arrayfun(@(r) dim4a_results(r, c_id).res.max_delta_Talpha_phys, 1:length(ratio_list));
    plot(ratio_list, vals_blind, 'Marker', '^', 'Color', ctrl_colors{c_id}, 'LineStyle', '--', ...
        'LineWidth', 1.8, 'DisplayName', sprintf('%s (失配物理缺额)', ctrl_names{c_id}));
end
for c_id = [1, 4]
    vals_match = arrayfun(@(r) dim4b_results(r, c_id).res.max_delta_Talpha_phys, 1:length(ratio_list));
    plot(ratio_list, vals_match, 'Marker', 'o', 'Color', ctrl_colors{c_id}, 'LineStyle', '-', ...
        'LineWidth', 2.0, 'DisplayName', sprintf('%s (已知校准物理缺额)', ctrl_names{c_id}));
end
xlabel('推力系数非对称比值 K_{f,L} / K_{f,R}'); ylabel('真实物理力矩缺额峰值 (N\cdotm)');
title('(d) 偏转物理力矩缺额对比 (真实物理回算)'); legend('Location', 'Northwest', 'FontSize', 7);

saveas(fig2, 'sensitivity_asymmetry.png');
close(fig2);

% --- 图 3: 执行器限流能力分级相变分析 (维度 3) ---
fig3 = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 750]);

subplot(2, 2, 1); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(i) dim3_results(i, c_id).res.max_sync, 1:length(imax_list));
    plot(imax_list, vals, 'Marker', 'o', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xline(8000, 'k:', 'LineWidth', 1.2, 'DisplayName', '饱和临界分水岭 ~8000');
xlabel('执行器电流限幅 I_{max} (counts)'); ylabel('全程峰值同步误差 (mm)');
title('(a) 峰值同步误差随电流限幅相变曲线'); legend('Location', 'Northeast', 'FontSize', 7.5);

subplot(2, 2, 2); hold on; grid on; box on;
sync_c2a = arrayfun(@(i) dim3_results(i, 1).res.max_sync, 1:length(imax_list));
sync_c2b_sync = arrayfun(@(i) dim3_results(i, 4).res.max_sync, 1:length(imax_list));
reduction_pct = (sync_c2a - sync_c2b_sync) ./ sync_c2a * 100.0;
plot(imax_list, reduction_pct, 'r-o', 'LineWidth', 2.0, 'MarkerFaceColor', 'r');
yline(0, 'k--');
xlabel('执行器电流限幅 I_{max} (counts)'); ylabel('SyncAlloc 同步误差改善比率 (%)');
title('(b) 同步优先分配收益随限流深度演化'); grid on; box on;

subplot(2, 2, 3); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(i) dim3_results(i, c_id).res.rmse_yG, 1:length(imax_list));
    plot(imax_list, vals, 'Marker', 's', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('执行器电流限幅 I_{max} (counts)'); ylabel('质心跟踪 RMSE (mm)');
title('(c) 质心跟踪精度随限流能力变化'); legend('Location', 'Northeast', 'FontSize', 7.5);

subplot(2, 2, 4); hold on; grid on; box on;
for c_id = 1:4
    vals = arrayfun(@(i) dim3_results(i, c_id).res.int_delta_tot, 1:length(imax_list));
    plot(imax_list, vals, 'Marker', 'd', 'Color', ctrl_colors{c_id}, 'LineStyle', ctrl_lines{c_id}, ...
        'LineWidth', ctrl_widths(c_id), 'DisplayName', ctrl_names{c_id});
end
xlabel('执行器电流限幅 I_{max} (counts)'); ylabel('总不可实现残差积分 \int||\Deltai_{tot}||');
title('(d) 总不可实现请求积分随限流深度演化'); legend('Location', 'Northeast', 'FontSize', 7.5);

saveas(fig3, 'sensitivity_Imax_escalation.png');
close(fig3);

% --- 图 4: 抗饱和参数带宽与往返分段载荷时域响应 (维度 5 & 维度 6) ---
fig4 = figure('Visible', 'off', 'Color', 'w', 'Position', [120, 120, 1100, 750]);

subplot(2, 2, 1); hold on; grid on; box on;
lam_idx_nom = 3; % lambda_aw = 20
kaw_vals = kaw_list;
delta_tot_kaw = arrayfun(@(k) dim5_grid(k, lam_idx_nom).res.int_delta_tot, 1:length(kaw_list));
tv_kaw = arrayfun(@(k) dim5_grid(k, lam_idx_nom).res.tv_total, 1:length(kaw_list));
yyaxis left;
plot(kaw_vals, delta_tot_kaw, 'b-o', 'LineWidth', 1.8);
ylabel('总不可实现残差积分 \int||\Deltai_{tot}||', 'Color', 'b');
yyaxis right;
plot(kaw_vals, tv_kaw, 'r-s', 'LineWidth', 1.8);
ylabel('双执行器控制量总变差 TV_{total}', 'Color', 'r');
xlabel('抗饱和增益 K_{aw} (\lambda_{aw}=20)');
title('(a) 抗饱和增益 K_{aw} 响应特性 (含 Kaw=0 基准)');

subplot(2, 2, 2); hold on; grid on; box on;
kaw_idx_nom = 4; % K_aw = 20 (kaw_list(4) == 20)
lam_vals = lam_list;
delta_tot_lam = arrayfun(@(l) dim5_grid(kaw_idx_nom, l).res.int_delta_tot, 1:length(lam_list));
tv_lam = arrayfun(@(l) dim5_grid(kaw_idx_nom, l).res.tv_total, 1:length(lam_list));
yyaxis left;
plot(lam_vals, delta_tot_lam, 'b-o', 'LineWidth', 1.8);
ylabel('总不可实现残差积分 \int||\Deltai_{tot}||', 'Color', 'b');
yyaxis right;
plot(lam_vals, tv_lam, 'r-s', 'LineWidth', 1.8);
ylabel('双执行器控制量总变差 TV_{total}', 'Color', 'r');
xlabel('抗饱和衰减率 \lambda_{aw} (K_{aw}=20)');
title('(b) 抗饱和衰减率 \lambda_{aw} 响应特性');

% 维度 6: 往返分段载荷时域响应
subplot(2, 2, 3); hold on; grid on; box on;
plot(traj.t, traj.y, 'k:', 'LineWidth', 1.5, 'DisplayName', '目标轨迹 y_d');
xline(3.3, 'm:', 'LineWidth', 1.4, 'DisplayName', '静止载荷切换 (3.3s)');
xline(traj.t_half, 'k--', 'LineWidth', 1.3, 'DisplayName', '返程启动点 (3.5s)');
dim6_colors = [ctrl_colors, {[0.5, 0.5, 0.5], [0.1, 0.8, 0.8]}];
dim6_lines = [ctrl_lines, {':', '-.'}];
dim6_widths = [ctrl_widths, 1.3, 1.3];

for c_id = 1:6
    res_curr = dim6_results(c_id).res;
    plot(traj.t, res_curr.yG, 'Color', dim6_colors{c_id}, 'LineStyle', dim6_lines{c_id}, ...
        'LineWidth', dim6_widths(c_id), 'DisplayName', dim6_results(c_id).name);
end
xlabel('时间 t (s)'); ylabel('质心位移 y_G (m)');
title('(c) 往返分段载荷位移跟踪 (正向4.5kg / 反向0.5kg)');
legend('Location', 'Southeast', 'FontSize', 6.5);

subplot(2, 2, 4); hold on; grid on; box on;
xline(3.3, 'm:', 'LineWidth', 1.4, 'DisplayName', '静止载荷切换 (3.3s)');
xline(traj.t_half, 'k--', 'LineWidth', 1.3, 'DisplayName', '返程启动点 (3.5s)');
for c_id = 1:6
    res_curr = dim6_results(c_id).res;
    plot(traj.t, res_curr.esync, 'Color', dim6_colors{c_id}, 'LineStyle', dim6_lines{c_id}, ...
        'LineWidth', dim6_widths(c_id), 'DisplayName', dim6_results(c_id).name);
end
xlabel('时间 t (s)'); ylabel('同步误差 y_R - y_L (mm)');
title('(d) 往返分段载荷同步误差时域响应');
legend('Location', 'Northeast', 'FontSize', 6.5);

saveas(fig4, 'sensitivity_kaw_and_load_transfer.png');
close(fig4);

fprintf('[OK] 所有学术图表已保存完成。\n\n');
diary off;

%% =========================================================================
%% 辅助仿真函数 1: 通用单工况仿真 (支持控制器参数失配与已知校准)
%% =========================================================================
function res = simulate_controller(c_id, traj, env, Imax_current, ctrl_c2, ctrl_c2b, mech, plant, Ts, ctrl_Kf_L, ctrl_Kf_R, t_half_idx)
    N = length(traj.t);
    x = zeros(4, 1);
    
    log_yG = zeros(1, N);
    log_alpha = zeros(1, N);
    log_yG_dot = zeros(1, N);
    log_alpha_dot = zeros(1, N);
    log_yL = zeros(1, N);
    log_yR = zeros(1, N);
    log_iL_cmd = zeros(1, N);
    log_iR_cmd = zeros(1, N);
    log_delta_iL_alloc = zeros(1, N);
    log_delta_iR_alloc = zeros(1, N);
    log_delta_iL_ext = zeros(1, N);
    log_delta_iR_ext = zeros(1, N);
    log_delta_iL_tot = zeros(1, N);
    log_delta_iR_tot = zeros(1, N);
    log_FG_des = zeros(1, N);
    log_FG_alloc_phys = zeros(1, N);
    log_FG_alloc_model = zeros(1, N);
    log_Talpha_des = zeros(1, N);
    log_Talpha_alloc_phys = zeros(1, N);
    log_Talpha_alloc_model = zeros(1, N);
    log_z_aw = zeros(2, N);
    
    state_c2b.z_aw = [0.0; 0.0];
    
    for k = 1:N
        yG_c = x(1); alpha_c = x(2);
        yG_dot_c = x(3); alpha_dot_c = x(4);
        
        yL_c = yG_c - 0.5 * mech.Le * alpha_c;
        yR_c = yG_c + 0.5 * mech.Le * alpha_c;
        
        q_curr = [yG_c; alpha_c];
        qdot_curr = [yG_dot_c; alpha_dot_c];
        qd_curr = [traj.y(k); 0.0];
        qdot_d_curr = [traj.ydot(k); 0.0];
        qddot_d_curr = [traj.yddot(k); 0.0];
        
        switch c_id
            case 1
                % C2a (独立截断, 无 AW)
                [iL_cmd, iR_cmd, info] = controller_c2a_robust( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2, mech.Le, ctrl_Kf_L, ctrl_Kf_R, Imax_current);
                Delta_i_alloc = [0.0; 0.0];
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_beta(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_beta(2);
                Talloc_model = info.v_actual(2);
                
            case 2
                % C2a-SyncAlloc (推力空间同步优先, 无 AW)
                [iL_cmd, iR_cmd, info] = controller_c2a_sync_alloc( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2, mech.Le, ctrl_Kf_L, ctrl_Kf_R, Imax_current);
                Delta_i_alloc = info.Delta_i_alloc;
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_beta(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_beta(2);
                Talloc_model = info.v_actual(2);
                
            case 3
                % C2b (独立截断, 动态 AW)
                [iL_cmd, iR_cmd, state_c2b, info] = controller_c2b_aw( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2b, mech.Le, ctrl_Kf_L, ctrl_Kf_R, Imax_current, state_c2b, Ts);
                Delta_i_alloc = [0.0; 0.0];
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_cmd(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_cmd(2);
                Talloc_model = info.v_actual(2);
                log_z_aw(:, k) = info.z_aw;
                
            case 4
                % C2b-SyncAlloc (提出复合方案)
                [iL_cmd, iR_cmd, state_c2b, info] = controller_c2b_sync_alloc( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2b, mech.Le, ctrl_Kf_L, ctrl_Kf_R, Imax_current, state_c2b, Ts);
                Delta_i_alloc = info.Delta_i_alloc;
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_cmd(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_cmd(2);
                Talloc_model = info.v_actual(2);
                log_z_aw(:, k) = info.z_aw;
        end
        
        % 运行时断言 (Point 4)
        assert(all(isfinite([x; iL_cmd; iR_cmd; Delta_i_alloc; Delta_i_ext; Delta_i_tot])), ...
            '状态或控制量出现 NaN/Inf，step=%d', k);
        assert(abs(iL_cmd) <= Imax_current + 1e-9, 'iL_cmd 越界，step=%d', k);
        assert(abs(iR_cmd) <= Imax_current + 1e-9, 'iR_cmd 越界，step=%d', k);
        if c_id == 2 || c_id == 4
            assert(norm(Delta_i_tot - (Delta_i_alloc + Delta_i_ext), inf) < 1e-9, ...
                '残差恒等式不成立，step=%d', k);
            assert(norm(Delta_i_ext, inf) < 1e-9, ...
                'SyncAlloc 外部硬件残差非零，step=%d', k);
        end
        
        % 使用被控对象真实推力系数回算实际物理广义力 (Point 1)
        [FG_actual_phys, Talpha_actual_phys] = ...
            actuator_map_m3508.current_to_force( ...
                iL_cmd, iR_cmd, mech.Le, env.Kf_L, env.Kf_R);
        
        % 动力学单步积分推演 (被控对象使用真实的物理参数 env.Kf_L, env.Kf_R)
        x = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, ...
            env.delta_m, env.d_load, env.delta_fric, Ts, env.Kf_L, env.Kf_R);
        
        log_yG(k) = yG_c;
        log_alpha(k) = alpha_c;
        log_yG_dot(k) = yG_dot_c;
        log_alpha_dot(k) = alpha_dot_c;
        log_yL(k) = yL_c;
        log_yR(k) = yR_c;
        log_iL_cmd(k) = iL_cmd;
        log_iR_cmd(k) = iR_cmd;
        log_delta_iL_alloc(k) = Delta_i_alloc(1);
        log_delta_iR_alloc(k) = Delta_i_alloc(2);
        log_delta_iL_ext(k) = Delta_i_ext(1);
        log_delta_iR_ext(k) = Delta_i_ext(2);
        log_delta_iL_tot(k) = Delta_i_tot(1);
        log_delta_iR_tot(k) = Delta_i_tot(2);
        log_FG_des(k) = FGdes;
        log_FG_alloc_phys(k) = FG_actual_phys;
        log_FG_alloc_model(k) = FGalloc_model;
        log_Talpha_des(k) = Tdes;
        log_Talpha_alloc_phys(k) = Talpha_actual_phys;
        log_Talpha_alloc_model(k) = Talloc_model;
    end
    
    assert(all(isfinite(log_yG)) && all(isfinite(log_yL)) && all(isfinite(log_yR)), ...
        '日志包含非有限数值');
    
    esync = (log_yR - log_yL) * 1e3; % mm
    eyG = (log_yG - traj.y) * 1e3;   % mm
    
    res.max_sync = max(abs(esync));
    res.rmse_sync = sqrt(mean(esync.^2));
    res.rmse_yG = sqrt(mean(eyG.^2));
    res.yG = log_yG;
    res.esync = esync;
    
    % 分阶段停靠超调与单调性检测 (Point 3)
    yG_fwd = log_yG(1:t_half_idx);
    yG_rev = log_yG(t_half_idx:end);
    res.overshoot_fwd = max(0, max(yG_fwd) - traj.y_target) * 1e3; % mm
    res.overshoot_rev = max(0, -min(yG_rev)) * 1e3;               % mm
    
    v_tol = 1e-4; % m/s
    res.fwd_reverse_samples = sum(log_yG_dot(1:t_half_idx) < -v_tol);
    res.rev_reverse_samples = sum(log_yG_dot(t_half_idx:end) > v_tol);
    res.is_monotonic_fwd = (res.fwd_reverse_samples == 0);
    res.is_monotonic_rev = (res.rev_reverse_samples == 0);
    
    % 调节时间 (返程改为相对返程起点的耗时, Point 3)
    tail_fwd = abs(yG_fwd - traj.y_target) * 1e3;
    idx_out_fwd = find(tail_fwd > 1.0, 1, 'last');
    if isempty(idx_out_fwd) || idx_out_fwd >= length(yG_fwd)
        res.settle_fwd = NaN;
    else
        res.settle_fwd = traj.t(idx_out_fwd + 1);
    end
    
    tail_rev = abs(yG_rev - 0.0) * 1e3;
    idx_out_rev = find(tail_rev > 1.0, 1, 'last');
    if isempty(idx_out_rev) || idx_out_rev >= length(yG_rev)
        res.settle_rev_abs = NaN;
        res.settle_rev = NaN;
    else
        res.settle_rev_abs = traj.t(t_half_idx + idx_out_rev);
        res.settle_rev = res.settle_rev_abs - traj.t(t_half_idx);
    end
    
    % 双执行器控制量总变差
    res.tv_iL = sum(abs(diff(log_iL_cmd)));
    res.tv_iR = sum(abs(diff(log_iR_cmd)));
    res.tv_total = res.tv_iL + res.tv_iR;
    
    % 饱和与缺额积分
    d_alloc_norm = sqrt(log_delta_iL_alloc.^2 + log_delta_iR_alloc.^2);
    d_ext_norm   = sqrt(log_delta_iL_ext.^2   + log_delta_iR_ext.^2);
    d_tot_norm   = sqrt(log_delta_iL_tot.^2   + log_delta_iR_tot.^2);
    res.int_delta_alloc = trapz(traj.t, d_alloc_norm);
    res.int_delta_ext   = trapz(traj.t, d_ext_norm);
    res.int_delta_tot   = trapz(traj.t, d_tot_norm);
    
    % 广义力与偏转力矩缺额峰值 (真实物理缺额 vs 模型内部缺额, Point 1)
    delta_FG_phys = abs(log_FG_alloc_phys - log_FG_des);
    delta_Talpha_phys = abs(log_Talpha_alloc_phys - log_Talpha_des);
    delta_Talpha_model = abs(log_Talpha_alloc_model - log_Talpha_des);
    res.max_delta_FG = max(delta_FG_phys);
    res.max_delta_Talpha = max(delta_Talpha_phys);               % 默认作为核心物理缺额
    res.max_delta_Talpha_phys = max(delta_Talpha_phys);          % 真实物理力矩缺额
    res.max_delta_Talpha_model = max(delta_Talpha_model);        % 分配器模型内力矩缺额
    
    % 状态 z_aw 峰值
    res.max_z_aw_norm = max(sqrt(log_z_aw(1,:).^2 + log_z_aw(2,:).^2));
end

%% =========================================================================
%% 辅助仿真函数 2: 往返分段载荷突变仿真 (时变参数与全控制器支持)
%% =========================================================================
function res = simulate_load_transfer(c_id, traj, Imax_current, ctrl, ctrl_c1, ctrl_c2, ctrl_c2b, mech, plant, Ts, t_half_idx)
    N = length(traj.t);
    x = zeros(4, 1);
    
    log_yG = zeros(1, N);
    log_alpha = zeros(1, N);
    log_yG_dot = zeros(1, N);
    log_alpha_dot = zeros(1, N);
    log_yL = zeros(1, N);
    log_yR = zeros(1, N);
    log_iL_cmd = zeros(1, N);
    log_iR_cmd = zeros(1, N);
    log_delta_iL_alloc = zeros(1, N);
    log_delta_iR_alloc = zeros(1, N);
    log_delta_iL_ext = zeros(1, N);
    log_delta_iR_ext = zeros(1, N);
    log_delta_iL_tot = zeros(1, N);
    log_delta_iR_tot = zeros(1, N);
    log_FG_des = zeros(1, N);
    log_FG_alloc_phys = zeros(1, N);
    log_FG_alloc_model = zeros(1, N);
    log_Talpha_des = zeros(1, N);
    log_Talpha_alloc_phys = zeros(1, N);
    log_Talpha_alloc_model = zeros(1, N);
    log_z_aw = zeros(2, N);
    
    state_c2b.z_aw = [0.0; 0.0];
    delta_fric = 0.30;
    
    % C0 / C1 控制器状态与常数
    m_to_ecd = (ctrl.ecd_cpr * mech.N) / (2.0 * pi * mech.rp);
    ms_to_rpm = (60.0 * mech.N) / (2.0 * pi * mech.rp);
    
    s_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    s_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    s_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl.spd_max_out, 2000);
    s_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl.spd_max_out, 2000);
    
    states_c1.pid_ang_L = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    states_c1.pid_ang_R = init_pid_struct(ctrl.ang_kp, ctrl.ang_ki, ctrl.ang_kd, 3200, 1000);
    states_c1.pid_spd_L = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl_c1.spd_max_out, 2000);
    states_c1.pid_spd_R = init_pid_struct(ctrl.spd_kp, ctrl.spd_ki, ctrl.spd_kd, ctrl_c1.spd_max_out, 2000);
    
    t_load_switch = 3.3;   % 静止保持期间切换 (全部 6 种控制器均已停稳 |v| < 1e-3 m/s)
    
    for k = 1:N
        tk = traj.t(k);
        % 分段载荷工况: 正向 4.5kg, 3.3s 静止保持期间切换为 0.5kg 空载, 3.5s 启动返程 (Point 2)
        if tk < t_load_switch
            dm_k = 4.5;
            d_load_k = 0.28;
        else
            dm_k = 0.5;
            d_load_k = 0.10;
        end
        
        yG_c = x(1); alpha_c = x(2);
        yG_dot_c = x(3); alpha_dot_c = x(4);
        
        yL_c = yG_c - 0.5 * mech.Le * alpha_c;
        yR_c = yG_c + 0.5 * mech.Le * alpha_c;
        vL_c = yG_dot_c - 0.5 * mech.Le * alpha_dot_c;
        vR_c = yG_dot_c + 0.5 * mech.Le * alpha_dot_c;
        
        yd_t = traj.y(k);
        
        % 速度梯度限幅
        ecd_L = yL_c * m_to_ecd;
        ecd_tar_L = yd_t * m_to_ecd;
        err_rev = abs(ecd_tar_L - ecd_L) / ctrl.ecd_cpr;
        if tk < traj.t_half
            stroke_dist = abs(ecd_L);
        else
            stroke_dist = abs(ecd_L - traj.y_target * m_to_ecd);
        end
        dyn_max = min(err_rev, stroke_dist / ctrl.ecd_cpr) * ctrl.ang_gradient_slope;
        dyn_max = max(ctrl.ang_gradient_min_out, min(ctrl.ang_gradient_max_out, dyn_max));
        
        q_curr = [yG_c; alpha_c];
        qdot_curr = [yG_dot_c; alpha_dot_c];
        qd_curr = [traj.y(k); 0.0];
        qdot_d_curr = [traj.ydot(k); 0.0];
        qddot_d_curr = [traj.yddot(k); 0.0];
        
        switch c_id
            case 1
                [iL_cmd, iR_cmd, info] = controller_c2a_robust( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2, mech.Le, mech.Kf, mech.Kf, Imax_current);
                Delta_i_alloc = [0.0; 0.0];
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_beta(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_beta(2);
                Talloc_model = info.v_actual(2);
                
            case 2
                [iL_cmd, iR_cmd, info] = controller_c2a_sync_alloc( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2, mech.Le, mech.Kf, mech.Kf, Imax_current);
                Delta_i_alloc = info.Delta_i_alloc;
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_beta(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_beta(2);
                Talloc_model = info.v_actual(2);
                
            case 3
                [iL_cmd, iR_cmd, state_c2b, info] = controller_c2b_aw( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2b, mech.Le, mech.Kf, mech.Kf, Imax_current, state_c2b, Ts);
                Delta_i_alloc = [0.0; 0.0];
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_cmd(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_cmd(2);
                Talloc_model = info.v_actual(2);
                log_z_aw(:, k) = info.z_aw;
                
            case 4
                [iL_cmd, iR_cmd, state_c2b, info] = controller_c2b_sync_alloc( ...
                    q_curr, qdot_curr, qd_curr, qdot_d_curr, qddot_d_curr, ...
                    ctrl_c2b, mech.Le, mech.Kf, mech.Kf, Imax_current, state_c2b, Ts);
                Delta_i_alloc = info.Delta_i_alloc;
                Delta_i_ext = info.Delta_i_external;
                Delta_i_tot = info.Delta_i_total;
                FGdes = info.v_cmd(1);
                FGalloc_model = info.v_actual(1);
                Tdes = info.v_cmd(2);
                Talloc_model = info.v_actual(2);
                log_z_aw(:, k) = info.z_aw;
                
            case 5
                % C0 (独立级联 PID)
                ecd_R = -yR_c * m_to_ecd;
                ecd_tar_R = -yd_t * m_to_ecd;
                s_ang_L.max_out = dyn_max; s_ang_R.max_out = dyn_max;
                [rpm_L, s_ang_L] = pid_calc(s_ang_L, ecd_L, ecd_tar_L);
                [rpm_R, s_ang_R] = pid_calc(s_ang_R, ecd_R, ecd_tar_R);
                [iL_raw, s_spd_L, iL_unsat] = pid_calc(s_spd_L,  vL_c * ms_to_rpm, rpm_L);
                [iR_raw, s_spd_R, iR_unsat] = pid_calc(s_spd_R, -vR_c * ms_to_rpm, rpm_R);
                [iL_cmd, iR_cmd, Delta_i_ext, ~] = saturation_model.hard_sat(iL_raw, iR_raw, Imax_current);
                Delta_i_alloc = [0.0; 0.0];
                Delta_i_tot = [iL_cmd; iR_cmd] - [iL_unsat; iR_unsat];
                FGdes = NaN; FGalloc_model = NaN; Tdes = NaN; Talloc_model = NaN;
                
            case 6
                % C1 (传统交叉耦合 CCC)
                [iL_cmd, iR_cmd, states_c1, info_c1] = controller_c1_ccc( ...
                    yL_c, yR_c, vL_c, vR_c, yd_t, dyn_max, ctrl_c1, mech, Imax_current, states_c1);
                Delta_i_alloc = [0.0; 0.0];
                Delta_i_ext = info_c1.Delta_i_external;
                Delta_i_tot = info_c1.Delta_i_total;
                FGdes = NaN; FGalloc_model = NaN; Tdes = NaN; Talloc_model = NaN;
        end
        
        % 运行时断言 (Point 4)
        assert(all(isfinite([x; iL_cmd; iR_cmd; Delta_i_alloc; Delta_i_ext; Delta_i_tot])), ...
            '状态或控制量出现 NaN/Inf，step=%d', k);
        assert(abs(iL_cmd) <= Imax_current + 1e-9, 'iL_cmd 越界，step=%d', k);
        assert(abs(iR_cmd) <= Imax_current + 1e-9, 'iR_cmd 越界，step=%d', k);
        if c_id == 2 || c_id == 4
            assert(norm(Delta_i_tot - (Delta_i_alloc + Delta_i_ext), inf) < 1e-9, ...
                '残差恒等式不成立，step=%d', k);
            assert(norm(Delta_i_ext, inf) < 1e-9, ...
                'SyncAlloc 外部硬件残差非零，step=%d', k);
        end
        
        % 使用被控对象真实推力系数回算实际物理广义力 (Point 1)
        if c_id <= 4
            [FG_actual_phys, Talpha_actual_phys] = ...
                actuator_map_m3508.current_to_force( ...
                    iL_cmd, iR_cmd, mech.Le, mech.Kf, mech.Kf);
        else
            FG_actual_phys = NaN;
            Talpha_actual_phys = NaN;
        end
        
        x = gantry_dynamics_step(x, iL_cmd, iR_cmd, mech, plant, ...
            dm_k, d_load_k, delta_fric, Ts, mech.Kf, mech.Kf);
        
        log_yG(k) = yG_c;
        log_alpha(k) = alpha_c;
        log_yG_dot(k) = yG_dot_c;
        log_alpha_dot(k) = alpha_dot_c;
        log_yL(k) = yL_c;
        log_yR(k) = yR_c;
        log_iL_cmd(k) = iL_cmd;
        log_iR_cmd(k) = iR_cmd;
        log_delta_iL_alloc(k) = Delta_i_alloc(1);
        log_delta_iR_alloc(k) = Delta_i_alloc(2);
        log_delta_iL_ext(k) = Delta_i_ext(1);
        log_delta_iR_ext(k) = Delta_i_ext(2);
        log_delta_iL_tot(k) = Delta_i_tot(1);
        log_delta_iR_tot(k) = Delta_i_tot(2);
        log_FG_des(k) = FGdes;
        log_FG_alloc_phys(k) = FG_actual_phys;
        log_FG_alloc_model(k) = FGalloc_model;
        log_Talpha_des(k) = Tdes;
        log_Talpha_alloc_phys(k) = Talpha_actual_phys;
        log_Talpha_alloc_model(k) = Talloc_model;
    end
    
    assert(all(isfinite(log_yG)) && all(isfinite(log_yL)) && all(isfinite(log_yR)), ...
        '日志包含非有限数值');
    
    % 载荷切换点速度检查 (Point 2)
    k_switch = find(traj.t >= t_load_switch, 1);
    fprintf('Controller %d at switch (t=%.2fs): v=%.6e, omega=%.6e\n', c_id, t_load_switch, log_yG_dot(k_switch), log_alpha_dot(k_switch));
    if c_id <= 4
        assert(abs(log_yG_dot(k_switch)) < 1e-3, 'C2载荷切换时质心速度未接近零');
        assert(abs(log_alpha_dot(k_switch)) < 1e-3, 'C2载荷切换时角速度未接近零');
    else
        % 经典基准 C0/C1 无前馈模型，调节较慢，检查在静止窗口内的有界性
        assert(abs(log_yG_dot(k_switch)) < 5e-3, '基准控制器载荷切换时质心速度超限');
        assert(abs(log_alpha_dot(k_switch)) < 5e-3, '基准控制器载荷切换时角速度超限');
    end
    res.v_switch = log_yG_dot(k_switch);
    res.omega_switch = log_alpha_dot(k_switch);
    
    esync = (log_yR - log_yL) * 1e3;
    eyG = (log_yG - traj.y) * 1e3;
    
    res.max_sync = max(abs(esync));
    res.rmse_sync = sqrt(mean(esync.^2));
    res.rmse_yG = sqrt(mean(eyG.^2));
    res.yG = log_yG;
    res.esync = esync;
    
    % 分阶段停靠超调与单调性检测 (Point 3)
    yG_fwd = log_yG(1:t_half_idx);
    yG_rev = log_yG(t_half_idx:end);
    res.overshoot_fwd = max(0, max(yG_fwd) - traj.y_target) * 1e3;
    res.overshoot_rev = max(0, -min(yG_rev)) * 1e3;
    
    v_tol = 1e-4; % m/s
    res.fwd_reverse_samples = sum(log_yG_dot(1:t_half_idx) < -v_tol);
    res.rev_reverse_samples = sum(log_yG_dot(t_half_idx:end) > v_tol);
    res.is_monotonic_fwd = (res.fwd_reverse_samples == 0);
    res.is_monotonic_rev = (res.rev_reverse_samples == 0);
    
    % 调节时间 (返程改为相对返程起点的耗时, Point 3)
    tail_fwd = abs(yG_fwd - traj.y_target) * 1e3;
    idx_out_fwd = find(tail_fwd > 1.0, 1, 'last');
    if isempty(idx_out_fwd) || idx_out_fwd >= length(yG_fwd)
        res.settle_fwd = NaN;
    else
        res.settle_fwd = traj.t(idx_out_fwd + 1);
    end
    
    tail_rev = abs(yG_rev - 0.0) * 1e3;
    idx_out_rev = find(tail_rev > 1.0, 1, 'last');
    if isempty(idx_out_rev) || idx_out_rev >= length(yG_rev)
        res.settle_rev_abs = NaN;
        res.settle_rev = NaN;
    else
        res.settle_rev_abs = traj.t(t_half_idx + idx_out_rev);
        res.settle_rev = res.settle_rev_abs - traj.t(t_half_idx);
    end
    
    res.tv_iL = sum(abs(diff(log_iL_cmd)));
    res.tv_iR = sum(abs(diff(log_iR_cmd)));
    res.tv_total = res.tv_iL + res.tv_iR;
    
    d_alloc_norm = sqrt(log_delta_iL_alloc.^2 + log_delta_iR_alloc.^2);
    d_ext_norm   = sqrt(log_delta_iL_ext.^2   + log_delta_iR_ext.^2);
    d_tot_norm   = sqrt(log_delta_iL_tot.^2   + log_delta_iR_tot.^2);
    res.int_delta_alloc = trapz(traj.t, d_alloc_norm);
    res.int_delta_ext   = trapz(traj.t, d_ext_norm);
    res.int_delta_tot   = trapz(traj.t, d_tot_norm);
    
    if c_id <= 4
        delta_FG_phys = abs(log_FG_alloc_phys - log_FG_des);
        delta_Talpha_phys = abs(log_Talpha_alloc_phys - log_Talpha_des);
        delta_Talpha_model = abs(log_Talpha_alloc_model - log_Talpha_des);
        res.max_delta_FG = max(delta_FG_phys);
        res.max_delta_Talpha = max(delta_Talpha_phys);
        res.max_delta_Talpha_phys = max(delta_Talpha_phys);
        res.max_delta_Talpha_model = max(delta_Talpha_model);
    else
        res.max_delta_FG = NaN;
        res.max_delta_Talpha = NaN;
        res.max_delta_Talpha_phys = NaN;
        res.max_delta_Talpha_model = NaN;
    end
    res.max_z_aw_norm = max(sqrt(log_z_aw(1,:).^2 + log_z_aw(2,:).^2));
end

%% 辅助函数: PID 结构体初始化
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

%% =========================================================================
%% 辅助函数 3: 自动导出报告 Markdown 表格 (严格避免手动复制误差)
%% =========================================================================
function export_markdown_tables(dim1a, dim1b, dim2, dim3, dim4a, dim4b, dim5, dim6, ...
    dm_list, d_list, fric_list, imax_list, ratio_list, kaw_list, lam_list, ctrl_names, dim6_ctrl_names)

    fid = fopen('sensitivity_tables.md', 'w');
    
    % --- 维度 1A ---
    fprintf(fid, '### 维度 1A: 偏载质量敏感性扫描表 (d = 0.28m, Imax = 4500 counts)\n\n');
    fprintf(fid, '| 偏载质量 $\\Delta m$ | 控制器配置 | 峰值同步误差 $\\text{Max\\_Sync}$ (mm) | 同步均方根 $\\text{RMSE}_{\\text{sync}}$ (mm) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | 总不可实现积分 $\\int\\|\\Delta\\mathbf{i}_{\\text{tot}}\\|$ | 物理力矩缺额 $\\Delta T_\\alpha$ (N·m) | 执行器总变差 $\\text{TV}_{\\text{total}}$ |\n');
    fprintf(fid, '| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for m_i = 1:length(dm_list)
        for c_i = 1:4
            r = dim1a(m_i, c_i).res;
            prefix = '';
            if c_i == 1, prefix = sprintf('**%.1f kg**', dm_list(m_i)); end
            fprintf(fid, '| %s | %s | %.4f | %.4f | %.2f | %.1f | %.4f | %.1f |\n', ...
                prefix, ctrl_names{c_i}, r.max_sync, r.rmse_sync, r.rmse_yG, r.int_delta_tot, r.max_delta_Talpha_phys, r.tv_total);
        end
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 1B ---
    fprintf(fid, '### 维度 1B: 偏心距敏感性扫描表 (\\Delta m = 3.0kg, Imax = 4500 counts)\n\n');
    fprintf(fid, '| 偏心距 $d$ | 控制器配置 | 峰值同步误差 $\\text{Max\\_Sync}$ (mm) | 同步均方根 $\\text{RMSE}_{\\text{sync}}$ (mm) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | 总不可实现积分 $\\int\\|\\Delta\\mathbf{i}_{\\text{tot}}\\|$ | 物理力矩缺额 $\\Delta T_\\alpha$ (N·m) | 执行器总变差 $\\text{TV}_{\\text{total}}$ |\n');
    fprintf(fid, '| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for d_i = 1:length(d_list)
        for c_i = 1:4
            r = dim1b(d_i, c_i).res;
            prefix = '';
            if c_i == 1, prefix = sprintf('**%.2f m**', d_list(d_i)); end
            fprintf(fid, '| %s | %s | %.4f | %.4f | %.2f | %.1f | %.4f | %.1f |\n', ...
                prefix, ctrl_names{c_i}, r.max_sync, r.rmse_sync, r.rmse_yG, r.int_delta_tot, r.max_delta_Talpha_phys, r.tv_total);
        end
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 2 ---
    fprintf(fid, '### 维度 2: 左右导轨摩擦非对称性扫描表 (\\Delta m = 3.0kg, d = 0.28m, Imax = 4500 counts)\n\n');
    fprintf(fid, '| 摩擦偏差比例 | 控制器配置 | 峰值同步误差 $\\text{Max\\_Sync}$ (mm) | 同步均方根 $\\text{RMSE}_{\\text{sync}}$ (mm) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | 总不可实现积分 $\\int\\|\\Delta\\mathbf{i}_{\\text{tot}}\\|$ | 物理力矩缺额 $\\Delta T_\\alpha$ (N·m) | 执行器总变差 $\\text{TV}_{\\text{total}}$ |\n');
    fprintf(fid, '| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for f_i = 1:length(fric_list)
        for c_i = 1:4
            r = dim2(f_i, c_i).res;
            prefix = '';
            if c_i == 1, prefix = sprintf('**%.0f%%**', fric_list(f_i)*100); end
            fprintf(fid, '| %s | %s | %.4f | %.4f | %.2f | %.1f | %.4f | %.1f |\n', ...
                prefix, ctrl_names{c_i}, r.max_sync, r.rmse_sync, r.rmse_yG, r.int_delta_tot, r.max_delta_Talpha_phys, r.tv_total);
        end
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 3 ---
    fprintf(fid, '### 维度 3: 执行器限流能力分级相变分析表 (\\Delta m = 3.0kg, d = 0.28m)\n\n');
    fprintf(fid, '| 限流电流 $I_{\\max}$ | 饱和状态定性 | C2a $\\text{Max\\_Sync}$ (mm) | SyncAlloc $\\text{Max\\_Sync}$ (mm) | 同步改善比率 (%%) | C2a $\\text{RMSE}_{yG}$ (mm) | SyncAlloc $\\text{RMSE}_{yG}$ (mm) | 总不可实现积分 $\\int\\|\\Delta\\mathbf{i}_{\\text{tot}}\\|$ |\n');
    fprintf(fid, '| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for i_i = 1:length(imax_list)
        r_c2a = dim3(i_i, 1).res;
        r_sync = dim3(i_i, 4).res;
        im = imax_list(i_i);
        if im >= 8000
            sat_qual = '线性充裕无饱和';
        elseif im == 6000
            sat_qual = '轻度动态饱和';
        elseif im == 4500
            sat_qual = '中度动态饱和 (标称)';
        else
            sat_qual = '深度重度饱和';
        end
        impr = (r_c2a.max_sync - r_sync.max_sync) / r_c2a.max_sync * 100;
        fprintf(fid, '| **%.0f** | %s | %.4f | **%.4f** | %+.1f%% | %.2f | %.2f | %.1f |\n', ...
            im, sat_qual, r_c2a.max_sync, r_sync.max_sync, impr, r_c2a.rmse_yG, r_sync.rmse_yG, r_sync.int_delta_tot);
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 4A ---
    fprintf(fid, '### 维度 4A: 左右推力非对称失配退化表 (Blind Mismatch, 控制器使用标称 Kf)\n\n');
    fprintf(fid, '| 推力比值 $r$ | 驱动物理分布与负载耦合关系 | C2a $\\text{Max\\_Sync}$ (mm) | SyncAlloc $\\text{Max\\_Sync}$ (mm) | 同步改善比率 (%%) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | C2a 物理力矩缺额 (N·m) | SyncAlloc 物理力矩缺额 (N·m) | SyncAlloc 模型力矩缺额 (N·m) |\n');
    fprintf(fid, '| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for r_i = 1:length(ratio_list)
        r_c2a = dim4a(r_i, 1).res;
        r_sync = dim4a(r_i, 4).res;
        rv = ratio_list(r_i);
        impr = (r_c2a.max_sync - r_sync.max_sync) / r_c2a.max_sync * 100;
        if rv < 1.0
            desc = '右电机强，天然分担右侧偏载';
        elseif rv == 1.0
            desc = '标称对称基准';
        else
            desc = '左电机强，与右侧偏载恶性叠加';
        end
        fprintf(fid, '| **%.2f** | %s | %.4f | **%.4f** | %+.1f%% | %.2f | %.4f | %.4f | %.4f |\n', ...
            rv, desc, r_c2a.max_sync, r_sync.max_sync, impr, r_sync.rmse_yG, r_c2a.max_delta_Talpha_phys, r_sync.max_delta_Talpha_phys, r_sync.max_delta_Talpha_model);
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 4B ---
    fprintf(fid, '### 维度 4B: 左右推力非对称已知校准表 (Matched Calibration, 作为 oracle 对照)\n\n');
    fprintf(fid, '| 推力比值 $r$ | 驱动物理分布与负载耦合关系 | C2a $\\text{Max\\_Sync}$ (mm) | SyncAlloc $\\text{Max\\_Sync}$ (mm) | 同步改善比率 (%%) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | C2a 物理力矩缺额 (N·m) | SyncAlloc 物理力矩缺额 (N·m) | SyncAlloc 模型力矩缺额 (N·m) |\n');
    fprintf(fid, '| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for r_i = 1:length(ratio_list)
        r_c2a = dim4b(r_i, 1).res;
        r_sync = dim4b(r_i, 4).res;
        rv = ratio_list(r_i);
        impr = (r_c2a.max_sync - r_sync.max_sync) / r_c2a.max_sync * 100;
        if rv < 1.0
            desc = '右电机强（天然被动抵消偏载）';
        elseif rv == 1.0
            desc = '标称对称基准';
        else
            desc = '左电机强，与右侧偏载恶性叠加';
        end
        fprintf(fid, '| **%.2f** | %s | %.4f | **%.4f** | %+.1f%% | %.2f | %.4f | %.4f | %.4f |\n', ...
            rv, desc, r_c2a.max_sync, r_sync.max_sync, impr, r_sync.rmse_yG, r_c2a.max_delta_Talpha_phys, r_sync.max_delta_Talpha_phys, r_sync.max_delta_Talpha_model);
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 5 参数权衡表 ---
    fprintf(fid, '### 维度 5: 动态抗饱和参数权衡网格扫描表 (K_aw 相对 K_aw=0 基准)\n\n');
    fprintf(fid, '| $K_{\\text{aw}}$ | $\\lambda_{\\text{aw}}$ | 峰值同步误差 $\\text{Max\\_Sync}$ (mm) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | 总不可实现积分 $\\int\\|\\Delta\\mathbf{i}_{\\text{tot}}\\|$ | 相对 $K_{\\text{aw}}=0$ 削减率 (%%) | 执行器总变差 $\\text{TV}_{\\text{total}}$ | 抗饱和状态模峰值 $\\|\\mathbf{z}_{\\text{aw}}\\|_{\\max}$ |\n');
    fprintf(fid, '| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    
    % 取 Kaw=0, lam=20 作为无抗饱和基准基底
    base_delta_tot = dim5(1, 3).res.int_delta_tot;
    for k_i = 1:length(kaw_list)
        for l_i = [1, 3, 5] % 输出部分代表性 lambda (5, 20, 80)
            r = dim5(k_i, l_i).res;
            kw = kaw_list(k_i);
            lw = lam_list(l_i);
            reduc = (base_delta_tot - r.int_delta_tot) / base_delta_tot * 100;
            fprintf(fid, '| **%.1f** | **%.1f** | %.4f | %.2f | %.1f | %+.1f%% | %.1f | %.1f |\n', ...
                kw, lw, r.max_sync, r.rmse_yG, r.int_delta_tot, reduc, r.tv_total, r.max_z_aw_norm);
        end
    end
    fprintf(fid, '\n---\n\n');
    
    % --- 维度 6 ---
    fprintf(fid, '### 维度 6: 往返分段载荷工况对比表 (正向 4.5kg, 3.3s 静止切换至 0.5kg, 3.5s 返程启动)\n\n');
    fprintf(fid, '| 控制器名称 | 架构特征分类 | 全程 $\\text{Max\\_Sync}$ (mm) | 质心位移 $\\text{RMSE}_{yG}$ (mm) | 正向超调 (mm) | 反向超调 (mm) | 正向调节时间 (s) | 返程调节耗时 (s) | 总不可实现积分 $\\int\\|\\Delta\\mathbf{i}_{\\text{tot}}\\|$ | 执行器总变差 $\\text{TV}_{\\text{total}}$ | 载荷切换速度 $v$ (m/s) | 载荷切换角速度 $\\omega$ (rad/s) | 单调性样本违例 (正/反) |\n');
    fprintf(fid, '| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n');
    for c_i = 1:6
        r = dim6(c_i).res;
        nm = dim6_ctrl_names{c_i};
        if c_i == 1
            arch = '独立截断 (无 AW)';
        elseif c_i == 2
            arch = '同步优先 (无 AW)';
        elseif c_i == 3
            arch = '独立截断 + 动态 AW';
        elseif c_i == 4
            arch = '**提出复合方案**';
        elseif c_i == 5
            arch = '传统独立级联 PID';
        else
            arch = '传统交叉耦合 CCC';
        end
        fprintf(fid, '| **%s** | %s | %.4f | %.2f | %.4f | %.4f | %.3f | %.3f | %.1f | %.1f | %.2e | %.2e | %d / %d |\n', ...
            nm, arch, r.max_sync, r.rmse_yG, r.overshoot_fwd, r.overshoot_rev, r.settle_fwd, r.settle_rev, r.int_delta_tot, r.tv_total, r.v_switch, r.omega_switch, r.fwd_reverse_samples, r.rev_reverse_samples);
    end
    
    fclose(fid);
end
