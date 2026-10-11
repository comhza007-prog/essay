%% VERIFY_BLNN - Structural, numerical, and development sanity tests
clear; clc;
this_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(this_dir);
addpath(this_dir);
addpath(fullfile(repo_root, 'step1_baseline_c0'));
addpath(fullfile(repo_root, 'step2_advanced_controllers'));

fprintf('=== BLNN verification ===\n');
passed = 0;

cfg.n0 = 4;
cfg.tau = 0.7;
cfg.m_nearest = 3;
cfg.theta = 0.10;
cfg.width = 1.0;
cfg.max_nodes = 60;
cfg.rbf_nodes = 60;
cfg.growth_interval_steps = 1;
cfg.epsilon = [30.0; 30.0];
cfg.eta = [0.01; 0.01];
cfg.input_clip = 2.0;
cfg.weight_limit = [100.0; 100.0];
cfg.dhat_limit = [100.0; 100.0];
cfg.gate_scale = 0.08;
cfg.headroom_fraction = 0.15;
cfg.input_scales = ones(5, 2);

%% Test 1: deterministic paper initial structure
s1 = nn_compensator_init('blnn', cfg);
s2 = nn_compensator_init('blnn', cfg);
assert(all(arrayfun(@(n) size(n.centers, 1), s1.net) == cfg.n0));
assert(isequal(s1.net(1).centers, s2.net(1).centers));
assert(all(arrayfun(@(n) numel(n.w_h), s1.net) == cfg.n0));
passed = passed + 1;
fprintf('Test 1 PASS: deterministic n0=4 BLNN initialization.\n');

%% Test 2: Eq. (30)-(31) growth and max-node guard
chi = 2.0 * ones(5, 2);
[~, s1, info] = nn_compensator_step(s1, chi, [0.2; -0.1], cfg, 0.001);
assert(all(info.nodes == cfg.n0 + 1));
assert(all(info.grew));
for k = 1:100
    phase = 0.1 * k;
    chi_k = 1.8 * [sin(phase) cos(phase); cos(phase) sin(phase); ...
        sin(2*phase) cos(2*phase); cos(3*phase) sin(3*phase); ...
        sin(4*phase) cos(4*phase)];
    [~, s1] = nn_compensator_step(s1, chi_k, [0.1; -0.1], cfg, 0.001);
end
assert(all(arrayfun(@(n) size(n.centers, 1), s1.net) <= cfg.max_nodes));
passed = passed + 1;
fprintf('Test 2 PASS: adaptive growth rule and 60-node ceiling.\n');

%% Test 3: Eq. (14) updates mapping and enhancement weights
s3 = nn_compensator_init('blnn', cfg);
[d0, s3] = nn_compensator_step(s3, zeros(5, 2), [0.5; -0.3], cfg, 0.001);
assert(all(abs(d0) < 1e-12));
assert(norm(s3.net(1).w_h) > 0);
assert(norm(s3.net(1).w_cos) > 0);
assert(norm(s3.net(1).w_sin) > 0);
assert(all(s3.projection_hits == 0));
passed = passed + 1;
fprintf('Test 3 PASS: online mapping/enhancement weight update.\n');

%% Test 4: fixed RBFNN has 60 nodes and no enhancement weights
s4 = nn_compensator_init('rbf', cfg);
for k = 1:10
    [~, s4] = nn_compensator_step(s4, 0.2 * ones(5, 2), [0.2; 0.1], cfg, 0.001);
end
assert(all(arrayfun(@(n) size(n.centers, 1), s4.net) == cfg.rbf_nodes));
assert(all(arrayfun(@(n) norm(n.w_cos) + norm(n.w_sin), s4.net) == 0));
passed = passed + 1;
fprintf('Test 4 PASS: fixed 60-node RBFNN ablation.\n');

%% Test 5: no-network mode is an exact zero compensator
s5 = nn_compensator_init('none', cfg);
[d5, s5, info5] = nn_compensator_step(s5, randn(5, 2), [1; 1], cfg, 0.001);
assert(all(d5 == 0) && all(info5.nodes == 0) && s5.total_growth == 0);
passed = passed + 1;
fprintf('Test 5 PASS: no-NN exact-zero ablation.\n');

%% Test 6: one-step closed-loop interface and current constraints
[~, mech, plant] = param_init();
ctrl.M_hat = diag([mech.mG_nom, mech.J_alpha_nom]);
ctrl.B_hat = diag([2 * plant.b_nom, plant.B_alpha]);
ctrl.K_hat = diag([0, plant.K_alpha]);
ctrl.A_hat = diag([2 * plant.fc_nom, 0]);
ctrl.Gamma = diag([25, 18]);
ctrl.K1 = diag([40, 25]);
ctrl.K2_init = [4; 1]; ctrl.K2_min = [0.05; 0.02]; ctrl.K2_max = [24; 10];
ctrl.k2_omega = [5; 2]; ctrl.k2_beta = [1; 1];
ctrl.phi = [0.025; 0.012]; ctrl.eps_v = 0.01; ctrl.eps_alpha = 0.01;
ctrl.I_fw_limit = [16000; 16000]; ctrl.K_aw = 16 * eye(2); ctrl.lambda_aw = 20;
ctrl.nn = cfg;
state.z_aw = zeros(2, 1); state.k2 = ctrl.K2_init;
state.net = nn_compensator_init('blnn', cfg);
Imax = [6000; 4000];
[iL, iR, state, info6] = controller_c3b_nn_sync_alloc( ...
    zeros(2,1), zeros(2,1), zeros(2,1), [0.1;0], zeros(2,1), zeros(2,1), ...
    ctrl, mech.Le, mech.Kf, 0.92*mech.Kf, Imax, state, 0.001);
assert(all(isfinite([iL; iR; info6.v_cmd; info6.d_hat])));
assert(abs(iL) <= Imax(1) + 1e-9 && abs(iR) <= Imax(2) + 1e-9);
passed = passed + 1;
fprintf('Test 6 PASS: finite closed-loop output and asymmetric limits.\n');

%% Test 7: full benchmark output and acceptance criteria
summary_path = fullfile(this_dir, 'blnn_results_summary.csv');
assert(isfile(summary_path), 'Run run_blnn_benchmark.m before acceptance verification.');
T = readtable(summary_path, 'TextType', 'string');
assert(height(T) == 8);
numeric_vars = setdiff(T.Properties.VariableNames, {'Scenario', 'Method'});
for k = 1:numel(numeric_vars)
    assert(all(isfinite(T.(numeric_vars{k}))));
end

scenario_names = unique(T.Scenario, 'stable');
rows_u = T(T.Scenario == "Uncertainty", :);
no_u = rows_u(rows_u.Method == "No NN", :);
paper_u = rows_u(rows_u.Method == "Paper-BLNN", :);
assert(paper_u.TrackingRMSE_mm < no_u.TrackingRMSE_mm);
assert(paper_u.SyncRMS_mm < no_u.SyncRMS_mm);

rows_s = T(T.Scenario == "Asymmetric saturation", :);
no_s = rows_s(rows_s.Method == "No NN", :);
ca_s = rows_s(rows_s.Method == "CA-BLNN", :);
assert(ca_s.TrackingRMSE_mm <= 1.05 * no_s.TrackingRMSE_mm);
assert(ca_s.SyncRMS_mm < no_s.SyncRMS_mm);

nn_rows = T(contains(T.Method, "BLNN"), :);
assert(all(nn_rows.FinalNodes_Translation >= cfg.n0 & nn_rows.FinalNodes_Translation <= cfg.max_nodes));
assert(all(nn_rows.FinalNodes_Rotation >= cfg.n0 & nn_rows.FinalNodes_Rotation <= cfg.max_nodes));
passed = passed + 1;
fprintf('Test 7 PASS: current benchmark meets development-level sanity criteria.\n');

fprintf('\nBLNN development checks complete: %d/7 PASS.\n', passed);
