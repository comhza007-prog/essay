%% RUN_BLNN_BENCHMARK - Complete BLNN simulation and ablation benchmark
% Comparison: no NN vs fixed 60-node RBFNN vs adaptive-width BLNN.
% The NN structure follows the source paper. The plant and actuator layer
% use the project's M3508 model, hard current limits, dynamic anti-windup,
% and synchronization-priority allocation.

clear; clc; close all;
this_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(this_dir);
addpath(this_dir);
addpath(fullfile(repo_root, 'common'));
addpath(fullfile(repo_root, 'step1_baseline_c0'));
addpath(fullfile(repo_root, 'step2_advanced_controllers'));

[ctrl_base, mech, plant] = param_init();
Ts = ctrl_base.Ts;
traj = trajectory_reciprocating(7.0, Ts, 0.8, 0.5, 1.2);
N = numel(traj.t);

%% Controller and paper-structure neural-network parameters
ctrl.M_hat = 0.88 * diag([mech.mG_nom, mech.J_alpha_nom]);
ctrl.B_hat = 0.82 * diag([2.0 * plant.b_nom, plant.B_alpha]);
ctrl.K_hat = diag([0.0, 0.92 * plant.K_alpha]);
ctrl.A_hat = diag([1.60 * plant.fc_nom, 0.0]);
ctrl.Gamma = diag([25.0, 18.0]);
ctrl.K1 = diag([40.0, 25.0]);
ctrl.K2_init = [4.0; 1.0];
ctrl.K2_min = [0.05; 0.02];
ctrl.K2_max = [24.0; 10.0];
ctrl.k2_omega = [5.0; 2.0];
ctrl.k2_beta = [1.0; 1.0];
ctrl.phi = [0.025; 0.012];
ctrl.eps_v = 0.01;
ctrl.eps_alpha = 0.01;
ctrl.I_fw_limit = [16000.0; 16000.0];
ctrl.K_aw = 16.0 * eye(2);
ctrl.lambda_aw = 20.0;

% Paper-reported structural parameters: n0=4, tau=0.7, m=3,
% Theta=0.3, Gaussian width=1. The two epsilon values are adapted only
% for the M3508 generalized-force units and are shared by RBFNN/BLNN.
ctrl.nn.n0 = 4;
ctrl.nn.tau = 0.7;
ctrl.nn.m_nearest = 3;
ctrl.nn.theta = 0.3;
ctrl.nn.width = 1.0;
ctrl.nn.max_nodes = 60;
ctrl.nn.rbf_nodes = 60;
ctrl.nn.growth_interval_steps = round(0.020 / Ts);
ctrl.nn.epsilon = [25.0; 18.0];
ctrl.nn.eta = [0.01; 0.01];
ctrl.nn.input_clip = 2.0;
ctrl.nn.weight_limit = [30.0; 15.0];
ctrl.nn.dhat_limit = [80.0; 10.0];
ctrl.nn.gate_scale = 0.08;
ctrl.nn.headroom_fraction = 0.15;
ctrl.nn.input_scales = [ ...
    2.0,  1.0; ...   % qddot
    0.7,  0.20; ...  % qdot
    1.0,  0.020; ... % q
    0.08, 0.010; ... % e
    0.7,  0.20];     % edot

%% Two deterministic validation scenarios
scenarios(1).name = 'Uncertainty';
scenarios(1).Imax = [16000.0; 16000.0];
scenarios(1).delta_m = 3.0;
scenarios(1).d_load = 0.24;
scenarios(1).delta_fric = 0.30;
scenarios(1).Kf_L = mech.Kf;
scenarios(1).Kf_R = 0.95 * mech.Kf;
scenarios(1).disturbance_scale = 1.00;

scenarios(2).name = 'Asymmetric saturation';
scenarios(2).Imax = [6000.0; 4000.0];
scenarios(2).delta_m = 4.0;
scenarios(2).d_load = 0.28;
scenarios(2).delta_fric = 0.45;
scenarios(2).Kf_L = mech.Kf;
scenarios(2).Kf_R = 0.92 * mech.Kf;
scenarios(2).disturbance_scale = 1.15;

net_modes = {'none', 'rbf', 'blnn', 'blnn'};
result_keys = {'none', 'rbf', 'blnn', 'blnn_ca'};
constraint_aware = [false, false, false, true];
method_names = {'No NN', 'RBFNN-60', 'Paper-BLNN', 'CA-BLNN'};
colors = [0.25 0.25 0.25; 0.85 0.33 0.10; ...
          0.49 0.18 0.56; 0.00 0.45 0.74];
results = struct([]);
summary_rows = {};
run_id = 0;

fprintf('BLNN benchmark: %d scenarios x %d methods x %d steps\n', ...
    numel(scenarios), numel(net_modes), N);

for s = 1:numel(scenarios)
    env = scenarios(s);
    fprintf('\nScenario %d/%d: %s\n', s, numel(scenarios), env.name);

    for m = 1:numel(net_modes)
        run_id = run_id + 1;
        fprintf('  %-9s ... ', method_names{m});
        x = zeros(4, 1);
        qddot_meas = zeros(2, 1);
        state.z_aw = zeros(2, 1);
        state.k2 = ctrl.K2_init;
        state.net = nn_compensator_init(net_modes{m}, ctrl.nn);
        state.nn_gate = ones(2, 1);
        ctrl.constraint_aware = constraint_aware(m);

        log_y = zeros(1, N);
        log_alpha = zeros(1, N);
        log_ey = zeros(1, N);
        log_sync = zeros(1, N);
        log_i = zeros(2, N);
        log_sat = false(1, N);
        log_d_true = zeros(2, N);
        log_d_hat = zeros(2, N);
        log_v_nn = zeros(2, N);
        log_nn_gate = zeros(2, N);
        log_severity = zeros(1, N);
        log_nodes = zeros(2, N);
        log_wnorm = zeros(2, N);
        log_k2 = zeros(2, N);
        log_ctrl_time = zeros(1, N);

        for k = 1:N
            q = x(1:2);
            qdot = x(3:4);
            qd = [traj.y(k); 0.0];
            qdot_d = [traj.ydot(k); 0.0];
            qddot_d = [traj.yddot(k); 0.0];

            timer_id = tic;
            [iL, iR, state, info] = controller_c3b_nn_sync_alloc( ...
                q, qdot, qddot_meas, qd, qdot_d, qddot_d, ctrl, ...
                mech.Le, env.Kf_L, env.Kf_R, env.Imax, state, Ts);
            log_ctrl_time(k) = toc(timer_id);

            [x_next, plant_info] = gantry_dynamics_step_uncertain( ...
                x, iL, iR, mech, plant, env, Ts);

            log_y(k) = q(1);
            log_alpha(k) = q(2);
            log_ey(k) = (q(1) - qd(1)) * 1e3;
            log_sync(k) = mech.Le * q(2) * 1e3;
            log_i(:, k) = [iL; iR];
            log_sat(k) = info.is_sat;
            log_d_true(:, k) = plant_info.d_unknown;
            log_d_hat(:, k) = info.d_hat;
            log_v_nn(:, k) = info.v_nn;
            log_nn_gate(:, k) = info.nn_gate;
            log_severity(k) = info.saturation_severity;
            log_nodes(:, k) = info.nn.nodes;
            log_wnorm(:, k) = info.nn.weight_norm;
            log_k2(:, k) = info.k2;

            qddot_meas = plant_info.qddot;
            x = x_next;
        end

        eval_idx = traj.t >= 0.5;
        rmse_y = sqrt(mean(log_ey.^2));
        rmse_sync = sqrt(mean(log_sync.^2));
        max_sync = max(abs(log_sync));
        iae_y = trapz(traj.t, abs(log_ey));
        sat_ratio = 100.0 * mean(log_sat);
        d_rmse = sqrt(mean((log_d_hat(:, eval_idx) - log_d_true(:, eval_idx)).^2, 2));
        warm_idx = max(2, round(0.5 / Ts)):N;
        mean_time_us = 1e6 * mean(log_ctrl_time(warm_idx));

        results(run_id).scenario_id = s;
        results(run_id).scenario = env.name;
        results(run_id).mode = result_keys{m};
        results(run_id).method = method_names{m};
        results(run_id).color = colors(m, :);
        results(run_id).t = traj.t;
        results(run_id).y = log_y;
        results(run_id).alpha = log_alpha;
        results(run_id).ey_mm = log_ey;
        results(run_id).sync_mm = log_sync;
        results(run_id).current = log_i;
        results(run_id).sat = log_sat;
        results(run_id).d_true = log_d_true;
        results(run_id).d_hat = log_d_hat;
        results(run_id).v_nn = log_v_nn;
        results(run_id).nn_gate = log_nn_gate;
        results(run_id).saturation_severity = log_severity;
        results(run_id).nodes = log_nodes;
        results(run_id).weight_norm = log_wnorm;
        results(run_id).k2 = log_k2;
        results(run_id).rmse_y_mm = rmse_y;
        results(run_id).rmse_sync_mm = rmse_sync;
        results(run_id).max_sync_mm = max_sync;
        results(run_id).iae_y_mm_s = iae_y;
        results(run_id).sat_ratio_pct = sat_ratio;
        results(run_id).d_rmse = d_rmse;
        results(run_id).mean_control_time_us = mean_time_us;
        results(run_id).projection_hits = sum(state.net.projection_hits);

        final_nodes = log_nodes(:, end);
        summary_rows(end + 1, :) = {env.name, method_names{m}, ...
            rmse_y, rmse_sync, max_sync, iae_y, sat_ratio, ...
            d_rmse(1), d_rmse(2), final_nodes(1), final_nodes(2), ...
            mean_time_us, sum(state.net.projection_hits)}; %#ok<SAGROW>

        fprintf('RMSE=%.3f mm, sync RMS=%.3f mm, nodes=%d/%d\n', ...
            rmse_y, rmse_sync, final_nodes(1), final_nodes(2));
    end
end

summary = cell2table(summary_rows, 'VariableNames', { ...
    'Scenario', 'Method', 'TrackingRMSE_mm', 'SyncRMS_mm', ...
    'MaxSync_mm', 'TrackingIAE_mm_s', 'SaturationRatio_pct', ...
    'ExplicitDisturbanceComponentRMSE_Force_N', ...
    'ExplicitDisturbanceComponentRMSE_Moment_Nm', ...
    'FinalNodes_Translation', 'FinalNodes_Rotation', ...
    'MeanControlTime_us', 'WeightProjectionHits'});
writetable(summary, fullfile(this_dir, 'blnn_results_summary.csv'));
save(fullfile(this_dir, 'blnn_results.mat'), 'results', 'summary', ...
    'ctrl', 'scenarios', 'traj', 'mech', 'plant');

%% Figure 1: tracking and synchronization errors in both scenarios
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1250 760]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for s = 1:2
    nexttile(2 * s - 1); hold on; grid on; box on;
    for m = 1:4
        r = get_result(results, s, result_keys{m});
        plot(r.t, r.ey_mm, 'LineWidth', 1.2, 'Color', colors(m, :));
    end
    ylabel('Tracking error (mm)');
    title(sprintf('%s: tracking', scenarios(s).name));
    if s == 2, xlabel('Time (s)'); end

    nexttile(2 * s); hold on; grid on; box on;
    for m = 1:4
        r = get_result(results, s, result_keys{m});
        plot(r.t, r.sync_mm, 'LineWidth', 1.2, 'Color', colors(m, :));
    end
    ylabel('Synchronization error (mm)');
    title(sprintf('%s: synchronization', scenarios(s).name));
    if s == 2, xlabel('Time (s)'); end
end
legend(method_names, 'Location', 'southoutside', 'Orientation', 'horizontal');
export_figure(fig, this_dir, 'Figure_1_BLNN_tracking_and_synchronization');

%% Figure 2: effective neural compensation and constraint gate
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 820]);
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
r_rbf = get_result(results, 2, 'rbf');
r_blnn = get_result(results, 2, 'blnn');
r_ca = get_result(results, 2, 'blnn_ca');
labels = {'RBFNN-60', 'Paper-BLNN', 'CA-BLNN'};
for channel = 1:2
    nexttile; hold on; grid on; box on;
    plot(r_rbf.t, r_rbf.v_nn(channel, :), '-', 'Color', colors(2, :), 'LineWidth', 1.0);
    plot(r_blnn.t, r_blnn.v_nn(channel, :), '-', 'Color', colors(3, :), 'LineWidth', 1.2);
    plot(r_ca.t, r_ca.v_nn(channel, :), '-', 'Color', colors(4, :), 'LineWidth', 1.2);
    if channel == 1
        ylabel('Force compensation (N)');
        title('Effective neural compensation under asymmetric saturation');
    else
        ylabel('Moment compensation (N m)');
        legend(labels, 'Location', 'best');
    end
end
nexttile; hold on; grid on; box on;
plot(r_ca.t, r_ca.nn_gate(1, :), 'Color', colors(4, :), 'LineWidth', 1.2);
plot(r_ca.t, double(r_ca.sat), 'k--', 'LineWidth', 1.0);
ylabel('Gate / saturation'); xlabel('Time (s)'); ylim([-0.05, 1.05]);
legend({'CA-BLNN gate', 'Saturation flag'}, 'Location', 'southoutside', ...
    'Orientation', 'horizontal');
export_figure(fig, this_dir, 'Figure_2_BLNN_constraint_aware_compensation');

%% Figure 3: adaptive node growth and weight evolution
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [120 120 1200 720]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for s = 1:2
    r = get_result(results, s, 'blnn_ca');
    nexttile(2 * s - 1); hold on; grid on; box on;
    stairs(r.t, r.nodes(1, :), 'LineWidth', 1.2);
    stairs(r.t, r.nodes(2, :), 'LineWidth', 1.2);
    ylabel('Node count'); title(sprintf('%s: node growth', scenarios(s).name));
    if s == 2, xlabel('Time (s)'); end

    nexttile(2 * s); hold on; grid on; box on;
    plot(r.t, r.weight_norm(1, :), 'LineWidth', 1.2);
    plot(r.t, r.weight_norm(2, :), 'LineWidth', 1.2);
    ylabel('Weight norm'); title(sprintf('%s: adaptive weights', scenarios(s).name));
    if s == 2, xlabel('Time (s)'); end
end
legend({'Translation', 'Rotation'}, 'Location', 'southoutside', ...
    'Orientation', 'horizontal');
export_figure(fig, this_dir, 'Figure_3_BLNN_node_and_weight_evolution');

%% Figure 4: scalar performance comparison
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [140 140 1150 650]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
metric_names = {'TrackingRMSE_mm', 'SyncRMS_mm'};
ylabels = {'Tracking RMSE (mm)', 'Sync RMS (mm)'};
for s = 1:2
    for metric = 1:2
        nexttile((s - 1) * 2 + metric);
        rows = summary(strcmp(summary.Scenario, scenarios(s).name), :);
        values = rows.(metric_names{metric});
        b = bar(values, 'FaceColor', 'flat');
        b.CData = colors;
        grid on; box on;
        set(gca, 'XTick', 1:4, 'XTickLabel', method_names);
        ylabel(ylabels{metric});
        title(scenarios(s).name);
    end
end
export_figure(fig, this_dir, 'Figure_4_BLNN_performance_summary');

close all;
fprintf('\nSaved summary, MAT data, and four publication figures in:\n  %s\n', this_dir);

function r = get_result(results, scenario_id, mode)
    idx = find([results.scenario_id] == scenario_id & ...
        strcmp({results.mode}, mode), 1, 'first');
    assert(~isempty(idx), 'Result not found for scenario %d, mode %s.', scenario_id, mode);
    r = results(idx);
end

function export_figure(fig, out_dir, stem)
    exportgraphics(fig, fullfile(out_dir, [stem '.png']), 'Resolution', 300);
    exportgraphics(fig, fullfile(out_dir, [stem '.pdf']), 'ContentType', 'vector');
end
