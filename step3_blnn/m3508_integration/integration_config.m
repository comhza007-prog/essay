function cfg = integration_config()
% M3508 engineering benchmark; all tuning shared across the four methods.
    folder = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(folder));
    addpath(folder, fileparts(folder), fullfile(root,'step1_baseline_c0'), ...
        fullfile(root,'step2_advanced_controllers'), fullfile(root,'step3_adaptive_rls'));
    [base,cfg.mech,cfg.plant] = param_init();
    cfg.Ts = base.Ts;
    cfg.traj = trajectory_reciprocating(7, cfg.Ts, .8, .5, 1.2);
    mech = cfg.mech; plant = cfg.plant;
    c.M_hat = .88*diag([mech.mG_nom,mech.J_alpha_nom]);
    c.B_hat = .82*diag([2*plant.b_nom,plant.B_alpha]);
    c.K_hat = diag([0,.92*plant.K_alpha]);
    c.A_hat = diag([1.60*plant.fc_nom,0]);
    c.Gamma = diag([25,18]); c.K1 = diag([40,25]);
    c.K2_init = [4;1]; c.K2_min = [.05;.02]; c.K2_max = [24;10];
    c.k2_omega = [5;2]; c.k2_beta = [1;1]; c.phi = [.025;.012];
    c.eps_v = .01; c.eps_alpha = .01;
    c.I_fw_limit = [16000;16000]; c.K_aw = 16*eye(2); c.lambda_aw = 20;
    c.constraint_aware = false;
    c.nn.n0 = 4; c.nn.tau = .7; c.nn.m_nearest = 3;
    c.nn.theta = .3; c.nn.width = 1;
    c.nn.max_nodes = 60; % Engineering resource limit, not a published cap.
    c.nn.rbf_nodes = 60; c.nn.growth_interval_steps = round(.020/cfg.Ts);
    c.nn.epsilon = [25;18]; c.nn.eta = [.01;.01];
    c.nn.input_clip = 2; c.nn.weight_limit = [30;15];
    c.nn.dhat_limit = [80;10]; c.nn.headroom_fraction = .15;
    c.nn.input_scales = [2,1;.7,.20;1,.020;.08,.010;.7,.20];
    cfg.ctrl = c;
    cfg.measurement.velocity_cutoff = 50;
    cfg.measurement.acceleration_cutoff = 20;
    cfg.measurement.encoder_resolution = mech.dy_ecd;
    cfg.gate.release_time = .030;
    cfg.gate.learning_headroom = .15;
    cfg.rls.theta_min = [8;30;4];
    cfg.rls.theta_max = [24;120;30];
    cfg.rls.filter_cutoff = 10;
    cfg.rls.lambda = .995; cfg.rls.pe_threshold = 1e-4;
    % Prior bounds explicitly expanded from Step3A: fixed initial model is
    % 11.528 kg / 57.4 N*s/m / 12.8 N. Reset retains the same initial model.
    names = {'Nominal','Uncertainty','Load step','Asymmetric limits'};
    for k = 1:4
        env.name = names{k}; env.Imax = [16000;16000];
        env.delta_m = 0; env.d_load = 0; env.delta_fric = 0;
        env.Kf_L = mech.Kf; env.Kf_R = mech.Kf;
        env.disturbance_scale = 0;
        cfg.scenarios(k) = env;
    end
    cfg.scenarios(2).delta_m = 3; cfg.scenarios(2).d_load = .24;
    cfg.scenarios(2).delta_fric = .30;
    cfg.scenarios(2).Kf_R = .95*mech.Kf;
    cfg.scenarios(2).disturbance_scale = 1;
    cfg.scenarios(3).delta_m = 4.5; % 4.5 -> 0.5 kg at rest, t=3.3 s.
    cfg.scenarios(4).delta_m = 4; cfg.scenarios(4).d_load = .28;
    cfg.scenarios(4).delta_fric = .45;
    cfg.scenarios(4).Kf_R = .92*mech.Kf;
    cfg.scenarios(4).disturbance_scale = 1.15;
    cfg.scenarios(4).Imax = [6000;4000];
    % Estimated actuator gains are common prior values, never plant truth.
    cfg.mapping.Kf_L = mech.Kf; cfg.mapping.Kf_R = mech.Kf;
    cfg.methods = {'Base','RLS','CA-BLNN','RLS+CA-BLNN'};
    cfg.result_dir = fullfile(folder,'results_dev_v1');
end
