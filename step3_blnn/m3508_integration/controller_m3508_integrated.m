function [current,state,info] = controller_m3508_integrated( ...
        q,qd_meas,qdd_meas,reference,cfg,env,method,state)
% RLS estimates the slow translational equivalent model. BLNN learns the
% residual through sigma. Both are engineering modules, not original C1.
    Ts = cfg.Ts; c = cfg.ctrl;
    useRls = contains(method,'RLS'); useNn = contains(method,'BLNN');
    if isempty(state)
        state.z_aw = zeros(2,1); state.k2 = c.K2_init;
        state.output_gate = 1; state.previous_force = 0;
        state.previous_position = q(1);
        mode = 'none'; if useNn, mode = 'blnn'; end
        state.net = nn_compensator_init(mode,c.nn);
        if useRls
            initial = [c.M_hat(1,1);c.B_hat(1,1);c.A_hat(1,1)];
            state.filter = rls_filter_svf(cfg.rls.filter_cutoff,Ts);
            state.estimator = rls_estimator_mech(initial,cfg.rls.lambda, ...
                cfg.rls.pe_threshold,Ts);
            state.estimator.theta_min = cfg.rls.theta_min;
            state.estimator.theta_max = cfg.rls.theta_max;
            state.estimator = state.estimator.reset(initial);
            state.theta = initial;
        end
    end
    % At sample k use the estimate computed through k-1, preventing an
    % algebraic loop between NN, model estimate, and actuator allocation.
    if useRls
        c.M_hat(1,1) = state.theta(1);
        c.B_hat(1,1) = state.theta(2);
        c.A_hat(1,1) = state.theta(3);
    end
    e = q-reference(:,1); edot = qd_meas-reference(:,2);
    sigma = edot+c.Gamma*e;
    Sf = [tanh(qd_meas(1)/c.eps_v);tanh(qd_meas(2)/c.eps_alpha)];
    va = c.B_hat*qd_meas+c.K_hat*q+c.A_hat*Sf+c.M_hat*reference(:,3);
    vr = -c.K1*sigma-diag(state.k2)*tanh(sigma./c.phi)-c.M_hat*c.Gamma*edot;
    Le = cfg.mech.Le; KL = cfg.mapping.Kf_L; KR = cfg.mapping.Kf_R;
    T = [KL,-KR;-.5*Le*KL,-.5*Le*KR];
    base = va+vr+T*state.z_aw;
    baseCurrent = T\base;
    limits = min(env.Imax(:),c.I_fw_limit(:));
    chi = [qdd_meas(:).';qd_meas(:).';q(:).';e(:).';edot(:).']./c.nn.input_scales;
    % Preview uses zero learning/growth; discarded state. Re-evaluation can
    % add zero-weight nodes without changing the same-step network output.
    [prediction,~,~] = nn_compensator_step(state.net,chi,sigma,c.nn,Ts,zeros(2,1));
    correction = T\(-prediction);
    safe = constraint_direction_gate(baseCurrent,correction,limits);
    % Fast closing, smooth reopening; the smoothed coefficient never exceeds
    % the same-step feasible coefficient. No delayed safety reaction.
    rising = state.output_gate+Ts/(cfg.gate.release_time+Ts)*(safe-state.output_gate);
    gate = min(safe,rising); state.output_gate = gate;
    headroom = min((limits-abs(baseCurrent))./limits);
    learn = min(gate,max(0,min(1,headroom/cfg.gate.learning_headroom)));
    [dhat,state.net,netInfo] = nn_compensator_step( ...
        state.net,chi,sigma,c.nn,Ts,learn*ones(2,1));
    request = base-gate*dhat;
    requestedCurrent = T\request;
    [current,allocation] = thrust_allocator_sync(request(1),request(2), ...
        Le,KL,KR,env.Imax,c.I_fw_limit);
    deficit = current-requestedCurrent;
    zBefore = state.z_aw;
    state.z_aw = state.z_aw+Ts*(c.K_aw*deficit-c.lambda_aw*state.z_aw);
    k2dot = c.k2_omega.*(abs(sigma)-c.k2_beta.*state.k2);
    state.k2 = max(c.K2_min,min(c.K2_max,state.k2+Ts*k2dot));
    info.theta = [c.M_hat(1,1);c.B_hat(1,1);c.A_hat(1,1)];
    info.pe_active = false; info.rls_residual = 0;
    if useRls
        % Filter previous position and previous ACTUALLY allocated force on
        % the same time index. Never identify from the unattainable request.
        [state.filter,~,vf,af,Ff,Sff] = state.filter.step( ...
            state.previous_position,state.previous_force);
        phi = [af;vf;Sff];
        info.rls_residual = Ff-phi.'*state.theta;
        [state.estimator,state.theta,rlsInfo] = state.estimator.step(Ff,phi);
        info.pe_active = rlsInfo.is_pe_active;
    end
    state.previous_position = q(1); state.previous_force = T(1,:)*current;
    info.sigma = sigma; info.base_current = baseCurrent;
    info.requested_current = requestedCurrent; info.deficit = deficit;
    info.saturated = allocation.is_sat_any;
    info.output_gate = gate; info.learning_gate = learn;
    info.nn = netInfo; info.dhat = dhat; info.v_nn = -gate*dhat;
    info.base_excess = max(0,abs(baseCurrent)-limits);
    info.final_excess = max(0,abs(requestedCurrent)-limits);
    info.base_force = va+vr; info.z_before = zBefore;
    assert(all(info.final_excess<=info.base_excess+1e-7), ...
        'NN correction increased instantaneous infeasibility.');
    assert(all(abs(current)<=limits+1e-8) && all(isfinite([current;state.z_aw;dhat])), ...
        'Invalid command/state or hard current limit violation.');
end
