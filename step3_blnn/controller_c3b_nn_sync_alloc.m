function [iL_cmd, iR_cmd, state, info] = controller_c3b_nn_sync_alloc( ...
    q, qdot, qddot_meas, qd, qdot_d, qddot_d, ctrl, ...
    Le, Kf_L, Kf_R, Imax, state, Ts)
%CONTROLLER_C3B_NN_SYNC_ALLOC Paper-structure NN + M3508 constraint adapter.
%   Network modes in state.net.mode:
%     none : no neural compensation
%     rbf  : fixed-node RBFNN comparison
%     blnn : adaptive-width BLNN with enhancement nodes and node growth
%
%   The BLNN block follows the source paper's Eqs. (28)-(34). The original
%   smooth-voltage saturation law is replaced by the project's identified
%   M3508 current limits, dynamic anti-windup, and SyncAlloc. This boundary
%   is intentional and is reported as an engineering adaptation.

    e = q - qd;
    edot = qdot - qdot_d;
    sigma = edot + ctrl.Gamma * e;

    if ~isfield(state, 'z_aw') || isempty(state.z_aw)
        state.z_aw = zeros(2, 1);
    end
    if ~isfield(state, 'k2') || isempty(state.k2)
        state.k2 = ctrl.K2_init(:);
    end
    if ~isfield(state, 'nn_gate') || isempty(state.nn_gate)
        state.nn_gate = ones(2, 1);
    end

    Sf = [tanh(qdot(1) / ctrl.eps_v); tanh(qdot(2) / ctrl.eps_alpha)];
    va = ctrl.B_hat * qdot + ctrl.K_hat * q + ctrl.A_hat * Sf + ...
        ctrl.M_hat * qddot_d;

    k2 = state.k2;
    vr = -ctrl.K1 * sigma - diag(k2) * tanh(sigma ./ ctrl.phi) ...
        - ctrl.M_hat * ctrl.Gamma * edot;

    % Paper input per generalized coordinate: [qddot, qdot, q, e, edot]^T.
    chi_raw = [qddot_meas(:).'; qdot(:).'; q(:).'; e(:).'; edot(:).'];
    chi_norm = chi_raw ./ ctrl.nn.input_scales;

    % The constraint-aware extension evaluates current headroom before the
    % NN term is applied. If the nominal robust/AW request is already near
    % the feasible boundary, NN output and learning are smoothly suppressed.
    T_act = [Kf_L, -Kf_R; -0.5 * Le * Kf_L, -0.5 * Le * Kf_R];
    z_aw = state.z_aw;
    if isfield(ctrl, 'constraint_aware') && ctrl.constraint_aware
        v_nominal = va + vr + T_act * z_aw;
        [iL_nom, iR_nom] = actuator_map_m3508.force_to_current( ...
            v_nominal(1), v_nominal(2), Le, Kf_L, Kf_R);
        if isscalar(Imax)
            Imax_vec = [Imax; Imax];
        else
            Imax_vec = Imax(:);
        end
        Ifw_vec = ctrl.I_fw_limit(:);
        if isscalar(ctrl.I_fw_limit), Ifw_vec = [Ifw_vec; Ifw_vec]; end
        Ieff = min(Imax_vec, Ifw_vec);
        headroom = min((Ieff - abs([iL_nom; iR_nom])) ./ max(Ieff, 1.0));
        gate_scalar = max(0.0, min(1.0, headroom / ctrl.nn.headroom_fraction));
        learning_gate = gate_scalar * ones(2, 1);
    else
        learning_gate = ones(2, 1);
    end
    [d_hat, state.net, nn_info] = nn_compensator_step( ...
        state.net, chi_norm, sigma, ctrl.nn, Ts, learning_gate);
    output_gate = learning_gate;
    v_nn = -(output_gate .* d_hat);
    v_beta = va + v_nn + vr;

    % Paper Eq. (15), projected to a positive engineering-safe interval.
    k2_dot = ctrl.k2_omega .* (abs(sigma) - ctrl.k2_beta .* k2);
    k2_next = k2 + Ts * k2_dot;
    k2_next = max(ctrl.K2_min(:), min(ctrl.K2_max(:), k2_next));
    state.k2 = k2_next;
    state.k2_dot = k2_dot;

    % M3508 dynamic anti-windup and synchronization-priority allocation.
    v_cmd = v_beta + T_act * z_aw;
    [iL_req, iR_req] = actuator_map_m3508.force_to_current( ...
        v_cmd(1), v_cmd(2), Le, Kf_L, Kf_R);
    i_request = [iL_req; iR_req];

    [i_alloc, alloc_info] = thrust_allocator_sync( ...
        v_cmd(1), v_cmd(2), Le, Kf_L, Kf_R, Imax, ctrl.I_fw_limit);

    if isscalar(Imax)
        Imax_vec = [Imax; Imax];
    else
        Imax_vec = Imax(:);
    end
    i_actual = max(-Imax_vec, min(Imax_vec, i_alloc));
    iL_cmd = i_actual(1);
    iR_cmd = i_actual(2);

    Delta_i_alloc = i_alloc - i_request;
    Delta_i_total = i_actual - i_request;
    z_aw_dot = ctrl.K_aw * Delta_i_alloc - ctrl.lambda_aw * z_aw;
    state.z_aw = z_aw + Ts * z_aw_dot;
    state.z_aw_dot = z_aw_dot;

    if isfield(ctrl, 'constraint_aware') && ctrl.constraint_aware
        severity = norm(Delta_i_alloc, inf) / max(norm(i_request, inf), 1.0);
        state.nn_gate = learning_gate;
    else
        severity = 0.0;
        state.nn_gate = ones(2, 1);
    end

    info.e = e;
    info.edot = edot;
    info.sigma = sigma;
    info.va = va;
    info.vr = vr;
    info.v_nn = v_nn;
    info.d_hat = d_hat;
    info.v_beta = v_beta;
    info.v_cmd = v_cmd;
    info.i_request = i_request;
    info.i_actual = i_actual;
    info.Delta_i_alloc = Delta_i_alloc;
    info.Delta_i_total = Delta_i_total;
    info.is_sat = alloc_info.is_sat_any || any(abs(Delta_i_total) > 1e-6);
    info.alloc_info = alloc_info;
    info.z_aw = z_aw;
    info.k2 = k2;
    info.k2_dot = k2_dot;
    info.chi_raw = chi_raw;
    info.nn = nn_info;
    info.nn_gate = output_gate;
    info.saturation_severity = severity;
end
