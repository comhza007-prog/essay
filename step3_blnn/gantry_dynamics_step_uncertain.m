function [x_next, details] = gantry_dynamics_step_uncertain( ...
    x, iL, iR, mech, plant, env, dt)
%GANTRY_DYNAMICS_STEP_UNCERTAIN RK4 plant with explicit unknown dynamics.
%   This wrapper leaves the shared project plant untouched and adds a known
%   benchmark-only generalized disturbance for NN estimation validation.

    [k1, details] = eval_deriv(x, iL, iR, mech, plant, env);
    [k2, ~] = eval_deriv(x + 0.5 * dt * k1, iL, iR, mech, plant, env);
    [k3, ~] = eval_deriv(x + 0.5 * dt * k2, iL, iR, mech, plant, env);
    [k4, ~] = eval_deriv(x + dt * k3, iL, iR, mech, plant, env);
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end

function [dxdt, details] = eval_deriv(x, iL, iR, mech, plant, env)
    y_dot = x(3);
    alpha_dot = x(4);
    Le = mech.Le;

    vL = y_dot - 0.5 * Le * alpha_dot;
    vR = y_dot + 0.5 * Le * alpha_dot;
    FL = env.Kf_L * iL;
    FR = -env.Kf_R * iR;

    bL = plant.b_nom;
    bR = plant.b_nom * (1.0 + env.delta_fric);
    fcL = plant.fc_nom;
    fcR = plant.fc_nom * (1.0 + env.delta_fric);
    F_fric_L = bL * vL + fcL * tanh(100.0 * vL);
    F_fric_R = bR * vR + fcR * tanh(100.0 * vR);

    M_tot = mech.mG_nom + env.delta_m;
    J_tot = mech.J_alpha_nom + env.delta_m * env.d_load^2;
    coupling = env.delta_m * env.d_load;
    Mq = [M_tot, coupling; coupling, J_tot];

    d_unknown = blnn_unknown_disturbance(x, env);
    generalized_force = [ ...
        FL + FR - F_fric_L - F_fric_R; ...
        0.5 * Le * (FR - FL) - 0.5 * Le * (F_fric_R - F_fric_L) ...
            - plant.K_alpha * x(2) - plant.B_alpha * alpha_dot] + d_unknown;
    qddot = Mq \ generalized_force;
    dxdt = [y_dot; alpha_dot; qddot];

    details.qddot = qddot;
    details.d_unknown = d_unknown;
    details.generalized_force = generalized_force;
    details.Mq = Mq;
end
