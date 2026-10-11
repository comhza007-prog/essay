function [d_hat, state, info] = nn_compensator_step(state, chi_norm, sigma, cfg, Ts, learning_gate)
%NN_COMPENSATOR_STEP One causal online update of NONE/RBFNN/BLNN estimator.
%   chi_norm is 5x2. Each column is [qddot, qdot, q, e, edot]^T after
%   fixed prior normalization. The controller uses v_nn = -d_hat.

    if nargin < 6 || isempty(learning_gate)
        learning_gate = ones(2, 1);
    end
    learning_gate = max(0.0, min(1.0, learning_gate(:)));
    assert(isequal(size(chi_norm), [5, 2]), 'chi_norm must be 5x2.');
    assert(numel(sigma) == 2, 'sigma must have two generalized coordinates.');

    state.step = state.step + 1;
    chi_norm = max(-cfg.input_clip, min(cfg.input_clip, chi_norm));
    d_hat = zeros(2, 1);
    nodes = zeros(2, 1);
    weight_norm = zeros(2, 1);
    novelty = zeros(2, 1);
    grew = false(2, 1);

    for channel = 1:2
        net = state.net(channel);
        x = chi_norm(:, channel).';

        if strcmp(state.mode, 'none')
            nodes(channel) = 0;
            continue;
        end

        if strcmp(state.mode, 'blnn') && learning_gate(channel) >= 0.5 && ...
                mod(state.step, cfg.growth_interval_steps) == 0 && ...
                size(net.centers, 1) < cfg.max_nodes
            distances = sqrt(sum((net.centers - x).^2, 2));
            [~, order] = sort(distances, 'ascend');
            n_near = min(cfg.m_nearest, numel(order));
            c_avg = mean(net.centers(order(1:n_near), :), 1);
            novelty(channel) = norm(x - c_avg, 2);
            if novelty(channel) > cfg.theta
                c_new = c_avg + cfg.tau * (x - c_avg);
                net.centers(end + 1, :) = c_new;
                net.widths(end + 1, 1) = cfg.width;
                net.w_h(end + 1, 1) = 0.0;
                net.w_cos(end + 1, 1) = 0.0;
                net.w_sin(end + 1, 1) = 0.0;
                net.growth_count = net.growth_count + 1;
                state.total_growth = state.total_growth + 1;
                grew(channel) = true;
            end
        end

        squared_distance = sum((net.centers - x).^2, 2);
        h = exp(-squared_distance ./ max(net.widths.^2, eps));

        if strcmp(state.mode, 'rbf')
            d_raw = net.w_h.' * h;
            net.w_h = net.w_h + learning_gate(channel) * Ts * cfg.epsilon(channel) * ...
                (sigma(channel) * h - cfg.eta(channel) * net.w_h);
        else
            e_cos = cos(h);
            e_sin = sin(h);
            d_raw = net.w_h.' * h + net.w_cos.' * e_cos + net.w_sin.' * e_sin;
            gain = learning_gate(channel) * Ts * cfg.epsilon(channel);
            leak = cfg.eta(channel);
            net.w_h = net.w_h + gain * (sigma(channel) * h - leak * net.w_h);
            net.w_cos = net.w_cos + gain * (sigma(channel) * e_cos - leak * net.w_cos);
            net.w_sin = net.w_sin + gain * (sigma(channel) * e_sin - leak * net.w_sin);
        end

        [net.w_h, hit_h] = project_weights(net.w_h, cfg.weight_limit(channel));
        [net.w_cos, hit_c] = project_weights(net.w_cos, cfg.weight_limit(channel));
        [net.w_sin, hit_s] = project_weights(net.w_sin, cfg.weight_limit(channel));
        if hit_h || hit_c || hit_s
            state.projection_hits(channel) = state.projection_hits(channel) + 1;
        end

        d_hat(channel) = max(-cfg.dhat_limit(channel), ...
            min(cfg.dhat_limit(channel), d_raw));
        nodes(channel) = size(net.centers, 1);
        weight_norm(channel) = sqrt(sum(net.w_h.^2) + ...
            sum(net.w_cos.^2) + sum(net.w_sin.^2));
        net.last_novelty = novelty(channel);
        state.net(channel) = net;
    end

    info.nodes = nodes;
    info.weight_norm = weight_norm;
    info.novelty = novelty;
    info.grew = grew;
    info.chi_norm = chi_norm;
    info.learning_gate = learning_gate;
    info.projection_hits = state.projection_hits;
end

function [w, hit] = project_weights(w, limit)
%PROJECT_WEIGHTS Numerical safety projection; inactive for nominal tuning.
    if isempty(w)
        hit = false;
        return;
    end
    projected = max(-limit, min(limit, w));
    hit = any(projected ~= w);
    w = projected;
end
