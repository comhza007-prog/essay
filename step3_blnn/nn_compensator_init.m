function state = nn_compensator_init(mode, cfg)
%NN_COMPENSATOR_INIT Initialize NONE, RBFNN, or paper-structure BLNN.
%   The BLNN implementation follows Eqs. (28)-(34) of Zhou et al.:
%   Gaussian mapping nodes, cosine/sine enhancement nodes, autonomous node
%   growth, and online output-weight adaptation. Two independent networks
%   are used for the translational and rotational generalized coordinates.

    mode = lower(char(mode));
    valid_modes = {'none', 'rbf', 'blnn'};
    assert(any(strcmp(mode, valid_modes)), 'Unsupported network mode: %s', mode);

    state.mode = mode;
    state.step = 0;
    state.total_growth = 0;
    state.input_dim = 5;
    state.channels = 2;
    state.projection_hits = zeros(2, 1);

    empty_net = struct('centers', zeros(0, 5), ...
                       'widths', zeros(0, 1), ...
                       'w_h', zeros(0, 1), ...
                       'w_cos', zeros(0, 1), ...
                       'w_sin', zeros(0, 1), ...
                       'last_novelty', 0.0, ...
                       'growth_count', 0);
    state.net = repmat(empty_net, 1, state.channels);

    for channel = 1:state.channels
        switch mode
            case 'none'
                centers = zeros(0, state.input_dim);
            case 'rbf'
                centers = deterministic_centers(cfg.rbf_nodes, state.input_dim, channel);
            case 'blnn'
                % The paper reports n0=4 and center values spanning [-1, 1].
                % Circular shifts provide four deterministic 5-D initial centers.
                base = [-1.0, -0.5, 0.0, 0.5, 1.0];
                centers = zeros(cfg.n0, state.input_dim);
                for node = 1:cfg.n0
                    centers(node, :) = circshift(base, [0, node - 1]);
                end
        end

        n_nodes = size(centers, 1);
        state.net(channel).centers = centers;
        state.net(channel).widths = cfg.width * ones(n_nodes, 1);
        state.net(channel).w_h = zeros(n_nodes, 1);
        state.net(channel).w_cos = zeros(n_nodes, 1);
        state.net(channel).w_sin = zeros(n_nodes, 1);
    end
end

function centers = deterministic_centers(n_nodes, input_dim, channel)
%DETERMINISTIC_CENTERS Reproducible fixed RBF centers without RNG state.
    centers = zeros(n_nodes, input_dim);
    golden = (sqrt(5.0) - 1.0) / 2.0;
    for node = 1:n_nodes
        for dim = 1:input_dim
            phase = 2.0 * pi * mod(node * (dim + channel) * golden, 1.0);
            centers(node, dim) = 0.90 * sin(phase + 0.17 * channel * dim);
        end
    end
end
