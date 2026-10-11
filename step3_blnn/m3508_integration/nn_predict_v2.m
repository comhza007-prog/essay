function output = nn_predict_v2(state, chi, cfg, enhancements)
% Read-only prediction. No learning, projection or allocation in preview.
    output = zeros(2,1);
    chi = max(-cfg.input_clip,min(cfg.input_clip,chi));
    if strcmp(state.mode,'none'), return; end
    for k=1:2
        n=state.net(k);
        h=exp(-sum((n.centers-chi(:,k).').^2,2)./max(n.widths.^2,eps));
        raw=n.w_h.'*h;
        if enhancements && ~strcmp(state.mode,'rbf')
            raw=raw+n.w_cos.'*cos(h)+n.w_sin.'*sin(h);
        end
        output(k)=max(-cfg.dhat_limit(k),min(cfg.dhat_limit(k),raw));
    end
end
