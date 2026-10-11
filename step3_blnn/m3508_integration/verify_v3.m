function checks=verify_v3()
% Allocator KKT and saved-v2 parity, before independent holdout execution.
    cfg=integration_config(); previous=rng;cleanup=onCleanup(@()rng(previous));
    rng(31007,'twister'); KL=cfg.mapping.Kf_L;KR=cfg.mapping.Kf_R;
    Le=cfg.mech.Le;T=[KL,-KR;-.5*Le*KL,-.5*Le*KR];
    bounds=[KL;KR].*[6000;4000];
    for k=1:1000
        request=[100*randn;20*randn];limits=[6000;4000];
        wanted=T\request;clip=max(-limits,min(limits,wanted));
        [weighted,~]=allocator_weighted_v3(request,Le,[KL;KR],limits,1);
        assert(max(abs(weighted-clip))<1e-8);
        w=2;[i,~]=allocator_weighted_v3(request,Le,[KL;KR],limits,w);
        force=[KL*i(1);-KR*i(2)];
        fg=request(1);df=2*request(2)/Le;
        grad=2*[sum(force)-fg-w*(force(2)-force(1)-df); ...
            sum(force)-fg+w*(force(2)-force(1)-df)];
        for j=1:2
            if abs(force(j)-bounds(j))<1e-8,assert(grad(j)<=1e-6);
            elseif abs(force(j)+bounds(j))<1e-8,assert(grad(j)>=-1e-6);
            else,assert(abs(grad(j))<1e-6);end
        end
    end
    checks.weight_one_equals_clip=true;checks.weighted_projection_KKT=true;
    folder=fileparts(mfilename('fullpath'));
    old=load(fullfile(folder,'results_allocator_followup_v2','allocator.mat'));
    for m=1:2
        method='Base';if m==2,method='CA-BLNN';end
        r=simulate_v3(cfg,cfg.scenarios(4),method,options_v3());
        ref=old.out.runs{4,m};
        assert(max(abs(r.q-ref.q),[],'all')<1e-11);
        assert(max(abs(r.current-ref.current),[],'all')<1e-8);
    end
    checks.v2_clip_saved_trajectory_parity=true;disp(checks);
end
