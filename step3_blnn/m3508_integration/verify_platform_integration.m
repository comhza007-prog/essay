function checks = verify_platform_integration()
% Meaningful geometry, baseline parity, model initialization, causal feedback.
    cfg=integration_config(); env=cfg.scenarios(4);
    previousRng=rng; cleanup=onCleanup(@()rng(previousRng));
    rng(20261010,'twister'); limits=env.Imax(:);
    for k=1:2000
        b=4*limits.*(rand(2,1)-.5); d=6*limits.*(rand(2,1)-.5);
        g=constraint_direction_gate(b,d,limits);
        assert(g>=0 && g<=1);
        assert(all(max(0,abs(b+g*d)-limits)<=max(0,abs(b)-limits)+1e-8));
        lower=min(-limits,b); upper=max(limits,b);
        assert(all(b+g*d>=lower-1e-8 & b+g*d<=upper+1e-8));
        if g<1-1e-7
            g2=min(1,g+1e-7);
            trial=b+g2*d;
            assert(any(trial<lower-1e-10 | trial>upper+1e-10));
        end
    end
    checks.directional_feasibility_and_maximality=true;
    % NN/RLS off must reproduce the existing controller, including AW and
    % adaptive robust gains, even while the unequal limits are active.
    s=[]; old=[];
    for k=1:100
        q=[.1+.005*sin(k/10);.001*cos(k/10)];
        v=[.03*cos(k/10);.002*sin(k/10)]; ref=[.11,.04,.2;0,0,0];
        if isempty(old)
            old.net=nn_compensator_init('none',cfg.ctrl.nn);
            old.z_aw=zeros(2,1);old.k2=cfg.ctrl.K2_init;
        end
        [i,s,info]=controller_m3508_integrated(q,v,zeros(2,1),ref,cfg,env,'Base',s);
        [il,ir,old,original]=controller_c3b_nn_sync_alloc(q,v,zeros(2,1), ...
            ref(:,1),ref(:,2),ref(:,3),cfg.ctrl,cfg.mech.Le, ...
            cfg.mapping.Kf_L,cfg.mapping.Kf_R,env.Imax,old,cfg.Ts);
        assert(max(abs(i-[il;ir]))<1e-7);
        assert(max(abs(s.z_aw-old.z_aw))<1e-7);
        assert(max(abs(info.base_force-original.v_beta))<1e-7);
    end
    checks.no_network_baseline_parity=true;
    q=zeros(2,1);ref=zeros(2,3);
    [i,s,info]=controller_m3508_integrated(q,q,q,ref,cfg,env,'RLS',[]);
    initial=[cfg.ctrl.M_hat(1,1);cfg.ctrl.B_hat(1,1);cfg.ctrl.A_hat(1,1)];
    assert(max(abs(s.theta-initial))<1e-12);
    assert(max(abs(info.theta-initial))<1e-12);
    assert(abs(s.previous_force-(cfg.mapping.Kf_L*i(1)-cfg.mapping.Kf_R*i(2)))<1e-12);
    checks.shared_initial_model_and_allocated_force=true;
    % Weight output existing before current learning/growth is used. A
    % feasible gate cannot cause an impossible current request.
    [~,s]=controller_m3508_integrated(q,q,q,ref,cfg,env,'CA-BLNN',[]);
    s.net.net(1).w_cos(:)=10; s.net.net(2).w_cos(:)=2;
    for k=1:50
        [~,s,info]=controller_m3508_integrated(q,q,q,[.8,.5,1.2;0,0,0], ...
            cfg,env,'CA-BLNN',s);
        assert(all(info.final_excess<=info.base_excess+1e-7));
    end
    checks.constraint_aware_closed_loop_interface=true;
    save(fullfile(fileparts(mfilename('fullpath')),'platform_integration_checks.mat'),'checks');
    disp(checks);fprintf('Platform integration: 4 groups passed.\n');
end
