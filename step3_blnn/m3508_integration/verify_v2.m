function checks=verify_v2()
    cfg=integration_config();o=options_v2();
    net=nn_compensator_init('blnn',cfg.ctrl.nn);
    net.net(1).w_cos(:)=1;net.net(2).w_sin(:)=1;
    chi=zeros(5,2);s=[.2;-.3];
    pre=nn_predict_v2(net,chi,cfg.ctrl.nn,true);
    [post,next]=nn_step_v2(net,chi,s,cfg.ctrl.nn,cfg.Ts,zeros(2,1),zeros(2,1),o);
    assert(max(abs(pre-post))<1e-12);
    assert(norm(next.net(1).w_cos)<norm(net.net(1).w_cos));
    assert(size(next.net(1).centers,1)==size(net.net(1).centers,1));
    checks.preview_parity_and_independent_leak=true;
    cfg.ctrl.nn.growth_interval_steps=1;
    chi=2*ones(5,2);
    [~,next]=nn_step_v2(net,chi,s,cfg.ctrl.nn,cfg.Ts,zeros(2,1),ones(2,1),o);
    assert(size(next.net(1).centers,1)>size(net.net(1).centers,1));
    checks.independent_growth_control=true;
    a=[];b=[];
    for k=1:100
        q=[.02*sin(k/11);.001*cos(k/11)];v=[.1*cos(k/11);.002*sin(k/11)];ref=zeros(2,3);
        [ia,a]=controller_m3508_v2(q,v,zeros(2,1),ref,cfg,cfg.scenarios(4),'Base',a,o);
        [ib,b]=controller_m3508_integrated(q,v,zeros(2,1),ref,cfg,cfg.scenarios(4),'Base',b);
        assert(max(abs(ia-ib))<1e-8);assert(max(abs(a.z_aw-b.z_aw))<1e-8);
    end
    checks.base_parity=true;
    disp(checks);fprintf('V2 checks: 3 groups passed.\n');
end
