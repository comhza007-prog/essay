function r = simulate_v2(cfg,env,method,o)
% Same plant, trajectory, quantization, differentiators and gains as dev_v1.
    t=cfg.traj.t; N=numel(t); x=zeros(4,1); state=[];
    noise=zeros(2,N); delay=0;
    if isfield(cfg,'validation')
        x=cfg.validation.initial_state;
        noise=cfg.validation.rail_noise;
        delay=cfg.validation.delay_steps;
    end
    measurementHistory=zeros(2,N);
    previous=zeros(2,1); velocity=zeros(2,1); acceleration=zeros(2,1);
    av=1-exp(-2*pi*cfg.measurement.velocity_cutoff*cfg.Ts);
    aa=1-exp(-2*pi*cfg.measurement.acceleration_cutoff*cfg.Ts);
    r.t=t; r.q=zeros(2,N); r.current=zeros(2,N); r.request=zeros(2,N);
    r.weight=zeros(2,N); r.nodes=zeros(2,N); r.z_aw=zeros(2,N);
    r.nn=zeros(2,N); r.sigma=zeros(2,N); r.sector_gate=zeros(2,N);
    r.output_gate=zeros(1,N); r.learning_gate=zeros(1,N);
    r.saturated=false(1,N); r.time=zeros(1,N); r.theta=zeros(3,N);
    limits=min(env.Imax(:),cfg.ctrl.I_fw_limit(:));
    for k=1:N
        q=x(1:2); Le=cfg.mech.Le;
        rail=[q(1)-.5*Le*q(2);q(1)+.5*Le*q(2)];
        rail=round((rail+noise(:,k))/cfg.measurement.encoder_resolution)*cfg.measurement.encoder_resolution;
        measurementHistory(:,k)=[mean(rail);(rail(2)-rail(1))/Le];
        qm=measurementHistory(:,max(1,k-delay));
        raw=(qm-previous)/cfg.Ts; previous=qm;
        velocity=velocity+av*(raw-velocity);
        if k==1, prevVelocity=velocity; end
        rawAcceleration=(velocity-prevVelocity)/cfg.Ts; prevVelocity=velocity;
        acceleration=acceleration+aa*(rawAcceleration-acceleration);
        ref=[cfg.traj.y(k),cfg.traj.ydot(k),cfg.traj.yddot(k);0,0,0];
        watch=tic;
        [i,state,info]=controller_m3508_v2(qm,velocity,acceleration,ref,cfg,env,method,state,o);
        r.time(k)=toc(watch);
        ek=env;
        if strcmp(env.name,'Load step')
            if t(k)<3.3, ek.delta_m=4.5; else, ek.delta_m=.5; end
        end
        xnext=gantry_dynamics_step_uncertain(x,i(1),i(2),cfg.mech,cfg.plant,ek,cfg.Ts);
        r.q(:,k)=q; r.current(:,k)=i; r.request(:,k)=info.requested_current;
        r.weight(:,k)=info.nn.weight_norm; r.nodes(:,k)=info.nn.nodes;
        r.nn(:,k)=info.v_nn; r.sigma(:,k)=info.sigma;
        r.sector_gate(:,k)=info.sector_gate; r.z_aw(:,k)=state.z_aw;
        r.output_gate(k)=info.output_gate; r.learning_gate(k)=info.learning_gate;
        r.saturated(k)=info.saturated; r.theta(:,k)=info.theta;
        assert(all(isfinite([xnext;info.theta;info.nn.weight_norm])));
        assert(all(abs(i)<=limits+1e-8));
        x=xnext;
    end
    e=(r.q(1,:)-cfg.traj.y)*1000; sync=cfg.mech.Le*r.q(2,:)*1000;
    timing=sort(r.time(t>=.5)); bound=any(abs(r.current)>=limits-1e-6,1);
    tail=t>=6.5;
    r.metrics=[sqrt(mean(e.^2)),sqrt(mean(sync.^2)),max(abs(sync)),e(end), ...
        sqrt(mean(e(tail).^2)),100*mean(r.saturated),100*mean(bound), ...
        cfg.Ts*sum(sum(abs(r.current-r.request))),max(r.weight(:)), ...
        r.nodes(1,end),r.nodes(2,end),sum(state.net.projection_hits), ...
        mean(timing)*1e6,timing(ceil(.95*numel(timing)))*1e6,max(timing)*1e6];
    r.scenario=env.name; r.method=method; r.options=o;
end
