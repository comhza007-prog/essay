function report = run_platform_comparison()
%RUN_PLATFORM_COMPARISON Fair engineering comparison on the dual-M3508 model.
% Methods: Base, RLS, CA-BLNN, and RLS+CA-BLNN. Each method sees the same
% trajectory, encoder quantization, disturbance, plant and actuator limits.
% This is the platform study for the thesis; it is not source-paper data.
    cfg=integration_config();
    if ~exist(cfg.result_dir,'dir'), mkdir(cfg.result_dir); end
    T=cfg.traj.t; N=numel(T); rows={}; allRuns=struct([]); runNo=0;
    for s=1:numel(cfg.scenarios)
        for m=1:numel(cfg.methods)
            env=cfg.scenarios(s); method=cfg.methods{m};
            runNo=runNo+1; fprintf('%s / %s\n',env.name,method);
            x=zeros(4,1); state=[]; qprev=zeros(2,1); qdotm=zeros(2,1); qddm=zeros(2,1);
            qhist=zeros(2,N); qdhist=zeros(2,N); ihist=zeros(2,N); shist=zeros(1,N);
            ghist=zeros(1,N); lhist=zeros(1,N); whist=zeros(2,N); nodes=zeros(2,N);
            rlsResidual=zeros(1,N); pe=false(1,N); time=zeros(1,N);
            saturated=false(1,N); requested=zeros(2,N); theta=zeros(3,N);
            projection=zeros(1,N); currentAtBound=false(1,N);
            velocityAlpha=1-exp(-2*pi*cfg.measurement.velocity_cutoff*cfg.Ts);
            accelAlpha=1-exp(-2*pi*cfg.measurement.acceleration_cutoff*cfg.Ts);
            for k=1:N
                % Encoder quantization + causal differentiators. The second
                % derivative is filtered by a first-order causal pole.
                q=x(1:2); Le=cfg.mech.Le;
                rail=[q(1)-.5*Le*q(2);q(1)+.5*Le*q(2)];
                rail=round(rail/cfg.measurement.encoder_resolution)*cfg.measurement.encoder_resolution;
                qm=[mean(rail);(rail(2)-rail(1))/Le];
                qdotRaw=(qm-qprev)/cfg.Ts; qprev=qm;
                qdotm=qdotm+velocityAlpha*(qdotRaw-qdotm);
                % qddm is the filtered measurement; update from velocity history.
                if k==1, qdotPrev=qdotm; end
                qddRaw=(qdotm-qdotPrev)/cfg.Ts; qdotPrev=qdotm;
                qddm=qddm+accelAlpha*(qddRaw-qddm);
                ref=[cfg.traj.y(k),cfg.traj.ydot(k),cfg.traj.yddot(k);0,0,0];
                timer=tic;
                [i,state,info]=controller_m3508_integrated(qm,qdotm,qddm,ref,cfg,env,method,state);
                time(k)=toc(timer);
                [xnext,~]=plant_step(x,i(1),i(2),cfg,env,T(k));
                qhist(:,k)=q; qdhist(:,k)=[cfg.traj.y(k);0]; ihist(:,k)=i;
                shist(k)=cfg.mech.Le*q(2)*1e3; ghist(k)=info.output_gate;
                lhist(k)=info.learning_gate; whist(:,k)=info.nn.weight_norm;
                nodes(:,k)=info.nn.nodes; rlsResidual(k)=info.rls_residual; pe(k)=info.pe_active;
                saturated(k)=info.saturated; requested(:,k)=info.requested_current;
                theta(:,k)=info.theta; projection(k)=sum(info.nn.projection_hits);
                currentAtBound(k)=any(abs(i)>=min(env.Imax(:),cfg.ctrl.I_fw_limit(:))-1e-6);
                assert(all(isfinite([xnext;info.nn.weight_norm;info.theta])));
                x=xnext;
            end
            idx=true(size(T)); timingIdx=T>=.5; ey=(qhist(1,:)-qdhist(1,:))*1e3;
            metrics=struct('tracking_rmse_mm',sqrt(mean(ey(idx).^2)), ...
                'tracking_max_mm',max(abs(ey(idx))), ...
                'sync_rms_mm',sqrt(mean(shist(idx).^2)), ...
                'sync_max_mm',max(abs(shist(idx))), ...
                'saturation_pct',100*mean(saturated(idx)));
            metrics.current_at_bound_pct=100*mean(currentAtBound(idx));
            metrics.deficit_count_s=cfg.Ts*sum(sum(abs(ihist-requested)));
            metrics.final_nodes_translation=nodes(1,end); metrics.final_nodes_rotation=nodes(2,end);
            metrics.weight_norm_peak=max(whist(:)); metrics.weight_projection_hits=projection(end);
            metrics.mean_control_us=1e6*mean(time(timingIdx));
            sortedTime=sort(time(timingIdx));
            metrics.p95_control_us=1e6*sortedTime(ceil(.95*numel(sortedTime)));
            metrics.max_control_us=1e6*max(time(timingIdx)); metrics.rls_residual_rms=sqrt(mean(rlsResidual(idx).^2));
            metrics.pe_active_pct=100*mean(pe(idx));
            rows(end+1,:)={env.name,method,metrics.tracking_rmse_mm,metrics.tracking_max_mm, ...
                metrics.sync_rms_mm,metrics.sync_max_mm,metrics.saturation_pct, ...
                metrics.final_nodes_translation,metrics.final_nodes_rotation,metrics.weight_norm_peak, ...
                metrics.mean_control_us,metrics.p95_control_us,metrics.max_control_us, ...
                metrics.rls_residual_rms,metrics.pe_active_pct,metrics.weight_projection_hits, ...
                metrics.current_at_bound_pct,metrics.deficit_count_s}; %#ok<AGROW>
            allRuns(runNo).scenario=env.name; allRuns(runNo).method=method;
            allRuns(runNo).t=T; allRuns(runNo).q=qhist; allRuns(runNo).qd=qdhist;
            allRuns(runNo).sync_mm=shist; allRuns(runNo).current=ihist; allRuns(runNo).gate=ghist;
            allRuns(runNo).learning_gate=lhist; allRuns(runNo).nodes=nodes;
            allRuns(runNo).weight_norm=whist; allRuns(runNo).metrics=metrics;
            allRuns(runNo).requested_current=requested; allRuns(runNo).saturated=saturated;
            allRuns(runNo).theta=theta; allRuns(runNo).pe_active=pe;
            allRuns(runNo).rls_residual=rlsResidual; allRuns(runNo).control_time=time;
        end
    end
    vars={'Scenario','Method','TrackingRMSE_mm','TrackingMax_mm','SyncRMS_mm','SyncMax_mm', ...
        'Saturation_pct','FinalNodesTranslation','FinalNodesRotation','WeightNormPeak', ...
        'MeanControl_us','P95Control_us','MaxControl_us','RLSResidualRMS','PEActive_pct', ...
        'WeightProjectionHits','CurrentAtBound_pct','Deficit_counts_s'};
    metricsTable=cell2table(rows,'VariableNames',vars);
    writetable(metricsTable,fullfile(cfg.result_dir,'platform_comparison_metrics.csv'));
    report=struct('cfg',cfg,'metrics',metricsTable,'runs',allRuns, ...
        'statement','Platform engineering benchmark; no original-paper numerical claim.');
    save(fullfile(cfg.result_dir,'platform_comparison.mat'),'report','-v7.3');
    for s=1:numel(cfg.scenarios)
        f=figure('Visible','off','Color','w','Position',[100,100,1000,850]);
        for m=1:numel(cfg.methods)
            r=allRuns((s-1)*numel(cfg.methods)+m);
            subplot(3,1,1);hold on;plot(T,(r.q(1,:)-r.qd(1,:))*1e3,'DisplayName',r.method);
            ylabel('Tracking error (mm)');grid on;
            subplot(3,1,2);hold on;plot(T,r.sync_mm,'DisplayName',r.method);
            ylabel('Synchronization error (mm)');grid on;
            subplot(3,1,3);hold on;plot(T,r.gate,'DisplayName',r.method);
            ylabel('Output gate');xlabel('Time (s)');grid on;
        end
        subplot(3,1,1);legend('Location','best');title(cfg.scenarios(s).name);
        exportgraphics(f,fullfile(cfg.result_dir,sprintf('scenario_%d.png',s)),'Resolution',200);
        close(f);
    end
    disp(metricsTable);
end

function [xnext,info]=plant_step(x,iL,iR,cfg,env,t)
    % Optional load-step scenario: 4.5 kg added before 3.3 s, 0.5 kg after.
    envk=env;
    if strcmp(env.name,'Load step')
        if t<3.3, envk.delta_m=4.5; else, envk.delta_m=.5; end
    end
    [xnext,info]=gantry_dynamics_step_uncertain(x,iL,iR,cfg.mech,cfg.plant,envk,cfg.Ts);
end
