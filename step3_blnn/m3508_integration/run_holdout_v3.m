function out=run_holdout_v3(dest)
% 20 paired trials per critical scenario. No data-dependent tuning or seeds.
    cfg0=integration_config();folder=fileparts(mfilename('fullpath'));
    if nargin<1,dest=fullfile(folder,'results_holdout_v3');end
    assert(~exist(fullfile(dest,'holdout.mat'),'file'),'Preserve existing results.');
    if ~exist(dest,'dir'),mkdir(dest);end
    previous=rng;cleanup=onCleanup(@()rng(previous));
    seeds=2026102001+(0:19); scen=[2 4]; rows={};samples=cell(2,20);runs=cell(2,20,2);
    names={'Scenario','Seed','Method','TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm', ...
        'FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits','Mean_us','P95_us','Max_us'};
    for s=1:2
        for trial=1:20
            seed=seeds(trial);rng(seed,'twister');cfg=cfg0;env=cfg.scenarios(scen(s));
            % Shared plant prior uncertainty, measurement noise and delay.
            env.delta_m=env.delta_m*(.8+.4*rand);
            env.d_load=env.d_load*(.9+.2*rand);
            env.delta_fric=env.delta_fric*(.8+.4*rand);
            env.Kf_L=env.Kf_L*(.95+.1*rand);env.Kf_R=env.Kf_R*(.95+.1*rand);
            env.disturbance_scale=env.disturbance_scale*(.8+.4*rand);
            cfg.validation.initial_state=[0;(.2*rand- .1)*1e-3;0;0];
            cfg.validation.delay_steps=randi([0 2]);
            cfg.validation.rail_noise=2*cfg.measurement.encoder_resolution*randn(2,numel(cfg.traj.t));
            samples{s,trial}=struct('env',env,'validation',cfg.validation,'seed',seed);
            for m=1:2
                method='Base';if m==2,method='CA-BLNN';end
                fprintf('Paired %s / seed %d / %s\n',env.name,seed,method);
                r=simulate_v3(cfg,env,method,options_v3());runs{s,trial,m}=r;
                rows(end+1,:)=[{env.name,seed,method},num2cell(r.metrics)]; %#ok<AGROW>
            end
        end
    end
    metrics=cell2table(rows,'VariableNames',names);
    summaries={};
    fields={'TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm','AbsFinalError_mm', ...
        'TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct'};
    for s=1:2
        base=zeros(20,7);full=base;
        for trial=1:20
            for m=1:2
                v=runs{s,trial,m}.metrics;
                vals=[v(1:3),abs(v(4)),v(5:7)];
                if m==1,base(trial,:)=vals;else,full(trial,:)=vals;end
            end
        end
        for k=1:7
            delta=full(:,k)-base(:,k);sem=std(delta)/sqrt(20);
            % Two-sided 95% t interval, df=19. These are paired differences,
            % not confidence intervals on a single noiseless run.
            ci=mean(delta)+[-1 1]*2.093024054*sem;
            summaries(end+1,:)={cfg0.scenarios(scen(s)).name,fields{k}, ...
                mean(base(:,k)),std(base(:,k)),mean(full(:,k)),std(full(:,k)), ...
                mean(delta),ci(1),ci(2),max(full(:,k)),sum(delta<0)}; %#ok<AGROW>
        end
    end
    summary=cell2table(summaries,'VariableNames',{'Scenario','Metric','BaseMean','BaseSD', ...
        'V3Mean','V3SD','PairedDeltaMean','DeltaCI95Lower','DeltaCI95Upper','V3Worst','ImprovedTrials'});
    out=struct('cfg',cfg0,'seeds',seeds,'samples',{samples},'runs',{runs}, ...
        'metrics',metrics,'paired_summary',summary, ...
        'protocol','Frozen clip/AW1 candidate; 20 unseen predeclared shared trials per scenario; 2 encoder-LSB noise; 0-2 ms delay; sampled plant prior uncertainty.');
    save(fullfile(dest,'holdout.mat'),'out','-v7.3');
    writetable(metrics,fullfile(dest,'trial_metrics.csv'));
    writetable(summary,fullfile(dest,'paired_summary.csv'));disp(summary);
end
