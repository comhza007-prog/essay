function out = run_recovery_v2()
% Controlled 20-run diagnosis. Never overwrite dev_v1 results.
    cfg=integration_config(); dirpath=fullfile(fileparts(mfilename('fullpath')),'results_recovery_v2');
    assert(~exist(fullfile(dirpath,'recovery.mat'),'file'),'Existing result: preserve it.');
    if ~exist(dirpath,'dir'), mkdir(dirpath); end
    labels={'Base','Legacy CA','Leak only','Sector only','Full v2'};
    options=repmat(options_v2(),1,5);
    options(1).independent_leak=false;options(1).sector_guard=false;
    options(2)=options(1);
    options(3).sector_guard=false;
    options(4).independent_leak=false;
    rows={}; runs=cell(4,5);
    for s=1:4
        for m=1:5
            fprintf('%s / %s\n',cfg.scenarios(s).name,labels{m});
            method='CA-BLNN'; if m==1, method='Base'; end
            r=simulate_v2(cfg,cfg.scenarios(s),method,options(m)); runs{s,m}=r;
            rows(end+1,:)=[{cfg.scenarios(s).name,labels{m}},num2cell(r.metrics)]; %#ok<AGROW>
        end
    end
    names={'Scenario','Method','TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm', ...
        'FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits', ...
        'Mean_us','P95_us','Max_us'};
    metrics=cell2table(rows,'VariableNames',names);
    old=load(fullfile(fileparts(mfilename('fullpath')),'results_dev_v1','platform_comparison.mat'));
    for s=1:4
        for m=1:2
            j=(s-1)*4+1; if m==2, j=j+2; end
            assert(max(abs(runs{s,m}.q-old.report.runs(j).q),[],'all')<1e-11,'Baseline parity failed.');
            assert(max(abs(runs{s,m}.current-old.report.runs(j).current),[],'all')<1e-6);
        end
    end
    out=struct('cfg',cfg,'metrics',metrics,'runs',{runs},'legacy_parity_passed',true);
    save(fullfile(dirpath,'recovery.mat'),'out','-v7.3');
    writetable(metrics,fullfile(dirpath,'recovery_metrics.csv'));
    disp(metrics);
    f=figure('Visible','off','Color','w');
    for m=[1,2,5]
        a=runs{4,m};
        subplot(2,1,1);hold on;plot(a.t,(a.q(1,:)-cfg.traj.y)*1e3,'DisplayName',labels{m});
        subplot(2,1,2);hold on;plot(a.t,cfg.mech.Le*a.q(2,:)*1e3,'DisplayName',labels{m});
    end
    subplot(2,1,1);ylabel('Tracking error (mm)');legend('Location','best');grid on;
    subplot(2,1,2);ylabel('Sync error (mm)');xlabel('Time (s)');grid on;
    exportgraphics(f,fullfile(dirpath,'recovery.png'),'Resolution',180);close(f);
end
