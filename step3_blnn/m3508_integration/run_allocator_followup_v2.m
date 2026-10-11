function out=run_allocator_followup_v2()
% Resolve the dispatcher confound: compare Base and CA using the SAME
% independent clipper as well as the SAME SyncAlloc from the ablation batch.
    cfg=integration_config();folder=fileparts(mfilename('fullpath'));
    dest=fullfile(folder,'results_allocator_followup_v2');
    assert(~exist(fullfile(dest,'allocator.mat'),'file'),'Preserve existing results.');
    if ~exist(dest,'dir'),mkdir(dest);end
    e=cfg.scenarios(4);e.name='Symmetric limits';e.Imax=[5000;5000];cfg.scenarios(5)=e;
    rows={};runs=cell(5,2);o=options_v2();o.sync_alloc=false;
    for s=1:5
        for m=1:2
            method='Base';if m==2,method='CA-BLNN';end
            fprintf('Allocator follow-up %s / %s / independent clipping\n',cfg.scenarios(s).name,method);
            r=simulate_v2(cfg,cfg.scenarios(s),method,o);runs{s,m}=r;
            rows(end+1,:)=[{cfg.scenarios(s).name,method},num2cell(r.metrics)]; %#ok<AGROW>
        end
    end
    names={'Scenario','Method','TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm', ...
        'FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits','Mean_us','P95_us','Max_us'};
    metrics=cell2table(rows,'VariableNames',names);
    out=struct('cfg',cfg,'metrics',metrics,'runs',{runs}, ...
        'protocol','Post-ablation deterministic allocator diagnosis; no claim of independently validated revised candidate.');
    save(fullfile(dest,'allocator.mat'),'out','-v7.3');
    writetable(metrics,fullfile(dest,'allocator_metrics.csv'));disp(metrics);
end
