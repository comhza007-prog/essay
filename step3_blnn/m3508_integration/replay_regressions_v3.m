function out=replay_regressions_v3()
% Old v2 failure seeds are development diagnostics, not independent holdout.
    folder=fileparts(mfilename('fullpath'));cfg0=integration_config();
    d=load(fullfile(folder,'results_robustness_v2','robustness.mat'));
    dest=fullfile(folder,'results_replay_v3');if ~exist(dest,'dir'),mkdir(dest);end
    assert(~exist(fullfile(dest,'replay.mat'),'file'),'Preserve existing results.');
    rows={};runs={};
    for j=1:20
        if d.out.runs{2,j,2}.metrics(1)<=d.out.runs{2,j,1}.metrics(1),continue;end
        sample=d.out.samples{2,j};cfg=cfg0;cfg.validation=sample.validation;
        for m=1:2
            method='Base';if m==2,method='CA-BLNN';end
            fprintf('Old regression seed %d / clip / %s\n',sample.seed,method);
            r=simulate_v3(cfg,sample.env,method,options_v3());runs{end+1}=r; %#ok<AGROW>
            rows(end+1,:)=[{sample.seed,method,d.out.runs{2,j,m}.metrics(1)},num2cell(r.metrics)]; %#ok<AGROW>
        end
    end
    names={'Seed','Method','OldStrictTrackingRMSE_mm','TrackingRMSE_mm','SyncRMS_mm', ...
        'SyncMax_mm','FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits','Mean_us','P95_us','Max_us'};
    metrics=cell2table(rows,'VariableNames',names);
    out=struct('runs',{runs},'metrics',metrics,'statement','Known development seeds only; not new validation.');
    save(fullfile(dest,'replay.mat'),'out','-v7.3');writetable(metrics,fullfile(dest,'replay_metrics.csv'));disp(metrics);
end
