function out=run_sensitivity_v3(dest)
% One-factor-at-a-time screening. Does not pick a new optimum afterward.
    cfg0=integration_config();folder=fileparts(mfilename('fullpath'));
    if nargin<1,dest=fullfile(folder,'results_sensitivity_v3');end
    assert(~exist(fullfile(dest,'sensitivity.mat'),'file'),'Preserve existing results.');
    if ~exist(dest,'dir'),mkdir(dest);end
    fields={'sector_fraction','theta','tau','width','max_nodes','eta','epsilon'};
    values={[.1 .25 .5],[.2 .3 .5],[.5 .7 .9],[.7 1 1.3],[30 60 90],[.5 1 2],[.5 1 2]};
    rows={};runs={};
    for s=[2 4]
        for p=1:numel(fields)
            for value=values{p}
                cfg=cfg0;o=options_v3();field=fields{p};
                if strcmp(field,'sector_fraction')
                    o.sector_fraction=value;
                elseif strcmp(field,'eta') || strcmp(field,'epsilon')
                    cfg.ctrl.nn.(field)=cfg0.ctrl.nn.(field)*value;
                else
                    cfg.ctrl.nn.(field)=value;
                end
                fprintf('Sensitivity %s / %s = %g\n',cfg.scenarios(s).name,field,value);
                r=simulate_v3(cfg,cfg.scenarios(s),'CA-BLNN',o);runs{end+1}=r; %#ok<AGROW>
                rows(end+1,:)=[{cfg.scenarios(s).name,field,value},num2cell(r.metrics)]; %#ok<AGROW>
            end
        end
    end
    names={'Scenario','Parameter','Value','TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm', ...
        'FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits','Mean_us','P95_us','Max_us'};
    metrics=cell2table(rows,'VariableNames',names);
    out=struct('cfg',cfg0,'metrics',metrics,'runs',{runs}, ...
        'protocol','7 one-factor screens, three levels each, 2 critical scenarios; eta/epsilon values are multipliers.');
    save(fullfile(dest,'sensitivity.mat'),'out','-v7.3');
    writetable(metrics,fullfile(dest,'sensitivity_metrics.csv'));disp(metrics);
end
