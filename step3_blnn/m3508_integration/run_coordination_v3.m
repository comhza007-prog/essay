function out=run_coordination_v3()
% 24 factorial cases. Same AW setting is used by Base and CA in each cell.
    cfg=integration_config();folder=fileparts(mfilename('fullpath'));
    dest=fullfile(folder,'results_coordination_v3');
    assert(~exist(fullfile(dest,'coordination.mat'),'file'),'Preserve old results.');
    if ~exist(dest,'dir'),mkdir(dest);end
    e=cfg.scenarios(4);e.name='Symmetric limits';e.Imax=[5000;5000];cfg.scenarios(5)=e;
    allocators={'clip','weighted','strict'};rows={};runs={};
    for s=[4 5]
        for a=1:3
            for aw=[0 1]
                for m=1:2
                    o=options_v3();o.allocator=allocators{a};o.aw_scale=aw;
                    method='Base';if m==2,method='CA-BLNN';end
                    fprintf('Coordination %s / %s / AW %g / %s\n',cfg.scenarios(s).name,o.allocator,aw,method);
                    r=simulate_v3(cfg,cfg.scenarios(s),method,o);runs{end+1}=r; %#ok<AGROW>
                    rows(end+1,:)=[{cfg.scenarios(s).name,o.allocator,aw,method},num2cell(r.metrics)]; %#ok<AGROW>
                end
            end
        end
    end
    names={'Scenario','Allocator','AWScale','Method','TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm', ...
        'FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits','Mean_us','P95_us','Max_us'};
    metrics=cell2table(rows,'VariableNames',names);
    out=struct('cfg',cfg,'runs',{runs},'metrics',metrics);
    save(fullfile(dest,'coordination.mat'),'out','-v7.3');
    writetable(metrics,fullfile(dest,'coordination_metrics.csv'));disp(metrics);
end
