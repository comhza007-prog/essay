function out=run_ablation_v2()
% Each single-factor ablation retains every other complete-method setting.
    cfg=integration_config(); folder=fileparts(mfilename('fullpath'));
    dest=fullfile(folder,'results_ablation_v2');
    assert(~exist(fullfile(dest,'ablation.mat'),'file'),'Preserve existing results.');
    if ~exist(dest,'dir'), mkdir(dest); end
    env=cfg.scenarios(4);env.name='Symmetric limits';env.Imax=[5000;5000];
    cfg.scenarios(5)=env;
    labels={'Base','RLS','RBF60-CA','BLNN ungated','Full v2', ...
        'No sector','No output gate','No learning gate','No growth gate', ...
        'Fixed four nodes','No enhancement','No SyncAlloc','RLS+Full v2'};
    methods={'Base','RLS','CA-RBF','CA-BLNN','CA-BLNN','CA-BLNN', ...
        'CA-BLNN','CA-BLNN','CA-BLNN','CA-BLNN','CA-BLNN','CA-BLNN','RLS+CA-BLNN'};
    opt=repmat(options_v2(),1,numel(labels));
    opt(4).sector_guard=false;opt(4).output_gate=false;
    opt(4).learning_gate=false;opt(4).growth_gate=false;
    opt(6).sector_guard=false;opt(7).output_gate=false;
    opt(8).learning_gate=false;opt(9).growth_gate=false;
    opt(10).fixed_nodes=true;opt(11).enhancements=false;
    opt(12).sync_alloc=false;
    rows={};runs=cell(numel(cfg.scenarios),numel(labels));
    for s=1:numel(cfg.scenarios)
        for m=1:numel(labels)
            fprintf('Ablation %s / %s\n',cfg.scenarios(s).name,labels{m});
            r=simulate_v2(cfg,cfg.scenarios(s),methods{m},opt(m));
            rows(end+1,:)=[{cfg.scenarios(s).name,labels{m}},num2cell(r.metrics)]; %#ok<AGROW>
            runs{s,m}=r;
        end
    end
    names={'Scenario','Method','TrackingRMSE_mm','SyncRMS_mm','SyncMax_mm', ...
        'FinalError_mm','TailRMSE_mm','AllocationDeficit_pct','CurrentAtBound_pct', ...
        'Deficit_counts_s','WeightPeak','NodesY','NodesAngle','ProjectionHits', ...
        'Mean_us','P95_us','Max_us'};
    metrics=cell2table(rows,'VariableNames',names);
    out=struct('cfg',cfg,'metrics',metrics,'runs',{runs});
    save(fullfile(dest,'ablation.mat'),'out','-v7.3');
    writetable(metrics,fullfile(dest,'ablation_metrics.csv'));disp(metrics);
    f=figure('Visible','off','Color','w','Position',[100 100 1050 750]);
    selected=[1 3 4 5 6 12];
    for j=selected
        r=runs{4,j};subplot(2,1,1);hold on;
        plot(r.t,(r.q(1,:)-cfg.traj.y)*1e3,'DisplayName',labels{j});
        subplot(2,1,2);hold on;plot(r.t,cfg.mech.Le*r.q(2,:)*1e3,'DisplayName',labels{j});
    end
    subplot(2,1,1);legend('Location','best');ylabel('Tracking error (mm)');grid on;
    subplot(2,1,2);ylabel('Sync error (mm)');xlabel('Time (s)');grid on;
    exportgraphics(f,fullfile(dest,'ablation.png'),'Resolution',180);close(f);
end
