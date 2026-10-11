function plot_validation_v3(resultRoot)
    folder=fileparts(mfilename('fullpath'));
    if nargin>0,folder=resultRoot;end
    d=load(fullfile(folder,'results_holdout_v3','holdout.mat'));
    f=figure('Visible','off','Color','w','Position',[100 100 1050 760]);
    titles={'Parameter uncertainty','Asymmetric current limits'};
    for s=1:2
        a=zeros(20,3);b=a;
        for j=1:20
            v=d.out.runs{s,j,1}.metrics;a(j,:)=[v(1),v(2),abs(v(4))];
            v=d.out.runs{s,j,2}.metrics;b(j,:)=[v(1),v(2),abs(v(4))];
        end
        for k=1:3
            subplot(2,3,(s-1)*3+k);hold on;
            plot(a(:,k),b(:,k),'o','MarkerSize',5,'DisplayName','Paired trial');
            lim=max([a(:,k);b(:,k)])*1.05;plot([0 lim],[0 lim],'k--');
            xlabel('Base (mm)');ylabel('CA-BLNN v3 (mm)');grid on;axis square;
            labels={'Tracking RMSE','Sync RMS','Absolute final error'};
            title({titles{s},labels{k}});
        end
    end
    exportgraphics(f,fullfile(folder,'results_holdout_v3','paired_comparison.png'),'Resolution',180);close(f);
    d=load(fullfile(folder,'results_sensitivity_v3','sensitivity.mat'));m=d.out.metrics;
    fields=unique(m.Parameter,'stable');
    f=figure('Visible','off','Color','w','Position',[100 100 1200 650]);
    for j=1:numel(fields)
        for s=1:2
            mask=strcmp(m.Parameter,fields{j});
            if s==1,mask=mask & strcmp(m.Scenario,'Uncertainty');
            else,mask=mask & strcmp(m.Scenario,'Asymmetric limits');end
            subplot(2,7,(s-1)*7+j);plot(m.Value(mask),m.TrackingRMSE_mm(mask),'-o');
            title(fields{j},'Interpreter','none');grid on;
            xlabel('Value / multiplier');
            if j==1,ylabel({titles{s},'Tracking RMSE (mm)'});end
        end
    end
    exportgraphics(f,fullfile(folder,'results_sensitivity_v3','sensitivity.png'),'Resolution',180);close(f);
    % Redraw saved ablation with explicit distinct colors and dashed strict
    % allocator: no numerical re-run, and no ambiguous duplicate blue lines.
    d=load(fullfile(folder,'results_ablation_v3','ablation.mat'));
    selected=[1 3 4 5 6 12];labels={'Base','RBF60-CA','BLNN ungated', ...
        'CA-BLNN v3','No sector','Strict SyncAlloc'};
    colors=[.15 .45 .75;.85 .35 .1;.75 .6 .1;.55 .15 .7;.1 .6 .3;.15 .15 .15];
    f=figure('Visible','off','Color','w','Position',[100 100 1100 760]);
    for k=1:numel(selected)
        r=d.out.runs{4,selected(k)};style='-';if k==6,style='--';end
        subplot(2,1,1);hold on;
        plot(r.t,1e3*(r.q(1,:)-d.out.cfg.traj.y),style,'Color',colors(k,:), ...
            'LineWidth',1.2,'DisplayName',labels{k});
        subplot(2,1,2);hold on;
        plot(r.t,1e3*d.out.cfg.mech.Le*r.q(2,:),style,'Color',colors(k,:), ...
            'LineWidth',1.2,'DisplayName',labels{k});
    end
    subplot(2,1,1);legend('Location','northwest');ylabel('Tracking error (mm)');grid on;
    subplot(2,1,2);ylabel('Sync error (mm)');xlabel('Time (s)');grid on;
    exportgraphics(f,fullfile(folder,'results_ablation_v3','ablation.png'),'Resolution',220);close(f);
    r=d.out.runs{4,5};
    f=figure('Visible','off','Color','w','Position',[100 100 1100 850]);
    subplot(4,1,1);hold on;plot(r.t,r.current(1,:),'DisplayName','Left applied');
    plot(r.t,r.current(2,:),'DisplayName','Right applied');
    yline(6000,'--','HandleVisibility','off');yline(-6000,'--','HandleVisibility','off');
    yline(4000,':','HandleVisibility','off');yline(-4000,':','HandleVisibility','off');
    title('Asymmetric limits: left +/-6000, right +/-4000 counts');
    ylabel('Current counts');legend('Location','best');grid on;
    subplot(4,1,2);plot(r.t,r.nodes);ylabel('Nodes');legend('Translation','Rotation');grid on;
    subplot(4,1,3);plot(r.t,r.weight);ylabel('Weight norm');legend('Translation','Rotation');grid on;
    subplot(4,1,4);plot(r.t,[r.output_gate;r.learning_gate;r.sector_gate]);
    ylabel('Gate coefficient');xlabel('Time (s)');ylim([-.05 1.05]);
    legend('Output','Learning/growth','Sector y','Sector angle','Location','best');grid on;
    exportgraphics(f,fullfile(folder,'results_ablation_v3','candidate_diagnostics.png'),'Resolution',220);close(f);
end
