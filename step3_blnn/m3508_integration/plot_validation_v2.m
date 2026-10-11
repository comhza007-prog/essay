function plot_validation_v2()
    folder=fileparts(mfilename('fullpath'));
    d=load(fullfile(folder,'results_robustness_v2','robustness.mat'));
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
            xlabel('Base (mm)');ylabel('CA-BLNN v2 (mm)');grid on;axis square;
            labels={'Tracking RMSE','Sync RMS','Absolute final error'};
            title({titles{s},labels{k}});
        end
    end
    exportgraphics(f,fullfile(folder,'results_robustness_v2','paired_comparison.png'),'Resolution',180);close(f);
    d=load(fullfile(folder,'results_sensitivity_v2','sensitivity.mat'));m=d.out.metrics;
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
    exportgraphics(f,fullfile(folder,'results_sensitivity_v2','sensitivity.png'),'Resolution',180);close(f);
end
