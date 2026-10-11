function out=run_resource_check_v2()
% Desktop wall-clock resource diagnostic, NOT MCU or hard real-time proof.
% Run after other simulation processes end. Exercise maximum 60+60 nodes.
    cfg=integration_config();folder=fileparts(mfilename('fullpath'));
    dest=fullfile(folder,'results_resources_v2');
    assert(~exist(fullfile(dest,'resources.mat'),'file'),'Preserve existing results.');
    if ~exist(dest,'dir'),mkdir(dest);end
    cfg.ctrl.nn.n0=60;o=options_v2();o.fixed_nodes=true;
    rows={};allTimes=cell(1,3);
    for trial=1:3
        state=[];times=zeros(1,11000);
        for k=1:numel(times)
            q=[.1+.001*sin(k/100);1e-4*cos(k/100)];v=[.03;1e-3];
            ref=[.11,.02,0;0,0,0];watch=tic;
            [~,state]=controller_m3508_v2(q,v,zeros(2,1),ref,cfg,cfg.scenarios(4),'CA-BLNN',state,o);
            times(k)=toc(watch);
        end
        times=times(1001:end);ordered=sort(times);allTimes{trial}=times;
        rows(end+1,:)={trial,1e6*mean(times),1e6*ordered(ceil(.95*numel(times))), ...
            1e6*max(times),sum(times>.001),60,60}; %#ok<AGROW>
    end
    metrics=cell2table(rows,'VariableNames',{'Repeat','Mean_us','P95_us','Max_us','Over1msSteps','NodesY','NodesAngle'});
    out=struct('metrics',metrics,'wall_times',{allTimes},'network_scalar_capacity',1080, ...
        'network_double_bytes',8640,'network_single_bytes',4320, ...
        'statement','Synthetic maximum-node desktop benchmark; excludes sensors/CAN/RTOS; wall-time outliers include scheduling; no MCU deployment claim.');
    save(fullfile(dest,'resources.mat'),'out');
    writetable(metrics,fullfile(dest,'resource_metrics.csv'));disp(metrics);
end
