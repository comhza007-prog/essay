function diagnostics=diagnose_v2_regressions()
% Identify all failed paired tracking trials, retain exact samples and trace.
    folder=fileparts(mfilename('fullpath'));
    d=load(fullfile(folder,'results_robustness_v2','robustness.mat'));rows={};
    for j=1:20
        a=d.out.runs{2,j,1};b=d.out.runs{2,j,2};sample=d.out.samples{2,j};
        if b.metrics(1)>a.metrics(1)
            values=[a.metrics(1),b.metrics(1),a.metrics(5),b.metrics(5), ...
                sample.env.delta_m,sample.env.d_load,sample.env.Kf_L,sample.env.Kf_R, ...
                sample.validation.delay_steps,max(abs(b.z_aw(:))),max(abs(b.request(:)))];
            rows(end+1,:)=[{sample.seed},num2cell(values)]; %#ok<AGROW>
        end
    end
    diagnostics=cell2table(rows,'VariableNames',{'Seed','BaseTracking_mm','V2Tracking_mm', ...
        'BaseTail_mm','V2Tail_mm','DeltaMass_kg','Eccentricity_m','KfL','KfR','Delay_ms','AWPeak_counts','RequestPeak_counts'});
    disp(diagnostics);
end
