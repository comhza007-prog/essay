function checks=verify_clean_reproduction_v3(cleanRoot)
% Compare ALL numerical trajectories, not timing, with independent fresh run.
    folder=fileparts(mfilename('fullpath'));
    if nargin<1
        root=fileparts(fileparts(folder));
        cleanRoot=fullfile(root,'BLNN_v3_cleancheck_20261010','step3_blnn', ...
            'm3508_integration','fresh_run');
    end
    files={'results_holdout_v3/holdout.mat','results_ablation_v3/ablation.mat', ...
        'results_sensitivity_v3/sensitivity.mat'};
    count=0;peakDifference=0;
    fields={'q','current','request','weight','nodes','z_aw','nn','sigma', ...
        'sector_gate','output_gate','learning_gate','theta','saturated'};
    for p=1:numel(files)
        a=load(fullfile(folder,files{p}));b=load(fullfile(cleanRoot,files{p}));
        assert(isequal(size(a.out.runs),size(b.out.runs)));
        for j=1:numel(a.out.runs)
            count=count+1;
            x=a.out.runs{j};y=b.out.runs{j};
            assert(max(abs(x.metrics(1:12)-y.metrics(1:12)))<1e-9);
            for k=1:numel(fields)
                value=max(abs(double(x.(fields{k}))-double(y.(fields{k}))),[],'all');
                peakDifference=max(peakDifference,value);assert(value<1e-9);
            end
        end
    end
    assert(count==187);
    checks=struct('clean_runs',count,'max_numerical_difference',peakDifference, ...
        'wall_clock_timing_excluded',true,'all_core_time_series_reproduced',true);
    disp(checks);save(fullfile(folder,'results_holdout_v3','clean_reproduction_check.mat'),'checks','cleanRoot');
end
