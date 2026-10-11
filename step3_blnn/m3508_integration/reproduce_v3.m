function resultRoot=reproduce_v3(resultRoot)
% Recompute final-candidate evidence in a NEW folder; never overwrite archives.
    folder=fileparts(mfilename('fullpath'));integration_config();
    if nargin<1,resultRoot=fullfile(folder,['reproduction_v3_',char(datetime('now','Format','yyyyMMdd_HHmmss'))]);end
    assert(~exist(resultRoot,'file') && ~exist(resultRoot,'dir'),'Choose a new output directory.');
    mkdir(resultRoot);verify_v3();
    run_holdout_v3(fullfile(resultRoot,'results_holdout_v3'));
    run_ablation_v3(fullfile(resultRoot,'results_ablation_v3'));
    run_sensitivity_v3(fullfile(resultRoot,'results_sensitivity_v3'));
    run_resource_check_v3(fullfile(resultRoot,'results_resources_v3'));
    theory_check_v3(resultRoot);plot_validation_v3(resultRoot);audit_results_v3(resultRoot);
    fprintf('Fresh reproduction saved to %s\n',resultRoot);
end
