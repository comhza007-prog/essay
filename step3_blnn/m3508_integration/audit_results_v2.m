function checks=audit_results_v2()
% Independent recomputation from saved time series. No modification of data.
    folder=fileparts(mfilename('fullpath'));cfg=integration_config();count=0;
    files={'results_recovery_v2/recovery.mat','results_ablation_v2/ablation.mat', ...
        'results_robustness_v2/robustness.mat','results_sensitivity_v2/sensitivity.mat', ...
        'results_allocator_followup_v2/allocator.mat'};
    for p=1:numel(files)
        data=load(fullfile(folder,files{p}));runs=data.out.runs;
        for j=1:numel(runs)
            r=runs{j};if isempty(r),continue;end
            count=count+1;e=(r.q(1,:)-cfg.traj.y)*1e3;s=cfg.mech.Le*r.q(2,:)*1e3;
            raw=[sqrt(mean(e.^2)),sqrt(mean(s.^2)),max(abs(s)),e(end),sqrt(mean(e(r.t>=6.5).^2))];
            assert(max(abs(raw-r.metrics(1:5)))<1e-9);
            assert(all(isfinite([r.q(:);r.current(:);r.weight(:);r.nn(:)])));
            % All runs, including sensitivity, have an explicitly saved cfg
            % or options. Validate common 16000 firmware ceiling here; the
            % tighter per-scenario limits are asserted by each simulator.
            assert(all(abs(r.current)<=16000+1e-8,'all'));
            if r.options.sector_guard
                assert(all(r.sigma.*r.nn<=r.options.sector_fraction*diag(cfg.ctrl.K1).*r.sigma.^2+1e-9,'all'));
            end
        end
    end
    assert(count==217,'Missing simulation records.');
    checks=struct('saved_runs',count,'recomputed_metrics',true,'finite_states',true, ...
        'firmware_ceiling',true,'sector_inequality',true);
    d=load(fullfile(folder,'results_robustness_v2','robustness.mat'));
    summary=d.out.paired_summary;
    for s=1:2
        a=zeros(20,7);b=a;
        for j=1:20
            for m=1:2
                v=d.out.runs{s,j,m}.metrics;z=[v(1:3),abs(v(4)),v(5:7)];
                if m==1,a(j,:)=z;else,b(j,:)=z;end
            end
        end
        for k=1:7
            delta=b(:,k)-a(:,k); row=(s-1)*7+k;
            assert(abs(summary.PairedDeltaMean(row)-mean(delta))<1e-10);
            assert(abs(summary.DeltaCI95Upper(row)-(mean(delta)+2.093024054*std(delta)/sqrt(20)))<1e-10);
        end
    end
    checks.paired_statistics_recomputed=true;disp(checks);
end
