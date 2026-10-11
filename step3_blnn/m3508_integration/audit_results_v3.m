function checks=audit_results_v3(resultRoot)
% Independent read-only recomputation; acceptance is not inferred from plots.
    folder=fileparts(mfilename('fullpath'));cfg=integration_config();
    if nargin<1,resultRoot=folder;end
    files={'results_holdout_v3/holdout.mat','results_ablation_v3/ablation.mat', ...
        'results_sensitivity_v3/sensitivity.mat'};
    optional={'results_coordination_v3/coordination.mat','results_replay_v3/replay.mat'};
    for j=1:numel(optional)
        if exist(fullfile(resultRoot,optional{j}),'file'),files{end+1}=optional{j};end %#ok<AGROW>
    end
    count=0;
    for p=1:numel(files)
        d=load(fullfile(resultRoot,files{p}));runs=d.out.runs;
        for j=1:numel(runs)
            row=j;
            if contains(files{p},'holdout')
                [s,trial,m]=ind2sub(size(runs),j);row=(s-1)*40+(trial-1)*2+m;
            elseif contains(files{p},'ablation')
                [s,m]=ind2sub(size(runs),j);row=(s-1)*13+m;
            end
            r=runs{j};count=count+1;limits=min(r.env.Imax(:),cfg.ctrl.I_fw_limit);
            e=(r.q(1,:)-cfg.traj.y)*1000;sync=cfg.mech.Le*r.q(2,:)*1000;
            raw=[sqrt(mean(e.^2)),sqrt(mean(sync.^2)),max(abs(sync)),e(end), ...
                sqrt(mean(e(r.t>=6.5).^2)),100*mean(r.saturated), ...
                100*mean(any(abs(r.current)>=limits-1e-6,1)), ...
                cfg.Ts*sum(abs(r.current-r.request),'all'),max(r.weight(:)),r.nodes(:,end)'];
            assert(max(abs(raw-r.metrics(1:11)))<1e-8);
            assert(all(isfinite([r.q(:);r.current(:);r.request(:);r.weight(:);r.nn(:);r.z_aw(:)])));
            assert(all(abs(r.current)<=limits+1e-8,'all'));
            assert(all(abs(r.nn)<=cfg.ctrl.nn.dhat_limit+1e-8,'all'));
            if r.options.sector_guard
                assert(all(r.sigma.*r.nn<=r.options.sector_fraction*diag(cfg.ctrl.K1).*r.sigma.^2+1e-9,'all'));
            end
            cap=60;
            if contains(files{p},'sensitivity') && strcmp(d.out.metrics.Parameter{row},'max_nodes')
                cap=d.out.metrics.Value(row);
            end
            assert(all(r.nodes<=cap,'all'));
            assert(max(abs(d.out.metrics{row,end-14:end}-r.metrics))<1e-8, ...
                'Saved table mismatch: %s, row %d.',files{p},row);
        end
    end
    assert(count>=187);
    d=load(fullfile(resultRoot,'results_holdout_v3','holdout.mat'));summary=d.out.paired_summary;
    assert(isequal(d.out.seeds,2026102001+(0:19)));
    assert(isempty(intersect(d.out.seeds,2026101001+(0:19))));
    for s=1:2
        a=zeros(20,7);b=a;
        for j=1:20
            for m=1:2
                r=d.out.runs{s,j,m};o=r.options;
                assert(strcmp(o.allocator,'clip') && o.aw_scale==1);
                assert(isequal(r.cfg_signature,[cfg.ctrl.nn.eta(:);cfg.ctrl.nn.epsilon(:)]));
                v=r.metrics;z=[v(1:3),abs(v(4)),v(5:7)];
                if m==1,a(j,:)=z;else,b(j,:)=z;end
            end
        end
        for k=1:7
            delta=b(:,k)-a(:,k);row=(s-1)*7+k;
            ci=mean(delta)+[-1 1]*2.093024054*std(delta)/sqrt(20);
            assert(abs(summary.PairedDeltaMean(row)-mean(delta))<1e-10);
            assert(max(abs(summary{row,8:9}-ci))<1e-10);
        end
        % Predeclared criteria, not an assertion that every trial improves.
        assert(all(summary.DeltaCI95Upper((s-1)*7+[1 2])<0));
        assert(summary.DeltaCI95Upper((s-1)*7+3)<=.1);
        row=(s-1)*7+4;assert(summary.V3Mean(row)<=summary.BaseMean(row)+.1);
    end
    checks=struct('saved_runs',count,'metrics_recomputed',true,'per_scenario_current_limits',true, ...
        'finite_states',true,'network_output_and_capacity_limits',true,'sector_inequality',true, ...
        'unseen_seeds_and_frozen_settings',true,'paired_statistics_recomputed',true, ...
        'predeclared_holdout_criteria',true);disp(checks);
end
