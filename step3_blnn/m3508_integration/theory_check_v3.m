function summary=theory_check_v3(resultRoot)
% Conditional linear-core checks, NOT global stability of saturated NN loop.
    cfg=integration_config();folder=fileparts(mfilename('fullpath'));
    if nargin<1,resultRoot=folder;end
    dest=fullfile(resultRoot,'results_theory_v3');if ~exist(dest,'dir'),mkdir(dest);end
    assert(~exist(fullfile(dest,'theory.mat'),'file'),'Preserve existing results.');
    envs=cfg.scenarios;envs(5)=envs(4);envs(5).name='Symmetric limits';
    envs(5).Imax=[5000;5000];
    envs(3).delta_m=4.5;envs(6)=envs(3);envs(6).delta_m=.5;
    envs(6).name='Load step after';labels={envs.name};
    d=load(fullfile(resultRoot,'results_holdout_v3','holdout.mat'));
    for s=1:2
        for j=1:20
            envs(end+1)=d.out.samples{s,j}.env; %#ok<AGROW>
            labels{end+1}=sprintf('%s seed %d',envs(end).name,d.out.seeds(j)); %#ok<AGROW>
        end
    end
    rows={};models=cell(numel(envs),3);c=cfg.ctrl;
    Cq=[eye(2),zeros(2)];Le=cfg.mech.Le;
    Tnom=[cfg.mapping.Kf_L,-cfg.mapping.Kf_R; ...
        -.5*Le*cfg.mapping.Kf_L,-.5*Le*cfg.mapping.Kf_R];
    for s=1:numel(envs)
        env=envs(s);ell=Le/2;bL=cfg.plant.b_nom;bR=bL*(1+env.delta_fric);
        M=[cfg.mech.mG_nom+env.delta_m,env.delta_m*env.d_load; ...
            env.delta_m*env.d_load,cfg.mech.J_alpha_nom+env.delta_m*env.d_load^2];
        D=[bL+bR,ell*(bR-bL);ell*(bR-bL),ell^2*(bL+bR)+cfg.plant.B_alpha];
        K=diag([0,cfg.plant.K_alpha]);
        Ccl=D-c.B_hat+c.K1+c.M_hat*c.Gamma;
        Kcl=K-c.K_hat+c.K1*c.Gamma;
        assert(min(eig(M))>0 && min(eig(Ccl))>0 && min(eig(Kcl))>0);
        A=[zeros(2),eye(2);-M\Kcl,-M\Ccl];
        P=lyap(A',eye(4));assert(min(eig(P))>0);
        assert(norm(A'*P+P*A+eye(4),'fro')<1e-8);
        Ttrue=[env.Kf_L,-env.Kf_R;-.5*Le*env.Kf_L,-.5*Le*env.Kf_R];
        G=Ttrue/Tnom;
        Ap=[zeros(2),eye(2);-M\K,-M\D];Bp=[zeros(2);M\G];
        E=expm([Ap,Bp;zeros(2,6)]*cfg.Ts);Ep=E(1:4,1:4);Gp=E(1:4,5:6);
        av=1-exp(-2*pi*cfg.measurement.velocity_cutoff*cfg.Ts);
        Q=c.K_hat-c.K1*c.Gamma;V=c.B_hat-c.K1-c.M_hat*c.Gamma;
        for delay=0:2
            % State: real q/v, past q(1..delay+1), previous velocity filter.
            n=4+2*(delay+1)+2;Qm=zeros(2,n);Qprev=Qm;Vprev=Qm;
            if delay==0,Qm(:,1:4)=Cq;
            else,Qm(:,4+2*(delay-1)+(1:2))=eye(2);end
            Qprev(:,4+2*delay+(1:2))=eye(2);Vprev(:,end-1:end)=eye(2);
            Vf=av/cfg.Ts*(Qm-Qprev)+(1-av)*Vprev;
            U=Q*Qm+V*Vf;Phi=zeros(n);Phi(1:4,1:4)=Ep;
            Phi(1:4,:)=Phi(1:4,:)+Gp*U;
            Phi(5:6,1:4)=Cq;
            for j=2:delay+1,Phi(4+2*(j-1)+(1:2),4+2*(j-2)+(1:2))=eye(2);end
            Phi(end-1:end,:)=Vf;
            rho=max(abs(eig(Phi)));assert(rho<1);
            Pd=dlyap(Phi',eye(n));assert(min(eig(Pd))>0);
            residual=norm(Phi'*Pd*Phi-Pd+eye(n),'fro');assert(residual<1e-5);
            models{s,delay+1}=struct('M',M,'D',D,'K',K,'A',A,'P',P, ...
                'Phi',Phi,'Pd',Pd,'Gp',Gp,'force_gain_ratio',G);
            rows(end+1,:)={labels{s},delay,min(eig(M)),min(eig(Ccl)), ...
                min(eig(Kcl)),max(real(eig(A))),rho,residual}; %#ok<AGROW>
        end
    end
    summary=cell2table(rows,'VariableNames',{'Scenario','Delay_ms','MinEigM', ...
        'MinEigCcl','MinEigKcl','ContinuousMaxRealEig','SampledSpectralRadius','DiscreteLyapunovResidual'});
    save(fullfile(dest,'theory.mat'),'cfg','envs','summary','models','-v7.3');
    writetable(summary,fullfile(dest,'linear_core_checks.csv'));
    fprintf('Conditional linear core: %d models; max sampled radius %.9f.\n', ...
        height(summary),max(summary.SampledSpectralRadius));
end
