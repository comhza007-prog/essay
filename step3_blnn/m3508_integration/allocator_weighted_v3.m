function [current,info]=allocator_weighted_v3(request,Le,Kf,limits,weight)
% Exact convex box projection in [FL,FR], evaluated on four boundary edges.
% Cost=(FL+FR-FG)^2+weight*(FR-FL-2*Talpha/Le)^2, units N^2.
    assert(weight>0 && all(Kf>0) && all(limits>0));
    bounds=Kf(:).*limits(:);fg=request(1);df=2*request(2)/Le;
    wanted=.5*[fg-df;fg+df];
    if all(abs(wanted)<=bounds),best=wanted;
    else
        points=zeros(2,4);col=0;
        for fl=[-bounds(1),bounds(1)]
            col=col+1;fr=(fg+weight*df+(weight-1)*fl)/(1+weight);
            points(:,col)=[fl;max(-bounds(2),min(bounds(2),fr))];
        end
        for fr=[-bounds(2),bounds(2)]
            col=col+1;fl=(fg-weight*df+(weight-1)*fr)/(1+weight);
            points(:,col)=[max(-bounds(1),min(bounds(1),fl));fr];
        end
        costs=(sum(points,1)-fg).^2+weight*(points(2,:)-points(1,:)-df).^2;
        [~,j]=min(costs);best=points(:,j);
    end
    current=[best(1)/Kf(1);-best(2)/Kf(2)];
    info.is_sat_any=any(abs(best-wanted)>1e-8);
end
