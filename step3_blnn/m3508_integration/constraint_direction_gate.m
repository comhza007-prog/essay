function gate = constraint_direction_gate(base_request, correction, limits)
% Largest common fraction that never increases component-wise infeasibility.
% Inside the box: remain inside it. Outside: only inward/zero correction
% is permitted, and the opposite boundary must not be crossed.
    assert(all(limits(:)>0) && all(isfinite([base_request(:);correction(:)])));
    gate = 1;
    for k = 1:2
        b = base_request(k); d = correction(k); lim = limits(k);
        if d == 0, continue; end
        if abs(b)>lim && sign(b)*d>0
            gate = 0; return;
        end
        if d>0, cap = (lim-b)/d;
        else, cap = (-lim-b)/d; end
        gate = min(gate,max(0,cap));
    end
    gate = min(1,gate);
end
