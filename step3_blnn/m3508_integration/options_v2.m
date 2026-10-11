function o = options_v2()
% Engineering changes, not source-paper controller or a stability theorem.
    o.independent_leak=true;
    o.sector_guard=true;
    o.sector_fraction=.25;
    o.enhancements=true;
    o.output_gate=true;
    o.learning_gate=true;
    o.growth_gate=true;
    o.fixed_nodes=false;
    o.sync_alloc=true;
end
