function o=options_v3()
% Frozen candidate before holdout: do not edit after observing holdout data.
    o=options_v2();o.allocator='clip';o.sync_weight=2;o.aw_scale=1;
    % Neural feedback still contains the rotational synchronization channel.
    % 'clip' removes only lexicographic actuator redistribution, not sync control.
end
