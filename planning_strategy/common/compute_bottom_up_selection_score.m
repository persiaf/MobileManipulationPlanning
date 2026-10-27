function score = compute_bottom_up_selection_score(coverage_gain, reachability, ...
        local_cost, transition_cost, side_switch, weights)
%COMPUTE_BOTTOM_UP_SELECTION_SCORE Evaluate the five normalized greedy terms.
    terms = [coverage_gain, reachability, local_cost, transition_cost, side_switch];
    if any(~isfinite(terms)) || any(terms < 0) || any(terms > 1)
        error('compute_bottom_up_selection_score:InvalidTerm', ...
            'All score terms must be finite values in [0,1].');
    end
    names = {'coverage_gain','reachability','local_cost','intra_region_move','side_switch'};
    w = zeros(1, numel(names));
    for i = 1:numel(names), w(i) = weights.(names{i}); end
    if any(w < 0) || sum(w) <= 0
        error('compute_bottom_up_selection_score:InvalidWeights', ...
            'Score weights must be nonnegative with a positive sum.');
    end
    w = w / sum(w);
    score = w(1)*terms(1) + w(2)*terms(2) - w(3)*terms(3) - ...
        w(4)*terms(4) - w(5)*terms(5);
end
