function [order,recovery]=finite_nearest_neighbor_order(D,start_idx,return_to_start)
%FINITE_NEAREST_NEIGHBOR_ORDER First finite NN tour in fixed start order.
% Try the actual start, then all other node indices in ascending order.
% Rotate each candidate back to the actual start and check every edge,
% including the return edge when requested. This is feasibility recovery,
% not a cheapest-restart search or proof that no physical route exists.
% Existing undirected 2-opt is deliberately not used on this directed D.
    if nargin<2,start_idx=1;end
    if nargin<3,return_to_start=true;end
    if ~isnumeric(D)||~isreal(D)||ndims(D)~=2||isempty(D)|| ...
            size(D,1)~=size(D,2)||any(isnan(D(:)))||any(D(:)<0)||any(diag(D)~=0)
        error('finite_nearest_neighbor_order:CostMatrix','D must be square, nonnegative, NaN-free and zero-diagonal.');
    end
    n=size(D,1);
    validateattributes(start_idx,{'numeric'},{'real','finite','scalar','integer','>=',1,'<=',n});
    if ~isscalar(return_to_start)||~any(return_to_start==[false true])
        error('finite_nearest_neighbor_order:ReturnFlag','return_to_start must be logical.');
    end
    starts=[start_idx,setdiff(1:n,start_idx,'stable')];
    attempts=struct('start',{},'closed_finite',{},'route_finite',{},'visit_order',{}, ...
        'closed_order',{},'cost',{},'bad_edges',{});
    recovery=struct('enabled',true,'status','searching','recovered',false, ...
        'initial_closed_finite',false,'selected_start',NaN,'attempt_count',0, ...
        'attempts',attempts,'return_to_start',logical(return_to_start), ...
        'route_order_source','nearest_neighbor','two_opt_applied',false, ...
        'rule','first finite rotated NN order in fixed node-index start order; not minimum restart cost');
    for seed=starts
        candidate=nearest_neighbor_order(D,seed,false);
        at=find(candidate==start_idx,1);candidate=[candidate(at:end),candidate(1:at-1)];
        closed=[candidate,start_idx];
        closed_costs=D(sub2ind(size(D),closed(1:end-1),closed(2:end)));
        checked=closed;
        if ~return_to_start,checked=candidate;end
        edge_costs=D(sub2ind(size(D),checked(1:end-1),checked(2:end)));
        bad=find(~isfinite(edge_costs));finite=isempty(bad);
        attempt=struct('start',seed,'closed_finite',all(isfinite(closed_costs)), ...
            'route_finite',finite,'visit_order',candidate,'closed_order',closed, ...
            'cost',sum(edge_costs),'bad_edges',[checked(bad)',checked(bad+1)']);
        recovery.attempts(end+1)=attempt;recovery.attempt_count=numel(recovery.attempts);
        if seed==start_idx,recovery.initial_closed_finite=attempt.closed_finite;end
        if finite
            order=candidate;recovery.selected_start=seed;recovery.recovered=seed~=start_idx;
            recovery.status='initial_finite';
            if recovery.recovered
                recovery.status='recovered';recovery.route_order_source='nearest_neighbor_finite_restart';
            end
            return;
        end
    end
    error('finite_nearest_neighbor_order:NoFiniteOrder', ...
        ['No finite order was found by the configured nearest-neighbor restarts. ' ...
         'This finite search failure is not a proof of physical infeasibility.']);
end
