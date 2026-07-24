function result = evaluate_ik_feasibility(request, adapter)
%EVALUATE_IK_FEASIBILITY Invoke a configured real-robot IK adapter.

    if nargin < 2 || isempty(adapter)
        error('NotImplemented: real robot IK adapter is not configured.');
    end
    if ~isa(adapter, 'function_handle')
        error('evaluate_ik_feasibility:InvalidAdapter', ...
            'adapter must be a function handle.');
    end
    result = adapter(request);
    if ~isstruct(result) || ~isfield(result, 'feasible')
        error('evaluate_ik_feasibility:InvalidResult', ...
            'IK adapter must return a struct containing feasible.');
    end
end
