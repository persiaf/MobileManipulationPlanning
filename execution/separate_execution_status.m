function status = separate_execution_status(predicted_tasks, commanded_tasks, executed_tasks, verified_tasks, metadata)
%SEPARATE_EXECUTION_STATUS Preserve legacy states with explicit evidence kind.
%   The historical four-argument call describes geometric simulation. Pass
%   metadata.source='unknown_legacy' for an unclassified legacy report, or an
%   explicit measured_execution/proxy_support/ik_validation source. A simulated
%   verification never supplies measured or IK evidence.

    if nargin < 5 || isempty(metadata), metadata = struct('source','geometric_simulation'); end
    if ischar(metadata) || isstring(metadata), metadata = struct('source',char(metadata)); end
    source = field_or(metadata,'source','unknown_legacy');
    allowed = {'geometric_simulation','measured_execution','proxy_support','ik_validation','unknown_legacy'};
    if ~any(strcmp(source,allowed))
        error('separate_execution_status:EvidenceSource','Unsupported evidence source "%s".',source);
    end
    status.predicted_tasks = task_ids(predicted_tasks);
    status.commanded_tasks = task_ids(commanded_tasks);
    status.executed_tasks = task_ids(executed_tasks);
    status.verified_tasks = task_ids(verified_tasks);
    e = struct('schema_version','2.0','source',source);
    e.geometry_evidence_available = field_or(metadata,'geometry_evidence_available',strcmp(source,'geometric_simulation'));
    e.simulation_evidence_available = field_or(metadata,'simulation_evidence_available',strcmp(source,'geometric_simulation'));
    e.proxy_evidence_available = field_or(metadata,'proxy_evidence_available',strcmp(source,'proxy_support'));
    e.ik_evidence_available = field_or(metadata,'ik_evidence_available',strcmp(source,'ik_validation'));
    e.measured_evidence_available = field_or(metadata,'measured_evidence_available',strcmp(source,'measured_execution'));
    e.geometric_predicted_tasks = evidence_ids(metadata,'geometric_predicted_tasks',status.predicted_tasks,e.geometry_evidence_available);
    e.simulated_executed_tasks = evidence_ids(metadata,'simulated_executed_tasks',status.executed_tasks,e.simulation_evidence_available);
    e.simulated_geometric_verified_tasks = evidence_ids(metadata,'simulated_geometric_verified_tasks',status.verified_tasks,e.simulation_evidence_available);
    e.proxy_supported_tasks = evidence_ids(metadata,'proxy_supported_tasks',status.predicted_tasks,e.proxy_evidence_available);
    e.ik_checked_tasks = evidence_ids(metadata,'ik_checked_tasks',status.executed_tasks,e.ik_evidence_available);
    e.ik_feasible_tasks = evidence_ids(metadata,'ik_feasible_tasks',status.verified_tasks,e.ik_evidence_available);
    e.measured_executed_tasks = evidence_ids(metadata,'measured_executed_tasks',status.executed_tasks,e.measured_evidence_available);
    e.measured_verified_tasks = evidence_ids(metadata,'measured_verified_tasks',status.verified_tasks,e.measured_evidence_available);
    e.ik_verified = logical(e.ik_evidence_available);
    e.actual_execution_measured = logical(e.measured_evidence_available);
    if strcmp(source,'geometric_simulation')
        e.legacy_verified_semantics = 'simulated_geometric_acceptance';
    elseif strcmp(source,'measured_execution')
        e.legacy_verified_semantics = 'measured_task_verification';
    else
        e.legacy_verified_semantics = source;
    end
    status.evidence_source = source;
    status.legacy_verified_semantics = e.legacy_verified_semantics;
    status.evidence = e;
end

function ids = evidence_ids(metadata,name,fallback,available)
    if available, ids = task_ids(field_or(metadata,name,fallback)); else, ids = []; end
end

function ids = task_ids(value)
    if isempty(value), ids = []; return; end
    if ~isnumeric(value) || any(~isfinite(value(:))) || ...
            any(value(:) < 1 | value(:) ~= fix(value(:)))
        error('separate_execution_status:TaskIds','Task identifiers must be positive finite integers.');
    end
    ids = unique(reshape(value,1,[]),'stable');
end

function value = field_or(s,name,fallback)
    if isstruct(s) && isfield(s,name), value = s.(name); else, value = fallback; end
end
