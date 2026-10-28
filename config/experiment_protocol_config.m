function protocol = experiment_protocol_config(base)
%EXPERIMENT_PROTOCOL_CONFIG Standard comparison contract from main defaults.
    if nargin < 1, base = planner_config(); end
    protocol.id = 'main_robot_geometry_repair_20261027_v2';
    protocol.enforce = false;
    protocol.robot_source = 'config/robot_config.m';
    protocol.reachability_threshold = base.reachability.threshold;
    protocol.reachability_mode = 'nominal';
    protocol.return_to_start = true;
    protocol.planning_time_scope = 'scene+candidate+selection+path_cost+route_order+stitch+route_validation';
    protocol.excluded_time_scope = 'execution_simulation+oracle_audit+rendering+file_io';
    protocol.evidence_source = 'geometric_simulation';
    protocol.ik_evidence_available = false;
    protocol.measured_evidence_available = false;
end
