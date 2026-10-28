function audit = independent_geometric_reachability(base_poses, target_xyz, robot_cfg, threshold)
%INDEPENDENT_GEOMETRIC_REACHABILITY Independent configured-envelope audit.
%   AUDIT = INDEPENDENT_GEOMETRIC_REACHABILITY(POSES,TARGETS,ROBOT,THRESHOLD)
%   accepts N-by-3 rows [x y yaw] and [x y z]. A single row on either side is
%   broadcast over the other input. It transforms targets into the chassis
%   frame first, then subtracts each arm mount; it does not call the production
%   reachability evaluators or their rotation helper.
%
%   hard_envelope_feasible tests the declared geometric inequalities only.
%   score_accepted also applies the independently calculated heuristic score.
%   Neither result is physical IK feasibility. The repository supplies no
%   complete joint-transform chain/URDF or configured IK adapter. This audit
%   does not check base/arm collision, target orientation, or motion continuity.

    if nargin < 3 || isempty(robot_cfg), robot_cfg = robot_config(); end
    if nargin < 4 || isempty(threshold)
        planner = planner_config(); threshold = planner.reachability.threshold;
    end
    validate_rows(base_poses, 'base_poses');
    validate_rows(target_xyz, 'target_xyz');
    if ~isnumeric(threshold) || ~isscalar(threshold) || ~isfinite(threshold) || ...
            threshold < 0 || threshold > 1
        error('independent_geometric_reachability:Threshold', ...
            'threshold must be a finite scalar in [0,1].');
    end
    np = size(base_poses,1); nt = size(target_xyz,1);
    if np ~= nt
        if np == 1
            base_poses = repmat(base_poses, nt, 1);
        elseif nt == 1
            target_xyz = repmat(target_xyz, np, 1);
        else
            error('independent_geometric_reachability:RowCount', ...
                'Inputs must have equal row counts or contain one broadcast row.');
        end
    end
    n = size(base_poses,1);
    delta = target_xyz(:,1:2) - base_poses(:,1:2);
    heading = base_poses(:,3);
    longitudinal = delta(:,1).*cos(heading) + delta(:,2).*sin(heading);
    transverse = -delta(:,1).*sin(heading) + delta(:,2).*cos(heading);
    arm_x = longitudinal - robot_cfg.arm_base_mount_x;
    arm_z = target_xyz(:,3) - robot_cfg.arm_base_mount_z;
    left = audit_arm([arm_x, transverse-robot_cfg.dual_arm.base_spacing/2, arm_z], ...
        robot_cfg.single_arm, threshold);
    right = audit_arm([arm_x, transverse+robot_cfg.dual_arm.base_spacing/2, arm_z], ...
        robot_cfg.single_arm, threshold);

    assignment = repmat({'none'},n,1);
    chosen_score = max(left.accepted_score, right.accepted_score);
    for k = 1:n
        if chosen_score(k) > 0
            if left.accepted_score(k) >= right.accepted_score(k)
                assignment{k} = 'left';
            else
                assignment{k} = 'right';
            end
        end
    end
    audit = struct();
    audit.model_scope = 'independent_configured_geometric_envelope';
    audit.validation_kind = 'geometric_model_cross_check';
    audit.threshold = threshold;
    audit.pair_count = n;
    audit.left = left;
    audit.right = right;
    audit.hard_envelope_feasible = left.hard_envelope_feasible | right.hard_envelope_feasible;
    audit.score_accepted = chosen_score > 0;
    audit.combined_score = chosen_score;
    audit.arm_assignment = assignment;
    audit.ik_verified = false;
    audit.ik_feasible = NaN(n,1);
    audit.collision_checked = false;
    audit.motion_verified = false;
    audit.kinematic_model_status = 'full_joint_transform_chain_not_available';
    audit.interpretation = ['Independent implementation of the configured geometric model; ' ...
        'does not establish physical IK, collision-free manipulation, or execution success.'];
end

function out = audit_arm(local_xyz, bounds, threshold)
    rho = hypot(local_xyz(:,1),local_xyz(:,2));
    % The 1 cm inner exclusion is part of the existing geometric evaluator's
    % model contract, not a measured joint-space workspace boundary.
    inner = 0.01;
    outer = bounds.reach_radius;
    zlo = bounds.reach_z_min; zhi = bounds.reach_z_max;
    if outer <= inner || zhi <= zlo || bounds.reach_forward <= 0 || ...
            bounds.reach_lateral <= 0 || bounds.reach_backward < 0
        error('independent_geometric_reachability:InvalidBounds', ...
            'The configured reach bounds are invalid.');
    end
    margins = [rho-inner, outer-rho, local_xyz(:,3)-zlo, ...
        zhi-local_xyz(:,3), local_xyz(:,1)+bounds.reach_backward, ...
        1.1*bounds.reach_lateral-abs(local_xyz(:,2))];
    hard = all(margins >= 0,2);
    radial_middle = max(0, min((rho-inner)/((outer-inner)/2), ...
        (outer-rho)/((outer-inner)/2)));
    height_middle = max(0, min((local_xyz(:,3)-zlo)/((zhi-zlo)/2), ...
        (zhi-local_xyz(:,3))/((zhi-zlo)/2)));
    forward_preference = min(1,max(0,local_xyz(:,1)/bounds.reach_forward));
    components = [1-rho/outer, radial_middle, height_middle, ...
        forward_preference, 1-abs(local_xyz(:,2))/bounds.reach_lateral];
    independently_calculated = min(1,max(0,components*[0.30;0.25;0.20;0.15;0.10]));
    accepted = hard & independently_calculated >= threshold & independently_calculated > 0;
    scores = zeros(size(rho)); scores(accepted) = independently_calculated(accepted);
    raw = independently_calculated; raw(~hard) = NaN;
    labels = {'inner_radial','outer_radial','lower_height','upper_height', ...
        'backward','lateral'};
    reasons = repmat({'accepted'},numel(rho),1);
    for k = 1:numel(rho)
        if ~hard(k)
            reasons{k} = strjoin(labels(margins(k,:) < 0), '|');
        elseif ~accepted(k)
            reasons{k} = 'score_below_acceptance';
        end
    end
    out = struct('arm_local_xyz',local_xyz,'horizontal_radius_m',rho, ...
        'constraint_margin_names',{labels},'constraint_margins_m',margins, ...
        'hard_envelope_feasible',hard,'raw_score',raw, ...
        'score_accepted',accepted,'accepted_score',scores,'reason',{reasons});
end

function validate_rows(value, name)
    if ~isnumeric(value) || ~ismatrix(value) || size(value,2) ~= 3 || ...
            any(~isfinite(value(:))) || ~isreal(value)
        error('independent_geometric_reachability:Input', ...
            '%s must contain finite real N-by-3 rows.',name);
    end
end
