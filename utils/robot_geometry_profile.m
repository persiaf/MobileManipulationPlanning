function cfg = robot_geometry_profile(profile_name)
%ROBOT_GEOMETRY_PROFILE Return explicit robot geometry profiles.
%
%   legacy_validated preserves robot_config() exactly. physical_robot maps
%   the measured xa/da/h0 values into the same base-frame fields used by the
%   reachability model, without changing default project behavior.

    if nargin < 1 || isempty(profile_name)
        profile_name = 'legacy_validated';
    end

    cfg = robot_config();
    cfg.geometry_profile = char(profile_name);

    switch lower(profile_name)
        case 'legacy_validated'
            return;
        case 'physical_robot'
            cfg.dual_arm.base_spacing = 0.40;
            cfg.arm_base_mount_x = 0.06;
            cfg.arm_base_mount_z = 0.40;
        otherwise
            error('robot_geometry_profile:UnknownProfile', ...
                  'Unknown robot geometry profile "%s".', profile_name);
    end
end
