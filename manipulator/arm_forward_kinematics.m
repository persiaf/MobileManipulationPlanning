function [T, J] = arm_forward_kinematics(q, model)
%ARM_FORWARD_KINEMATICS Revolute chain FK and spatial geometric Jacobian.
% J(1:3,:) maps joint rates to TCP linear velocity in the arm base frame;
% J(4:6,:) maps them to angular velocity in that same frame. Joint origin
% rotations are identity, as in the pinned VX300s model.

    n = size(model.joint_axes,1);
    validateattributes(q,{'numeric'},{'real','finite','vector','numel',n}, ...
        mfilename,'q');
    q = q(:);
    T = eye(4);
    origins = zeros(3,n);
    axes_world = zeros(3,n);
    for k = 1:n
        T(1:3,4) = T(1:3,4) + T(1:3,1:3)*model.joint_origin_xyz(k,:)';
        origins(:,k) = T(1:3,4);
        a = model.joint_axes(k,:)';
        a = a/norm(a);
        axes_world(:,k) = T(1:3,1:3)*a;
        K = [0,-a(3),a(2); a(3),0,-a(1); -a(2),a(1),0];
        R = eye(3) + sin(q(k))*K + (1-cos(q(k)))*(K*K);
        T(1:3,1:3) = T(1:3,1:3)*R;
    end
    T = T*model.tool_transform;
    if nargout > 1
        J = zeros(6,n);
        for k = 1:n
            J(:,k) = [cross(axes_world(:,k),T(1:3,4)-origins(:,k)); ...
                axes_world(:,k)];
        end
    end
end
