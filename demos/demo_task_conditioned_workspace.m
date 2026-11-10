function report=demo_task_conditioned_workspace(output_dir)
%DEMO_TASK_CONDITIONED_WORKSPACE Compare identical arm, targets and base poses.
%   Run from project root:
%     addpath('pipeline'); init_paths;
%     report=demo_task_conditioned_workspace;
%   This small finite-grid experiment writes CSV/MAT/PNG and reports numerical
%   IK support, not collision-free workspace volume or real wiping success.
    if nargin<1 || isempty(output_dir)
        output_dir=fullfile('results','task_conditioned_workspace');
    end
    if ~isfolder(output_dir),mkdir(output_dir);end
    robot=robot_config();
    robot.manipulator=task_manipulation_config(robot);
    pose=[0,0,0];
    % Same physical arm for all columns; no comparison against the small arm.
    xs=linspace(0.40,0.90,6);
    ys=linspace(-0.15,0.25,5);
    [X,Y]=meshgrid(xs,ys);
    z=0.45;
    support=zeros(numel(X),3);
    qpaths=cell(numel(X),3);
    types={'position_only','top_down_grasp','table_wipe'};
    for i=1:numel(X)
        for j=1:numel(types)
            task=struct('x',X(i),'y',Y(i),'z',z,'task_type',types{j});
            % A fixed world-frame wipe direction represents a specified task.
            % It may require a different docking yaw than an axis-only grasp.
            if j==3
                task.manipulation.waypoints_world=[X(i),Y(i)-0.04,z;X(i),Y(i)+0.04,z];
            end
            [~,~,support(i,j),arm,details]=evaluate_task_reachability(pose,task,robot);
            if support(i,j)>0,qpaths{i,j}=details.(arm).q_path;end
        end
        if mod(i,5)==0,fprintf('Task workspace samples: %d/%d\n',i,numel(X));end
    end
    samples=table(X(:),Y(:),repmat(z,numel(X),1),support(:,1),support(:,2),support(:,3), ...
        'VariableNames',{'x_m','y_m','z_m','position_only','top_down_grasp','table_wipe'});
    writetable(samples,fullfile(output_dir,'workspace_samples.csv'));
    report=struct('model_revision',robot.manipulator.model.revision, ...
        'mount_source',robot.manipulator.mount_source,'hardware_calibrated',false, ...
        'sample_count',numel(X),'supported_counts',sum(support,1), ...
        'task_types',{types},'collision_checked',false, ...
        'evidence_source','sampled_numerical_ik','samples',samples,'q_paths',{qpaths}, ...
        'robot_cfg',robot);
    fig=figure('Visible','off','Color','w','Position',[100,100,1100,360]);
    cleanup=onCleanup(@()close(fig)); %#ok<NASGU>
    titles={'Position only','Downward grasp + approach','Wipe: fixed pose + 8 cm path'};
    for j=1:3
        subplot(1,3,j);
        scatter(X(:),Y(:),100,support(:,j),'filled');
        caxis([0,1]);colormap([.8,.8,.8;.12,.55,.36]);
        axis equal;grid on;xlim([.35,.95]);ylim([-.20,.30]);
        xlabel('world x (m)');ylabel('world y (m)');
        title(sprintf('%s\n%d/%d supported',titles{j},sum(support(:,j)),numel(X)));
    end
    sgtitle('VX300s finite-grid IK support at z = 0.45 m; collisions not checked');
    exportgraphics(fig,fullfile(output_dir,'workspace_comparison.png'),'Resolution',150);
    save(fullfile(output_dir,'workspace_report.mat'),'report');
    fprintf('Supported samples [position, grasp, wipe]: [%d %d %d] / %d\n',sum(support,1),numel(X));
end
