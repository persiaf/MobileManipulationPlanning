classdef MobileManipulationExperimentManager < matlab.apps.AppBase
    %MOBILEMANIPULATIONEXPERIMENTMANAGER App Designer configuration entry.
    %   The app owns no planning implementation. It maps UI selections to a
    %   unified experiment config and delegates execution to the existing
    %   experiment_manager functions.

    properties (Access = public)
        UIFigure matlab.ui.Figure
        SceneDropDown matlab.ui.control.DropDown
        RobotDropDown matlab.ui.control.DropDown
        PlannerDropDown matlab.ui.control.DropDown
        StrategyDropDown matlab.ui.control.DropDown
        ConstraintDropDown matlab.ui.control.DropDown
        ExperimentModeDropDown matlab.ui.control.DropDown
        SeedField matlab.ui.control.NumericEditField
        RunButton matlab.ui.control.Button
        StatusText matlab.ui.control.TextArea
    end

    properties (Access = private)
        LastResult
    end

    methods (Access = public)
        function runButtonPushed(app, ~)
            app.RunButton.Enable = 'off';
            app.StatusText.Value = {'Building unified experiment config...'};
            drawnow;
            try
                cfg = app.buildConfigFromSelection();
                mode = char(app.ExperimentModeDropDown.Value);
                is_headless = contains(lower(mode), 'headless');
                save_record = contains(lower(mode), 'save');
                app.StatusText.Value = {sprintf('Running %s...', cfg.experiment_id), ...
                    'Planning is delegated to run_selected_experiment.'};
                drawnow;
                app.LastResult = run_selected_experiment(cfg, ...
                    'headless', is_headless, 'render_outputs', true, ...
                    'save_record', save_record);
                status = app.LastResult.status;
                app.StatusText.Value = {sprintf('Finished: %s', status), ...
                    sprintf('Experiment: %s', cfg.experiment_id), ...
                    sprintf('Output: results/managed_experiments/%s/', cfg.experiment_id)};
            catch ME
                app.StatusText.Value = {['Run failed: ' ME.message]};
                uialert(app.UIFigure, ME.message, 'Experiment Manager Error');
            end
            app.RunButton.Enable = 'on';
        end

        function cfg = buildConfigFromSelection(app)
            seed = app.SeedField.Value;
            scene_id = char(app.SceneDropDown.Value);
            robot_id = char(app.RobotDropDown.Value);
            planner_id = char(app.PlannerDropDown.Value);
            strategy_id = char(app.StrategyDropDown.Value);
            constraint_id = char(app.ConstraintDropDown.Value);

            % Reuse the existing config builder for every profile. The app
            % only composes returned structs; it does not reproduce logic.
            base = experiment_config_builder('baseline', 'seed', seed);
            scene_base = experiment_config_builder(scene_id, 'seed', seed);
            scene_cfg = scene_base.scene_cfg;
            robot_cfg = base.robot_cfg;
            base.scene_cfg = scene_cfg;
            base.robot_cfg = robot_cfg;
            base.planner_cfg.global_planner = planner_id;
            base.planner_cfg.strategy.name = strategy_id;
            % Interactive mode is allowed to show the stage figures. Headless
            % mode is applied by run_selected_experiment after this config is
            % built, so do not unconditionally disable visualization here.
            base.planner_cfg.runtime.disable_visualization = false;
            base.planner_cfg.runtime.disable_video = true;
            % Qualify the private static method explicitly. Without the
            % class qualifier MATLAB resolves this as a workspace function
            % and raises "Undefined function 'constraint_config'".
            base.validation = MobileManipulationExperimentManager.constraint_config(constraint_id);
            base.scene = base.scene_cfg;
            base.robot = base.robot_cfg;
            base.planner = base.planner_cfg;
            base.strategy = base.planner_cfg.strategy;
            base.scene_profile = scene_id;
            base.robot_profile = robot_id;
            base.planner_profile = planner_id;
            base.strategy_profile = strategy_id;
            base.validation_profile = constraint_id;
            base.experiment_id = MobileManipulationExperimentManager.make_experiment_id( ...
                scene_id, robot_id, planner_id, strategy_id);
            cfg = base;
        end
    end

    methods (Static, Access = private)
        function validation = constraint_config(name)
            validation = struct('profile', name, 'validate_scene', true, ...
                'validate_coverage', true, 'error_on_invalid_scene', false);
            switch lower(name)
                case 'strict scene'
                    validation.error_on_invalid_scene = true;
                case 'no scene validation'
                    validation.validate_scene = false;
                case 'coverage required'
                    validation.validate_coverage = true;
                otherwise
                    % Standard constraint profile.
            end
        end

        function id = make_experiment_id(scene, robot, planner, strategy)
            id = lower(strjoin({scene, robot, planner, strategy}, '_'));
            id = regexprep(id, '[^a-zA-Z0-9_]', '_');
        end
    end

    methods (Access = public)
        function app = MobileManipulationExperimentManager
            createComponents(app);
            registerApp(app, app.UIFigure);
            if nargout == 0, clear app; end
        end

        function delete(app)
            if ~isempty(app.UIFigure) && isvalid(app.UIFigure), delete(app.UIFigure); end
        end
    end
end

function createComponents(app)
    app.UIFigure = uifigure('Visible', 'off', 'Name', 'Mobile Manipulation Experiment Manager', ...
        'Position', [100 100 760 520], 'Color', [0.97 0.97 0.97]);
    app.UIFigure.CloseRequestFcn = @(~,~) delete(app);
    root = uigridlayout(app.UIFigure, [10 2]);
    root.RowHeight = {42, 36, 36, 36, 36, 36, 36, 42, 46, '1x'};
    root.ColumnWidth = {190, '1x'};
    root.Padding = [24 20 24 20];
    title = uilabel(root, 'Text', 'Mobile Manipulation Experiment Manager', ...
        'FontSize', 18, 'FontWeight', 'bold');
    title.Layout.Row = 1; title.Layout.Column = [1 2];

    [app.SceneDropDown, app.RobotDropDown, app.PlannerDropDown, ...
        app.StrategyDropDown, app.ConstraintDropDown, app.ExperimentModeDropDown] = ...
        createSelectors(root);

    app.RunButton = uibutton(root, 'push', 'Text', 'Run Experiment', 'ButtonPushedFcn', @(~,~) app.runButtonPushed());
    app.RunButton.Layout.Row = 8; app.RunButton.Layout.Column = [1 2];
    app.RunButton.BackgroundColor = [0.00 0.447 0.741]; app.RunButton.FontColor = [1 1 1];
    seed_label = uilabel(root, 'Text', 'Random Seed'); seed_label.Layout.Row = 9; seed_label.Layout.Column = 1;
    app.SeedField = uieditfield(root, 'numeric', 'Value', 2026, 'Limits', [0 inf], 'RoundFractionalValues', true);
    app.SeedField.Layout.Row = 9; app.SeedField.Layout.Column = 2;
    app.StatusText = uitextarea(root, 'Editable', 'off', 'Value', {'Ready. Select profiles and click Run Experiment.'});
    app.StatusText.Layout.Row = 10; app.StatusText.Layout.Column = [1 2];
    app.UIFigure.Visible = 'on';
end

function [scene, robot, planner, strategy, constraint, mode] = createSelectors(root)
    scene = add_selector(root, 'Scene', 2, {'baseline','strategy_comparison'});
    robot = add_selector(root, 'Robot', 3, {'robot_config'});
    robot.Enable = 'off';
    planner = add_selector(root, 'Planner', 4, {'convex_astar','astar','jps','theta_star'});
    strategy = add_selector(root, 'Strategy', 5, {'bottom_up','top_down','bidirectional'});
    constraint = add_selector(root, 'Constraint', 6, {'standard','strict scene','coverage required','no scene validation'});
    mode = add_selector(root, 'Experiment Mode', 7, {'Interactive','Headless','Headless + Save Record'});
end

function drop = add_selector(root, label_text, row, values)
    label = uilabel(root, 'Text', label_text); label.Layout.Row = row; label.Layout.Column = 1;
    drop = uidropdown(root, 'Items', values, 'Value', values{1});
    drop.Layout.Row = row; drop.Layout.Column = 2;
end
