function state = demo_complex_planning(profile_id, seed)
%DEMO_COMPLEX_PLANNING Show a sparse Bottom-Up planning overlay.

    if nargin < 1 || isempty(profile_id), profile_id = 'V2'; end
    if nargin < 2 || isempty(seed), seed = 2026; end
    profile_id = upper(char(profile_id));

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    override.planner.strategy.name = 'bottom_up';
    override.planner.routing.mode = 'sparse_knn';
    override.planner.routing.sparse_knn.k = 4;
    override.planner.candidate_pair_prefilter.enable = false;
    effective = resolve_effective_config('profile', profile_id, 'seed', seed, ...
        'experiment_override', override);
    print_effective_config(effective);
    vis_cfg = effective.visualization;
    state = load_current_demo_state(profile_id, seed, effective.planner);
    visualize_planning_overlay(state.scene, state.plan, state.stats, vis_cfg, ...
        'title', sprintf('%s Sparse Bottom-Up Planning Overlay', profile_id), ...
        'filename', sprintf('%s_planning', profile_id));
end
