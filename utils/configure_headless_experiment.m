function cleanup_obj = configure_headless_experiment()
%CONFIGURE_HEADLESS_EXPERIMENT Disable MATLAB figure display for batch runs.
%   cleanup_obj restores the previous DefaultFigureVisible state when the
%   caller exits, including errors.

    old_visible = get(0, 'DefaultFigureVisible');
    close all force;
    set(0, 'DefaultFigureVisible', 'off');
    cleanup_obj = onCleanup(@() restore_figure_state(old_visible));
end

function restore_figure_state(old_visible)
    try
        close all force;
        set(0, 'DefaultFigureVisible', old_visible);
    catch
        set(0, 'DefaultFigureVisible', 'on');
    end
end
