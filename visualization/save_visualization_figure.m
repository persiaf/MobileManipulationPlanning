function save_visualization_figure(fig, vis_cfg, filename)
%SAVE_VISUALIZATION_FIGURE Save a maintained visualizer output when requested.

    if isempty(filename), return; end
    out_dir = vis_cfg.output_dir;
    if ~isfolder(out_dir), mkdir(out_dir); end
    if vis_cfg.save_png
        png_path = fullfile(out_dir, [filename '.png']);
        if exist('exportgraphics', 'file') == 2
            exportgraphics(fig, png_path, 'Resolution', vis_cfg.dpi);
        else
            print(fig, png_path, '-dpng', sprintf('-r%d', vis_cfg.dpi));
        end
    end
    if vis_cfg.save_fig
        savefig(fig, fullfile(out_dir, [filename '.fig']));
    end
end
