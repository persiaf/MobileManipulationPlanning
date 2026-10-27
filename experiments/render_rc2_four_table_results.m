function manifest = render_rc2_four_table_results(raw_file, output_dir)
%RENDER_RC2_FOUR_TABLE_RESULTS Public S2-VIS entry point.
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
    manifest=render_rc2_four_table_style(raw_file,output_dir);
end
