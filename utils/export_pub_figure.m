function export_pub_figure(fig_handle, base_name)
%EXPORT_PUB_FIGURE 导出 300 dpi PNG + .fig 文件，并关闭 figure。
%
%   输入:
%     fig_handle  - figure 句柄
%     base_name   - 不含扩展名的文件名（不含路径分隔符）

    if ~isvalid(fig_handle)
        return;
    end

    set(fig_handle, 'PaperPositionMode', 'auto');
    set(fig_handle, 'InvertHardcopy', 'off');

    % PNG (300 dpi)
    print(fig_handle, [base_name, '.png'], '-dpng', '-r300');

    % FIG
    savefig(fig_handle, [base_name, '.fig'], 'compact');
end