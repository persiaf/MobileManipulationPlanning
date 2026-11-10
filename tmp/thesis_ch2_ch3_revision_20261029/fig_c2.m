function fig_c2(outdir,repo)
figure_style(); seeds=[2026 2027 2028]; counts=zeros(3,3);
for j=1:numel(seeds)
    path=fullfile(repo,'results','repair_20261027','evidence_20261027_221823','complex','cases',sprintf('C2_seed%d_task_audit.csv',seeds(j)));
    t=readtable(path); base=logical(t.production_library_covered); extra=~base & logical(t.independent_dense_support_found);
    counts(j,:)=[sum(base),sum(extra),sum(~base & ~extra)];
end
fractions=counts./sum(counts,2)*100;
writematrix([seeds(:),counts,fractions],fullfile(outdir,'c2_plot_values.csv'));
f=figure('Units','inches','Position',[1 1 5.8 3.5]);
bars=bar(1:3,fractions,'stacked','BarWidth',.48); hold on;
colors=[.16 .38 .59;.90 .60 .20;.83 .83 .83];
for k=1:3, bars(k).FaceColor=colors(k,:); bars(k).EdgeColor=[.25 .25 .25]; end
for i=1:3
    y=0;
    for k=1:3
        col='k'; if k==1,col='w';end
        text(i,y+fractions(i,k)/2,sprintf('%d',counts(i,k)),'HorizontalAlignment','center','Color',col,'FontName','Times New Roman','FontWeight','bold');
        y=y+fractions(i,k);
    end
end
set(gca,'XTick',1:3,'XTickLabel',string(seeds),'YLim',[0 100],'YTick',0:20:100,'TickDir','out','Box','off');
xlabel('随机种子','FontName','SimSun'); ylabel('任务占比 / %','FontName','SimSun');
legend(bars,{'原候选库覆盖','加密搜索新增见证','有限搜索未找到'},'Location','northoutside','Orientation','horizontal','FontName','SimSun','FontSize',9,'Box','off');
exportgraphics(f,fullfile(outdir,'fig3_2_c2.png'),'Resolution',300);
exportgraphics(f,fullfile(outdir,'fig3_2_c2.pdf'),'ContentType','vector'); close(f);
end
