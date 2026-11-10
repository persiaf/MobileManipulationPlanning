function fig_proxy(outdir,cfg)
figure_style(); x=linspace(-.14,.58,181); y=linspace(-.43,.43,201); [X,Y]=meshgrid(x,y); Z=zeros(size(X));
for k=1:numel(X)
    left=evaluate_single_arm_reachability([0 0 0],[X(k) Y(k) .5],'left',cfg,.30);
    right=evaluate_single_arm_reachability([0 0 0],[X(k) Y(k) .5],'right',cfg,.30);
    Z(k)=max(left,right);
end
save(fullfile(outdir,'proxy_grid.mat'),'X','Y','Z');
f=figure('Units','inches','Position',[1 1 5.4 4.1]);
im=imagesc(x,y,Z); set(im,'AlphaData',Z>0); axis xy equal tight; hold on;
colormap(parula(128)); caxis([.30 .65]); cb=colorbar; cb.Label.String='\rho'; cb.Label.Interpreter='tex';
rectangle('Position',[-cfg.chassis.length/2 -cfg.chassis.width/2 cfg.chassis.length cfg.chassis.width],'EdgeColor',[.2 .2 .2],'LineStyle','--');
plot(cfg.arm_base_mount_x,cfg.dual_arm.base_spacing/2,'ko','MarkerFaceColor','w','MarkerSize',6);
plot(cfg.arm_base_mount_x,-cfg.dual_arm.base_spacing/2,'ks','MarkerFaceColor','w','MarkerSize',6);
text(cfg.arm_base_mount_x+.015,cfg.dual_arm.base_spacing/2+.04,'A_L','Interpreter','tex');
text(cfg.arm_base_mount_x+.015,-cfg.dual_arm.base_spacing/2-.04,'A_R','Interpreter','tex');
xlim([-.23 .60]); ylim([-.44 .44]);
xlabel('x_B / m','Interpreter','tex'); ylabel('y_B / m','Interpreter','tex');
set(gca,'TickDir','out','Box','off');
exportgraphics(f,fullfile(outdir,'fig3_1_proxy.png'),'Resolution',300);
exportgraphics(f,fullfile(outdir,'fig3_1_proxy.pdf'),'ContentType','vector'); close(f);
end
