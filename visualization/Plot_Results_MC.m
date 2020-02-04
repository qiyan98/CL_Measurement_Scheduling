%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised August 2018%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot results
%%
clear
load CL_data_MC.mat;
%% Generate random colors
rng(0,'twister');
color_setting = cell(1,N);
for i = 1:N
    color_setting{i} = rand(1,3);
end
%% Plot aggregated trajectories
% figure;
% box on;
% hold on;
% for i = 1:N
%     plot(XX(1,:,i),XX(2,:,i),'--','color',color_setting{i},'linewidth',1);
% end
% for i = 1:N
%     plot(XX_hat_DR(1,:,i),XX_hat_DR(2,:,i),'-*','color',color_setting{i},'linewidth',0.3,'markersize',4,'MarkerIndices',1:0.1*k_f:k_f);
%     plot(XX_hat_CEKF_OPT(1,:,i),XX_hat_CEKF_OPT(2,:,i),'color',color_setting{i},'linewidth',1);
%     plot(XX_hat_CEKF_OPT(1,1,i),XX_hat_CEKF_OPT(2,1,i),'x','color',color_setting{i},'markersize',12);
% %     plot(XX_hat_CEKF(1,:,i),XX_hat_TEKF(2,:,i),'-*','color',color_setting{i},'linewidth',1,'markersize',1,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(XX_hat_CUKF(1,:,i),XX_hat_CUKF(2,:,i),'-*','color',color_setting{i},'linewidth',1,'markersize',4,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(XX_hat_ICI(1,:,i),XX_hat_ICI_EKF(2,:,i),'--+','color',color_setting{i},'linewidth',1,'markersize',4,'MarkerIndices',1:0.12*k_f:k_f);
% end
%
% str_lgd = cell(1,N);
% for i = 1:N
%     str_lgd{i} = ['Robot ',num2str(i)];
% end
% legend(str_lgd,'location','best');
% xlabel('x direction /m');
% ylabel('y direction /m');

%% Plot separate trajectories
% figure;
% row_subplot = ceil(N/4);
% for i = 1:N
%     subplot(row_subplot,4,i);
%     plot(XX(1,:,i),XX(2,:,i),'color',color_setting{i},'LineWidth',1)
%     hold on
%     plot(XX_hat_CD(1,:,i),XX_hat_CD(2,:,i),'--','color',color_setting{i},'LineWidth',1);
%     plot(XX_hat_SD(1,:,i),XX_hat_SD(2,:,i),'--o','color',color_setting{i},'linewidth',1,'markersize',2,'MarkerIndices',1:0.05*k_f:k_f);
%     plot(XX_hat_CD(1,1,i),XX_hat_CD(2,1,i),'x','color',color_setting{i},'MarkerSize',12);
%     legend(['Robot ',num2str(i)]);
% end
%% Absolute Error
% figure; box on;
% for i = 1:N
%     subplot(ceil(N/2),2,i);
%     box on;
%     hold on;
%     plot(Tk,abs(XX(1,:,i)-XX_hat_DR(1,:,i)),'LineWidth',1,'color','black');
%     plot(Tk,abs(XX(1,:,i)-XX_hat_CEKF(1,:,i)),'LineWidth',1);
%     plot(Tk,abs(XX(1,:,i)-XX_hat_CUKF(1,:,i)),'LineWidth',1);
%     xlabel('time /s');
%     ylabel('Abs error in X /m');
%     xlim([0 t_f]);
%         legend('DR','CEKF','CUKF','location','best');
%     legend('DR','CEKF','location','best');
%     title(['Robot ',num2str(i)]);
% end
% %% RMSE
% figure;
% ha = tightPlots(3,3,6.85,[5 5],[0.3 0.3],[0.4 0.3],[0.4 1.4],'inch');
% box on;
% for i = 1:N
%     %     subplot(ceil(N/3),3,i);
%     axes(ha(i));
%     box on;
%     hold on;
%     plot(Tk,RMSE_CEKF_Dense_mont(:,i),'linewidth',1);
%     %     plot(Tk,RMSE_CEKF_Random(:,i),'linewidth',1);
%     %     plot(Tk,RMSE_CEKF_subMOD_mont(:,i),'linewidth',1);
%     plot(Tk,RMSE_SAEKF_subMOD_mont(:,i),'linewidth',1);
%     plot(Tk,RMSE_CEKF_subOPT_mont(:,i),'linewidth',1);
%     plot(Tk,RMSE_CEKF_subMOD_multi_mont(:,i),'linewidth',1);
%     plot(Tk,RMSE_CEKF_subOPT_multi_mont(:,i),'linewidth',1);
%     plot(Tk,RMSE_DR_mont(:,i),'linewidth',1);
%     xlim([0 t_f]);
%     %     xlabel('time/s','fontsize',14);
%     if mod(i-1,3) == 0
%         ylabel('RMS error/m','fontsize',12);
%     end
%     if mod(i-2,3) == 0 && i+3>N
%         xlabel('Time/s','fontsize',12);
%     end
%     %     title(['Robot ',num2str(i)],'fontsize',14);
%     text(0.05,0.9,['Robot ',num2str(i)],'Units','normalized');
% end
% h_lgd = legend('Dense-CL','OPT-CL','SubOPT-CL','OPT-Multi-CL','SubOPT-Multi-CL','DR','fontsize',10);
% set(h_lgd,'position',[0.895 0.4 0.01 0.2],'Units','normalized');
% saveas(gcf,'fig_output\rmse_MC','epsc');
%% Aggregated RMSE
figure;
ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
set(gcf,'color','white');
box on;
hold on;

ind_marker_dense = 1:length(Tk);
ind_marker_sparse = 1:20:length(Tk);

plot(Tk,sum(RMSE_CEKF_subMOD_mont,2),'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
plot(Tk,sum(RMSE_CEKF_subMOD_multi_mont,2),'--','linewidth',4,'Color', 'k','markerindices',ind_marker_dense);

ind_marker_sparse = 1:3:length(Tk);
plot(Tk,sum(RMSE_CEKF_subOPT_mont,2),'.','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
ind_marker_sparse = 1:14:length(Tk);
plot(Tk,sum(RMSE_CEKF_subOPT_multi_mont,2),'.','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',12);

plot(Tk,sum(RMSE_CEKF_Random_mont,2),'--','linewidth',2,'Color', 'b','markerindices',ind_marker_dense);
plot(Tk,sum(RMSE_CEKF_Random_multi_mont,2),'--','linewidth',4,'Color', 'b','markerindices',ind_marker_dense);
plot(Tk,sum(RMSE_CEKF_Dense_mont,2),'k-','linewidth',4,'markerindices',ind_marker_sparse,'markersize',14);

ind_marker_sparse = 1:20:length(Tk);
plot(Tk,sum(RMSE_DR_mont,2),'r--*','linewidth',2,'markerindices',ind_marker_sparse,'markersize',6);

legend('[19], q^i = 1','[19], q^i = 3','Alg. 1, q^i = 1','Alg. 1, q^i = 3',...
    'random, q^i = 1', 'random, q^i = 3','q^i = 8','DR',...
    'position',[0.27 0.7 0.15 0.2],'units','normalized',...
    'NumColumns',2);

grid on;

set(gca,...
'Units','normalized',...
'XTick',0:10:Tk(end),...
'YTick',0:1:8,...
'Position',[.14 .15 .83 .82],...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',26,...
'FontWeight','bold',...
'FontName','Times')

ylabel({'Averaged aggretaed RMSE /m'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',28,...
'FontName','Times')


xlabel('Time (s)',...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',28,...
'FontName','Times')

% saveas(gcf,'fig_output\rmse_MC','epsc');
saveas(gcf,'fig_output\rmse_MC','pdf');
%% D-Accuracy
figure;
ha = tightPlots(1,1,6.85,[5 3],[0.3 0.3],[0.4 0.3],[0.5 0.4],'inch');
box on;
hold on;
plot(Tk,D_ACC_CEKF_Dense_mont,'linewidth',1);
%     plot(Tk,D_ACC_CEKF_Random,'linewidth',1);
%     plot(Tk,D_ACC_CEKF_subMOD_mont,'linewidth',1);
plot(Tk,D_ACC_SAEKF_subMOD_mont,'linewidth',1);
plot(Tk,D_ACC_CEKF_subOPT_mont,'linewidth',1);
plot(Tk,D_ACC_CEKF_subMOD_multi_mont,'linewidth',1);
plot(Tk,D_ACC_CEKF_subOPT_multi_mont,'linewidth',1)
plot(Tk,D_ACC_CEKF_Random_mont,'linewidth',1);
plot(Tk,D_ACC_CEKF_Random_multi_mont,'linewidth',1);
plot(Tk,D_ACC_DR_mont,'linewidth',1);
xlim([0 t_f]);
ylabel('D-accuracy/m','fontsize',12);
xlabel('Time/s','fontsize',12);
%     title(['Robot ',num2str(i)],'fontsize',14);
h_lgd = legend('Dense-CL','OPT-CL','SubOPT-CL','OPT-Multi-CL','SubOPT-Multi-CL',...
    'Random','Rondom-multi','DR','fontsize',10,'location','best');
% set(h_lgd,'position',[0.895 0.4 0.01 0.2],'Units','normalized');
%% Error with 3-sigma boundaries in X-direction
% figure;
% ha = tightPlots(3,3,6.85,[5 5],[0.3 0.3],[0.4 0.2],[0.4 0.4],'inch');
% box on;
% for i = 1:N
% %     subplot(ceil(N/3),3,i);
%     axes(ha(i));
%     box on;
%     hold on;
% %     plot(Tk,+Three_sigma_CEKF(1,:,i),[color_plot,'-.'],'LineWidth',1);
% %     plot(Tk,-Three_sigma_CEKF(1,:,i),[color_plot,'-.'],'LineWidth',1);
% %     plot(Tk,XX(1,:,i)-XX_hat_CEKF(1,:,i),'red','LineWidth',1);
% %     title(['Robot ',num2str(i)]);
%
% %     plot(Tk,+Three_sigma_CEKF_Random(1,:,i),['blue','--*'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(Tk,-Three_sigma_CEKF_Random(1,:,i),['blue','--*'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(Tk,XX(1,:,i)-XX_hat_CEKF_Random(1,:,i),'b-*','LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%
%     plot(Tk,+Three_sigma_SAEKF_OPT(1,:,i),['red','--^'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%     plot(Tk,-Three_sigma_SAEKF_OPT(1,:,i),['red','--^'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%     plot(Tk,XX(1,:,i)-XX_hat_SAEKF_OPT(1,:,i),'r-*','LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%
% %     plot(Tk,+Three_sigma_CUKF(1,:,i),['blue','--*'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(Tk,-Three_sigma_CUKF(1,:,i),['blue','--*'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(Tk,XX(1,:,i)-XX_hat_CUKF(1,:,i),'b-*','LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%
%     plot(Tk,+Three_sigma_DR(1,:,i),['black','--+'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%     plot(Tk,-Three_sigma_DR(1,:,i),['black','--+'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
% %     plot(Tk,XX(1,:,i)-XX_hat_DR(1,:,i),'black-+','LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%
%     min_y = min(ylim); max_y = max(ylim);
%     if Missed_on_off==0
%         for i_mea=1:length(RelMea_Table)
%             if(RelMea_Table{i_mea}(1,1) > 0) % if it is a reasonable measurement
%                 mea_start = RelMea_Table{i_mea}(1,1)*delta;
%                 mea_end = RelMea_Table{i_mea}(1,2)*delta;
%                 line([floor(mea_start),floor(mea_start)],[min_y,max_y],'color','green','linestyle','--','linewidth',1)
%                 line([floor(mea_end),floor(mea_end)],[min_y,max_y],'color','blue','linestyle','--','linewidth',1)
%             end
%         end
% %         axis([0 250 -10 10])
%         xlim([0 t_f]);
%     else
% %         axis([0 250 -30 30])
%     end
% %     xlabel('Time/s');
% %     ylim([-10 10]);
%     if mod(i-2,3) == 0 && i+3>N
%     xlabel('Time/s','fontsize',12);
%     end
%     if mod(i-1,3) == 0
%     ylabel('Error/m','fontsize',12);
%     end
%     text(0.05,0.9,['Robot ',num2str(i)],'Units','normalized');
% end
% saveas(gcf,'fig_output\3sigma','epsc');
%% NEES result
% figure; box on;
% hold on;
% plot(Tk,NEES_DR,'linewidth',1,'color','black');
%
% plot(Tk,NEES_CEKF_Dense,'linewidth',1);
% plot(Tk,NEES_CEKF_Random,'linewidth',1);
% plot(Tk,NEES_SAEKF_OPT,'linewidth',1);
% % plot(Tk,NEES_CEKF_OPT,'linewidth',1);
% line([0 max(xlim)],[1 1],'linewidth',1,'color','red','linestyle','--');
% % ylim([0 3]);
% % ylim([0 max(NEES_CEKF_OPT)]);
% legend('DR','Dense-CL','Random-CL','OPT-CL','NEES = 1');
% xlabel('Time/s');
% ylabel('NEES');
%% plot trace/logdet of the aggregated covariance matrix
figure;
ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
set(gcf,'color','white');
box on;
hold on;

ind_marker_dense = 1:4:length(Tk);
ind_marker_sparse = 1:11:length(Tk);
plot(Tk,log(Det_CEKF_Dense_mont),'k-','linewidth',5);
plot(Tk,log(Det_CEKF_subMOD_mont),'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
plot(Tk,log(Det_CEKF_subOPT_mont),'.','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_dense,'markersize',9);
plot(Tk,log(Det_CEKF_subMOD_multi_mont),'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
plot(Tk,log(Det_CEKF_subOPT_multi_mont),'.','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',11);

plot(Tk,log(Det_CEKF_Random_mont),'--','linewidth',2,'Color', 'b','markerindices',ind_marker_dense);
plot(Tk,log(Det_CEKF_Random_multi_mont),'--','linewidth',5,'Color', 'b','markerindices',ind_marker_dense);
ylim([-150 -80]);

grid on;
set(gca,...
'Units','normalized',...
'XTick',0:10:Tk(end),...
'Position',[.14 .15 .83 .82],...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',28,...
'FontWeight','bold',...
'FontName','Times')

ylabel({'Logarithm of averaged'; 'determinant of covariance'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',28,...
'FontName','Times')


xlabel('Time (s)',...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',28,...
'FontName','Times')

saveas(gcf,'fig_output\logdet_MC','epsc');
saveas(gcf,'fig_output\logdet_MC','pdf');
%% Communication times evolution
% figure;
% subplot(2,1,1);
% hold on;
% box on;
% plot(Tk,count_com_TEKF(:,1),'linewidth',1);
% plot(Tk,count_com_SATEKF(:,1),'linewidth',1);
% plot(Tk,count_com_TEKF(:,2),'linewidth',1);
% ylabel('# Communications');
% legend('TEKF','SA-TEKF','CEKF','location','best');
% subplot(2,1,2);
% hold on; box on;
% plot(Tk,count_com_TEKF(:,1)./count_com_TEKF(:,2),'linewidth',1);
% ylabel('Ratio');
% xlabel('Time/s');
%% Topology changing
% G = digraph();
% for i=1:N
%     for j=1:N
%         if i~=j
%             G = addedge(G,i,j);
%         end
%     end
% end
% figure;
% ha = tightPlots(1,1,6.85*.4,[5 4],[0.9 0.8],[0.1 0.1],[0.1 0.1],'inch');
% plot(G);
% saveas(gcf,'fig_output\topology_dense','pdf');

%%
% G = cell(1,5);
% for i=1:5
%     G{i} = digraph();
%     G{i} = addnode(G{i},N);
% end
% cur_edge = zeros(3,2); ii = 1;
% time_edge = zeros(5,1);
% for i=1:length(mea_schedule)
%     if ~isempty(mea_schedule{i})
%         if ~isequal(mea_schedule{i}(:,2:3),cur_edge)
%             cur_edge = mea_schedule{i}(:,2:3);
%             G{ii} = digraph(table(cur_edge,'VariableNames',{'EndNodes'}));
%             time_edge(ii) = i*delta;
% %             if G{ii}.numnodes~=10
% %                 disp('bad!');
% %             end
%             ii = ii + 1;
%         end
%     end
% end
%%
% figure;
% ha = tightPlots(3,4,6.85,[5 5],[0.3 0.2],[0.3 0.05],[0.1 0.05],'inch');
% for i=1:min(length(G),12)
%     j = 3*i;
% %     subplot(3,4,i);
%     axes(ha(i));
%     plot(G{j});
% %     xlabel(['t = ',num2str(time_edge(j)),'s, # Edges = ',num2str(G{j}.numedges)]);
%     xlabel(['t = ',num2str(time_edge(j)),'s'],'fontsize',14);
% end
% saveas(gcf,'fig_output\topology_sparse','pdf');

%%
figure;
ha = tightPlots(1,1,3.2*4,[3.2 1],[0 0],[0 0],[0 0],'inch');
% figure('Units','inches','Position',[0 0 3.2 1]*2,'PaperPositionMode','auto');
for jj = 1
%     axes(ha(1));
    set(gcf,'color','w'); hold on; box on; grid on;
    for ii = 150:150:k_f
%     for ii = 50:5:k_f
        cur_mea = mea_schedule_subOPT_multi{ii};
        cur_mea = cur_mea(:,2:3);
        cur_mea(cur_mea(:,1) == cur_mea(:,2),:) = []; % delete absolute measurements
        
        cur_mea_tmp = cur_mea(cur_mea(:,1) == 3,:);
        cur_tmp = ii*ones(length(cur_mea_tmp),1)*0.1;
        plot(cur_tmp,cur_mea_tmp(:,2),'x','color','k','markersize',14,'linewidth',1.2);
        
        cur_mea_tmp = cur_mea(cur_mea(:,1) == 6,:);
        cur_tmp = ii*ones(length(cur_mea_tmp),1)*0.1;
        plot(cur_tmp,cur_mea_tmp(:,2),'o','color','k','markersize',14,'linewidth',1.2);
        
        cur_mea_tmp = cur_mea(cur_mea(:,1) == 9,:);
        cur_tmp = ii*ones(length(cur_mea_tmp),1)*0.1;
        plot(cur_tmp,cur_mea_tmp(:,2),'+','color','k','markersize',14,'linewidth',1.2);
    end
    xlim([0 100]);
    ylim([0 10]);
    set(gca,'fontsize',16);
    xlabel('Time (s)','fontsize',14);
    ylabel('Landmark robot ID','fontsize',14);
%     title('Landmark Selection of Master Robot 3','fontsize',14);
    xticks([0:15:90,100]);
    yticks([1:9]);
end
grid on;
set(gca,...
'Units','normalized',...
'XTick',[0:15:90,100],...
'Position',[.14 .25 .83 .70],...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',24,...
'FontWeight','bold',...
'FontName','Times')

ylabel({'Landmark robot ID'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',22,...
'FontName','Times')


xlabel('Time (s)',...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',24,...
'FontName','Times')

saveas(gcf,'fig_output\landmark_selection_MC','epsc');