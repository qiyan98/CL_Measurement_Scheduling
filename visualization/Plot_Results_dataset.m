%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised August 2018%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot results
%%
clear
load CL_data_dataset.mat;
%% Generate random colors
rng(0,'twister');
color_setting = cell(1,N);
for i = 1:N
    color_setting{i} = rand(1,3);
end
%% Plot aggregated trajectories
figure;
box on;
hold on;
for i = 1:N
    plot(XX(1,:,i),XX(2,:,i),'--','color',color_setting{i},'linewidth',1);
end
for i = 1:N
    plot(XX_hat_DR(1,:,i),XX_hat_DR(2,:,i),'-*','color',color_setting{i},'linewidth',0.3,'markersize',4,'MarkerIndices',1:floor(0.1*k_f):k_f);
    plot(XX_hat_CEKF_subMOD(1,:,i),XX_hat_CEKF_subMOD(2,:,i),'color',color_setting{i},'linewidth',1);
    plot(XX_hat_CEKF_subMOD(1,1,i),XX_hat_CEKF_subMOD(2,1,i),'x','color',color_setting{i},'markersize',12);
    plot(XX_hat_CEKF_subOPT(1,:,i),XX_hat_CEKF_subOPT(2,:,i),'color',color_setting{i},'linewidth',1);
    plot(XX_hat_CEKF_subOPT(1,1,i),XX_hat_CEKF_subOPT(2,1,i),'x','color',color_setting{i},'markersize',12);
end

str_lgd = cell(1,N);
for i = 1:N
    str_lgd{i} = ['Robot ',num2str(i)];
end
legend(str_lgd,'location','best');
xlabel('x direction /m');
ylabel('y direction /m');
set(gca,'fontsize',14);

%% Plot separate trajectories
% figure;
% row_subplot = ceil(N/4);
% for i = 1:N
%     subplot(row_subplot,4,i);
%     plot(XX(1,:,i),XX(2,:,i),'color',color_setting{i},'LineWidth',1)
%     hold on
% %     plot(XX_hat_CEKF_subMOD(1,:,i),XX_hat_CEKF_subMOD(2,:,i),'--','color',color_setting{i},'LineWidth',1);
% %     plot(XX_hat_CEKF_subOPT(1,:,i),XX_hat_CEKF_subOPT(2,:,i),'--o','color',color_setting{i},'linewidth',1,'markersize',2,'MarkerIndices',1:0.05*k_f:k_f);
%     plot(XX_hat_DR(1,1,i),XX_hat_DR(2,1,i),'x','color',color_setting{i},'MarkerSize',12);
%     legend(['Robot ',num2str(i)]);
% end
%% Absolute Error - x direction
% figure; box on;
% for i = 1:N
%     subplot(ceil(N/2),2,i);
%     box on;
%     hold on;
%     plot(Tk,abs(XX(1,:,i)-XX_hat_DR(1,:,i)),'LineWidth',1,'color','black');
%     plot(Tk,abs(XX(1,:,i)-XX_hat_CEKF_subMOD(1,:,i)),'LineWidth',1);
%     plot(Tk,abs(XX(1,:,i)-XX_hat_CEKF_subOPT(1,:,i)),'LineWidth',1);
%     xlabel('time /s');
%     ylabel('Abs error in X /m');
%     xlim([0 t_f]);
%     legend('DR','subMOD','subOPT','location','best');
%     title(['Robot ',num2str(i)]);
% end
% %% RMSE
% figure;
% ha = tightPlots(3,3,6.85,[5 5],[0.3 0.3],[0.4 0.3],[0.4 1.4],'inch'); % for generated data
% % ha = tightPlots(2,3,6.85,[5 5],[0.3 0.3],[0.4 0.3],[0.4 1.4],'inch'); % for dataset
% box on;
% for i = 1:N
%     axes(ha(i));
%     box on;
%     hold on;
%     plot(Tk,RMSE_CEKF_Dense(:,i),'linewidth',1);
%     plot(Tk,RMSE_SAEKF_subMOD(:,i),'linewidth',1);
%     plot(Tk,RMSE_CEKF_subOPT(:,i),'linewidth',1);
%     plot(Tk,RMSE_DR(:,i),'linewidth',1);
%     xlim([0 t_f]);
%     if mod(i-1,3) == 0
%     ylabel('RMS error/m','fontsize',12);
%     end
%     if mod(i-2,3) == 0 && i+3>N
%     xlabel('Time/s','fontsize',12);
%     end
% %     title(['Robot ',num2str(i)],'fontsize',14);
%     ylim([0 1.5]);
%     text(0.05,0.9,['Robot ',num2str(i)],'Units','normalized');
% end
% h_lgd = legend('Dense-CL','subMOD','subOPT','DR','fontsize',10);
% set(h_lgd,'position',[0.895 0.4 0.01 0.2],'Units','normalized');
% saveas(gcf,'fig_output\rmse','epsc');
%% Aggregated RMSE
figure;
ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
set(gcf,'color','white');
box on; hold on;

ind_marker_dense = 1:10:length(Tk);
ind_marker_sparse = 1:10:length(Tk);


plot(Tk,sum(RMSE_CEKF_subMOD,2),'--','linewidth',2,'Color', 'b','markerindices',ind_marker_dense);
plot(Tk,sum(RMSE_CEKF_subMOD_multi,2),'--','linewidth',4,'Color', '#0072BD','markerindices',ind_marker_dense);
plot(Tk,sum(RMSE_CEKF_subOPT,2),'-','linewidth',2,'Color', 'y','markerindices',ind_marker_sparse,'markersize',10);
plot(Tk,sum(RMSE_CEKF_subOPT_multi,2),'-','linewidth',4,'Color','#EDB120','markerindices',ind_marker_sparse,'markersize',10);

% plot(Tk,sum(RMSE_CEKF_Random,2),'-','linewidth',2,'Color', 'r','markerindices',ind_marker_sparse,'markersize',10);
% plot(Tk,sum(RMSE_CEKF_Random_multi,2),'-','linewidth',4,'Color','r','markerindices',ind_marker_sparse,'markersize',10);

plot(Tk,sum(RMSE_CEKF_Dense,2),'k--','linewidth',2,'markerindices',ind_marker_sparse,'markersize',10);
plot(Tk,sum(RMSE_DR,2),'k-','linewidth',4);

%     'random, q^i = 1','random, q^i = 3',...
legend('[19],q^i = 1','[19], q^i = 3','our, q^i = 1','our, q^i = 3',...
    'dense mea.','DR',...
    'position',[0.2 0.8 0.1 0.01],'units','normalized');

set(gca,'fontsize',16);
% ylabel('Averaged aggretaed RMSE /m','fontsize',14);
xlabel('Time /s','fontsize',14);

set(gca,...
'Units','normalized',...
'XTick',0:20:Tk(end),...
'Position',[.14 .15 .83 .82],...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',16,...
'FontWeight','bold',...
'FontName','Times')

ylabel({'Aggretaed RMSE /m'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',20,...
'FontName','Times')


xlabel('Time (s)',...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontWeight','bold',...
'FontSize',20,...
'FontName','Times')

saveas(gcf,'fig_output\rmse_dataset','epsc');
%% D-Accuracy
figure;
ha = tightPlots(1,1,6.85,[5 3],[0.3 0.3],[0.4 0.3],[0.5 0.4],'inch');
box on;
hold on;
plot(Tk,D_ACC_CEKF_Dense,'linewidth',1);
%     plot(Tk,D_ACC_CEKF_Random,'linewidth',1);
plot(Tk,D_ACC_CEKF_subMOD,'linewidth',1);
plot(Tk,D_ACC_CEKF_subMOD_multi,'linewidth',1,'linestyle','--');
plot(Tk,D_ACC_CEKF_subOPT,'linewidth',1);
plot(Tk,D_ACC_CEKF_subOPT_multi,'linewidth',1,'linestyle','--');
plot(Tk,D_ACC_DR,'linewidth',1);
xlim([0 t_f]);
ylabel('D-accuracy/m','fontsize',12);
xlabel('Time/s','fontsize',12);
h_lgd = legend('Dense-CL','SubMOD-CL','subMOD-multi','SubOPT-CL','subOPT-multi','DR','fontsize',10,'location','best');

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
%     plot(Tk,+Three_sigma_SAEKF_SubMOD(1,:,i),['red','--^'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%     plot(Tk,-Three_sigma_SAEKF_SubMOD(1,:,i),['red','--^'],'LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
%     plot(Tk,XX(1,:,i)-XX_hat_SAEKF_SubMOD(1,:,i),'r-*','LineWidth',1,'markersize',2,'MarkerIndices',1:0.1*k_f:k_f);
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
% plot(Tk,NEES_SAEKF_SubMOD,'linewidth',1);
% % plot(Tk,NEES_CEKF_SubMOD,'linewidth',1);
% line([0 max(xlim)],[1 1],'linewidth',1,'color','red','linestyle','--');
% % ylim([0 3]);
% % ylim([0 max(NEES_CEKF_SubMOD)]);
% legend('DR','Dense-CL','Random-CL','OPT-CL','NEES = 1');
% xlabel('Time/s');
% ylabel('NEES');
%% plot trace/logdet of the aggregated covariance matrix
figure;
ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
set(gcf,'color','white'); box on; hold on;

ind_marker_dense = 1:3:length(Tk);

plot(Tk,Logdet_CEKF_Dense,'k-','linewidth',4);

ind_marker_sparse = 1:10:length(Tk);
plot(Tk,Logdet_CEKF_subMOD,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
plot(Tk,Logdet_CEKF_subOPT,'.','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_dense,'markersize',6);
ind_marker_sparse = 1:20:length(Tk);
plot(Tk,Logdet_CEKF_subMOD_multi,'--','linewidth',4,'Color', 'k','markerindices',ind_marker_dense);
plot(Tk,Logdet_CEKF_subOPT_multi,'.','linewidth',4,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',10);

ind_marker_sparse = 1:30:length(Tk);
plot(Tk,Logdet_CEKF_Random,'--','linewidth',2,'Color', 'b','markerindices',ind_marker_sparse,'markersize',15);
plot(Tk,Logdet_CEKF_Random_multi,'--','linewidth',4,'Color', 'b','markerindices',ind_marker_sparse,'markersize',20);

ylim([-95 -40]);
grid on;

set(gca,...
'Units','normalized',...
'XTick',0:30:Tk(end),...
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
legend('q^i = 4',...
    '[19], q^i = 1','Alg. 1, q^i = 1',...
    '[19], q^i = 3','Alg. 1, q^i = 3',...
    'random, q^i = 1', 'random, q^i = 3',...
    'position',[0.27 0.7 0.15 0.2],'units','normalized',...
    'NumColumns',2);
% saveas(gcf,'fig_output\logdet_dataset','epsc');
saveas(gcf,'fig_output\logdet_dataset','pdf');
%% Computational timing
figure; hold on; box on;
timing_subOPT = timing_subOPT(timing_subOPT>0);
timing_subMOD = timing_subMOD(timing_subMOD>0);
bar(categorical({'subOPT','subModular'}),[mean(timing_subOPT),mean(timing_subMOD)]);
ylabel('Running time per robot /ms');
set(gca,'fontsize',14);
title(['q^i = ',num2str(q_i)]);
% legend('SubOPT','SubModular');

%% # of communications evolution
% figure;
% ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
% set(gcf,'color','white'); box on; hold on;
% comm_cur_CEKF_subMOD = zeros(length(Tk),1);
% comm_cur_CEKF_subOPT = comm_cur_CEKF_subMOD;
% comm_cur_CEKF_subMOD_multi = comm_cur_CEKF_subMOD;
% comm_cur_CEKF_subOPT_multi = comm_cur_CEKF_subMOD;
% for i=2:length(comm_CEKF_subMOD)
%     comm_cur_CEKF_subMOD(i,:) = comm_CEKF_subMOD(i,:) - comm_CEKF_subMOD(i-1,:);
%     comm_cur_CEKF_subOPT(i,:) = comm_CEKF_subOPT(i,:) - comm_CEKF_subOPT(i-1,:);
%     comm_cur_CEKF_subMOD_multi(i,:) = comm_CEKF_subMOD_multi(i,:) - comm_CEKF_subMOD_multi(i-1,:);
%     comm_cur_CEKF_subOPT_multi(i,:) = comm_CEKF_subOPT_multi(i,:) - comm_CEKF_subOPT_multi(i-1,:);
% end
% 
% comm_saved_subOPT = zeros(length(Tk),1);
% comm_saved_subOPT_multi = comm_saved_subOPT;
% comm_cur_saved_subOPT_multi = comm_saved_subOPT;
% for i=1:length(comm_CEKF_subMOD)
%     comm_saved_subOPT(i,:) = comm_CEKF_subMOD(i,:) - comm_CEKF_subOPT(i,:);
%     comm_saved_subOPT_multi(i,:) = comm_CEKF_subMOD_multi(i,:) - comm_CEKF_subOPT_multi(i,:);
% end
% 
% for i = 2:length(comm_cur_saved_subOPT_multi)
%     comm_cur_saved_subOPT_multi(i,:) = comm_saved_subOPT_multi(i,:) - comm_saved_subOPT_multi(i-1,:);
% end
% 
% ind_marker_dense = 1:length(Tk);
% ind_marker_sparse = 1:3:length(Tk);
% 
% plot(Tk,comm_CEKF_subMOD,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,comm_CEKF_subOPT,'.','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,comm_CEKF_subMOD_multi,'--','linewidth',4,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,comm_CEKF_subOPT_multi,'.','linewidth',4,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% plot(Tk,comm_cur_CEKF_subMOD,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,comm_cur_CEKF_subOPT,'-','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,comm_cur_CEKF_subMOD_multi,'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,comm_cur_CEKF_subOPT_multi,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% plot(Tk,comm_CEKF_subMOD,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,comm_saved_subOPT,'-','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,comm_CEKF_subMOD_multi,'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,zeros(1,length(Tk)),'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,comm_saved_subOPT_multi,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% plot(Tk,comm_cur_saved_subOPT_multi,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% legend("scheduling overhead of ours","scheduling overhead of [19]");
% set(gca,'fontsize',16);
% xlabel('Time /s','fontsize',14);
% ylim([-95 -55]);
% 
% set(gca,...
% 'Units','normalized',...
% 'XTick',0:10:Tk(end),...
% 'Position',[.1 .12 .88 .82],...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',16,...
% 'FontWeight','bold',...
% 'FontName','Times')
% 
% ylabel({'Number of communications in current step'},...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontWeight','bold',...
% 'FontSize',20,...
% 'FontName','Times')
% 
% 
% xlabel('Time (s)',...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontWeight','bold',...
% 'FontSize',20,...   
% 'FontName','Times')
% 
% saveas(gcf,'fig_output\num_comm_dataset','epsc');
%% Total transmitted data evolution
% bytes_cur_subMOD = zeros(length(Tk),1);
% bytes_cur_subOPT = bytes_cur_subMOD;
% bytes_cur_subMOD_multi = bytes_cur_subMOD;
% bytes_cur_subOPT_multi = bytes_cur_subMOD;
% for i=2:length(comm_CEKF_subMOD)
%     bytes_cur_subMOD(i,:) = bytes_subMOD(i,:) - bytes_subMOD(i-1,:);
%     bytes_cur_subOPT(i,:) = bytes_subOPT(i,:) - bytes_subOPT(i-1,:);
%     bytes_cur_subMOD_multi(i,:) = bytes_subMOD_multi(i,:) - bytes_subMOD_multi(i-1,:);
%     bytes_cur_subOPT_multi(i,:) = bytes_subOPT_multi(i,:) - bytes_subOPT_multi(i-1,:);
% end
% 
% bytes_saved_subOPT = zeros(length(Tk),N);
% bytes_saved_subOPT_multi = bytes_cur_subMOD;
% for i=1:length(comm_CEKF_subMOD)
%     bytes_saved_subOPT(i,:) = bytes_subMOD(i,:) - bytes_subOPT(i,:);
%     bytes_saved_subOPT_multi(i,:) = bytes_subMOD_multi(i,:) - bytes_subOPT_multi(i,:);
% end
% 
% ind_marker_dense = 1:length(Tk);
% ind_marker_sparse = 1:3:length(Tk);
% 
% figure;
% ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
% set(gcf,'color','white'); box on; hold on;
% 
% plot(Tk,bytes_subMOD/1e6,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,bytes_subOPT/1e6,'.','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,bytes_subMOD_multi/1e6,'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,bytes_subOPT_multi/1e6,'.','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% plot(Tk,bytes_cur_subMOD/1e3,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,bytes_cur_subOPT/1e3,'-','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,bytes_cur_subMOD_multi/1e3,'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,bytes_cur_subOPT_multi/1e3,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% plot(Tk,bytes_saved_subOPT/1e6,'-','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,bytes_saved_subOPT_multi/1e6,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% grid on;
% set(gca,...
% 'Units','normalized',...
% 'XTick',0:30:Tk(end),...
% 'YTick',0:0.5:7,...
% 'Position',[.1 .15 .85 .82],...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',28,...
% 'FontWeight','bold',...
% 'FontName','Times')
% 
% ylabel({'Total transmitted data /MB'},...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontWeight','bold',...
% 'FontSize',24,...
% 'FontName','Times')
% 
% 
% xlabel('Time (s)',...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontWeight','bold',...
% 'FontSize',28,...   
% 'FontName','Times')
% 
% saveas(gcf,'fig_output\trans_data_dataset','epsc');
% 
% ind_marker_dense = 1:length(Tk);
% ind_marker_sparse = 1:3:length(Tk);
% 
% figure;
% ha = tightPlots(1,1,3.3*3.9,[3.3 1.8],[0 0],[0 0],[0 0],'inch');
% set(gcf,'color','white'); box on; hold on;
% 
% plot(Tk,bytes_cur_subMOD/1e3,'--','linewidth',2,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,bytes_cur_subOPT/1e3,'-','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,bytes_cur_subMOD_multi/1e3,'--','linewidth',5,'Color', 'k','markerindices',ind_marker_dense);
% plot(Tk,bytes_cur_subOPT_multi/1e3,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% plot(Tk,bytes_saved_subOPT/1e6,'-','linewidth',2,'Color', 0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',9);
% plot(Tk,bytes_saved_subOPT_multi/1e6,'-','linewidth',5,'Color',0.8*[0.75 0.75 0.75],'markerindices',ind_marker_sparse,'markersize',14);
% 
% grid on;
% set(gca,...
% 'Units','normalized',...
% 'XTick',0:30:Tk(end),...
% 'YTick',0:0.5:4,...
% 'Position',[.12 .15 .85 .82],...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',28,...
% 'FontWeight','bold',...
% 'FontName','Times')
% 
% ylabel({'Total transmitted data','in current step /KB'},...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontWeight','bold',...
% 'FontSize',24,...
% 'FontName','Times')
% 
% 
% xlabel('Time (s)',...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontWeight','bold',...
% 'FontSize',28,...   
% 'FontName','Times')
% saveas(gcf,'fig_output\trans_data_per_step_dataset','epsc');
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
% ha = tightPlots(2,3,6.85,[5 5],[0.3 0.2],[0.3 0.05],[0.1 0.05],'inch');
% for i=1:min(length(G),6)
%     j = 130*i;
% %     subplot(3,4,i);
%     axes(ha(i));
%     plot(G{j});
% %     xlabel(['t = ',num2str(time_edge(j)),'s, # Edges = ',num2str(G{j}.numedges)]);
%     xlabel(['t = ',num2str(time_edge(j)),'s'],'fontsize',14);
% end
% saveas(gcf,'fig_output\topology_sparse','pdf');