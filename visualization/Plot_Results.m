%%%%%%%%%%created by Qi Yan%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised August 2018%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot results
%%
clear
load CL_data.mat;
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
ha = tightPlots(1,1,6.85,[5 3],[0.5 0.3],[0.6 0.3],[0.5 0.3],'inch');
box on; hold on; set(ha,'color','white');
plot(Tk,sum(RMSE_DR,2),'linewidth',1);
plot(Tk,sum(RMSE_CEKF_Dense,2),'linewidth',1);
plot(Tk,sum(RMSE_CEKF_subMOD,2),'linewidth',1);
plot(Tk,sum(RMSE_CEKF_subMOD_multi,2),'linewidth',1);
plot(Tk,sum(RMSE_CEKF_subOPT,2),'linewidth',1);
plot(Tk,sum(RMSE_CEKF_subOPT_multi,2),'linewidth',1);
xlabel('Time /s');
ylabel('Aggregated RMSE /m');
legend('DR','Dense','subMOD','subMOD-multi','subOPT','subOPT-multi','location','northwest');
set(gca,'fontsize',14);
% saveas(gcf,'fig_output\rmse_sim','epsc');
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

% h_lgd = legend('Dense-CL','OPT-CL','SAEKF-CL','SubOPT-CL','DR','fontsize',10,'location','best');
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
ha = tightPlots(1,1,6.85*.9,[5 4],[0.4 0.8],[0.6 0.1],[0.6 0.3],'inch');
% subplot(2,1,1);
% axes(ha(1));
% box on;
% hold on;
% % plot(Tk,Tr_DR,'linewidth',1,'color','black');
% % plot(Tk,Tr_CEKF_Dense,'linewidth',1);
% % plot(Tk,Tr_CEKF_Random,'linewidth',1);
% % % plot(Tk,Tr_CEKF_SubMOD,'linewidth',1);
% % plot(Tk,Tr_SAEKF_SubMOD,'linewidth',1);
% 
% plot(Tk,Logdet_CEKF_Dense,'linewidth',1);
% plot(Tk,Logdet_CEKF_Random,'linewidth',1);
% plot(Tk,Logdet_SAEKF_SubMOD,'linewidth',1);
% plot(Tk,Logdet_CEKF_subOPT,'linewidth',1);
% plot(Tk,Logdet_DR,'linewidth',1);
% [V_1,~] = TrajGen(1);
% % bound_CEKF = tr_bound(P_CEKF{ceil(mea_start/delta)+1},V_1(1),delta);
% % line([mea_start mea_end],[bound_CEKF bound_CEKF],'linewidth',2,'linestyle','--');
% % line([mea_start mea_end],[trace(Pi_bar_SubMOD) trace(Pi_bar_SubMOD)],'linewidth',2,'linestyle','--','color','blue');
% % line([mea_start mea_end],[trace(Pi_bar_init) trace(Pi_bar_init)],'linewidth',2,'linestyle','-.','color','red');
% % plot(Tr_ICI,'linewidth',1);
% % plot(Tr_ICI_EKF,'linewidth',1);
% % xlim([max(Tk)*49/60 max(Tk)*55/60]);
% % ylim([0 trace(Pi_bar_SubMOD)*1.2]);
% % ylabel('Trace of covariance/m^2');
% ylabel('logdet of covariance');
% % ylim([0 0.02]);
% legend('Dense-CL','Random-CL','OPT-CL','SubOPT-CL','DR','location','best');

% subplot(2,1,2);
% axes(ha(2));
box on;
hold on;
plot(Tk,Logdet_CEKF_Dense,'linewidth',1);
% plot(Tk,Logdet_CEKF_Random,'linewidth',1);
plot(Tk,Logdet_CEKF_subMOD,'linewidth',1);
plot(Tk,Logdet_CEKF_subMOD_multi,'linewidth',1,'linestyle','--');
% plot(Tk,Logdet_SAEKF_SubMOD,'linewidth',1);
plot(Tk,Logdet_CEKF_subOPT,'linewidth',1);
plot(Tk,Logdet_CEKF_subOPT_multi,'linewidth',1,'linestyle','--');
% legend('Dense-CL','OPT-CL','SAEKF-CL','SubOPT-CL','location','best');
legend('Dense-CL','SubMOD-CL','subMOD-multi','SubOPT-CL','subOPT-multi','location','best','fontsize',14);
% ylabel('Trace of covariance/m^2');
ylabel('Logdet of covariance','fontsize',14);
xlabel('Time /s','fontsize',14);
% saveas(gcf,'fig_output\logdet_sim','epsc');
%% Computational timing
figure; hold on; box on;
timing_subOPT = timing_subOPT(timing_subOPT>0);
timing_subMOD = timing_subMOD(timing_subMOD>0);
bar(categorical({'subOPT','subModular'}),[mean(timing_subOPT),mean(timing_subMOD)]);
ylabel('Running time per robot /ms');
set(gca,'fontsize',14);
title(['q^i = ',num2str(q_i)]);
% legend('SubOPT','SubModular');

%% Communication times evolution
figure; hold on; box on; grid on; set(gcf,'color','white');
plot(Tk,comm_CEKF_subMOD,'linewidth',1);
plot(Tk,comm_CEKF_subOPT,'linewidth',1);
legend('subMOD','subOPT','location','best');
ylabel('Number of total communications');
xlabel('Time/s');
set(gca,'fontsize',14);
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