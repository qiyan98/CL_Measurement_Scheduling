%%%%%%%%%%modified by Qi Yan%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Last Revised August 2018%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%Trajectory Plot%%%%%%%%%%%%%%%%%%%
%%
figure(f1);
if Missed_on_off==1
plot(XX(1,:,1),XX(2,:,1),'b')
hold on
plot(XX(1,:,2),XX(2,:,2),'r')
plot(XX(1,:,3),XX(2,:,3),'g')
plot(XX(1,:,4),XX(2,:,4),'k')
plot(XX(1,:,5),XX(2,:,5),'c')
legend('Robot 1','Robot 2','Robot 3','Robot 4', 'Robot 5')
xlabel('x')
ylabel('y')

plot(XX_hat(1,:,1),XX_hat(2,:,1),'b--')
plot(XX_hat(1,:,2),XX_hat(2,:,2),'r--')
plot(XX_hat(1,:,3),XX_hat(2,:,3),'g--')
plot(XX_hat(1,:,4),XX_hat(2,:,4),'k--')
plot(XX_hat(1,:,5),XX_hat(2,:,5),'c--')

plot(XX_hat(1,1,1),XX_hat(2,1,1),'bx','MarkerSize',12)
plot(XX_hat(1,1,2),XX_hat(2,1,2),'rx','MarkerSize',12)
plot(XX_hat(1,1,3),XX_hat(2,1,3),'gx','MarkerSize',12)
plot(XX_hat(1,1,4),XX_hat(2,1,4),'kx','MarkerSize',12)
plot(XX_hat(1,1,5),XX_hat(2,1,5),'cx','MarkerSize',12)
else
    plot(XX(1,:,1),XX(2,:,1),'b','LineWidth',2)
hold on
plot(XX(1,:,2),XX(2,:,2),'r','LineWidth',2)
plot(XX(1,:,3),XX(2,:,3),'g','LineWidth',2)
plot(XX(1,:,4),XX(2,:,4),'k','LineWidth',2)
plot(XX(1,:,5),XX(2,:,5),'c','LineWidth',2)


plot(XX_hat(1,:,1),XX_hat(2,:,1),'b--','LineWidth',2)
plot(XX_hat(1,:,2),XX_hat(2,:,2),'r--','LineWidth',2)
plot(XX_hat(1,:,3),XX_hat(2,:,3),'g--','LineWidth',2)
plot(XX_hat(1,:,4),XX_hat(2,:,4),'k--','LineWidth',2)
plot(XX_hat(1,:,5),XX_hat(2,:,5),'c--','LineWidth',2)

plot(XX_hat(1,1,1),XX_hat(2,1,1),'bx','MarkerSize',12)
plot(XX_hat(1,1,2),XX_hat(2,1,2),'rx','MarkerSize',12)
plot(XX_hat(1,1,3),XX_hat(2,1,3),'gx','MarkerSize',12)
plot(XX_hat(1,1,4),XX_hat(2,1,4),'kx','MarkerSize',12)
plot(XX_hat(1,1,5),XX_hat(2,1,5),'cx','MarkerSize',12)
end
%%
figure(f2);
if Missed_on_off==1
plot(XX(1,:,1),XX(2,:,1),'b')
hold on
legend('Robot 1')
xlabel('x')
ylabel('y')
plot(XX_hat(1,:,1),XX_hat(2,:,1),'b--')
plot(XX_hat(1,1,1),XX_hat(2,1,1),'bx','MarkerSize',12)
else
plot(XX(1,:,1),XX(2,:,1),'b','LineWidth',2)
legend('Robot 1')
hold on
plot(XX_hat(1,:,1),XX_hat(2,:,1),'b--','LineWidth',2)
plot(XX_hat(1,1,1),XX_hat(2,1,1),'bx','MarkerSize',12)
end
%%
figure(f3);
if Missed_on_off==1
plot(XX(1,:,2),XX(2,:,2),'r')
hold on
legend('Robot 2')
xlabel('x')
ylabel('y')
plot(XX_hat(1,:,2),XX_hat(2,:,2),'r--')
plot(XX_hat(1,1,2),XX_hat(2,1,2),'rx','MarkerSize',12)
else
plot(XX(1,:,2),XX(2,:,2),'r','LineWidth',2)
legend('Robot 2')
hold on
plot(XX_hat(1,:,2),XX_hat(2,:,2),'r--','LineWidth',2)
plot(XX_hat(1,1,2),XX_hat(2,1,2),'rx','MarkerSize',12)
end
%%

% figure(f4);
% if Missed_on_off==1
% plot(XX(1,:,3),XX(2,:,3),'g')
% hold on
% legend('Robot 3')
% xlabel('x')
% ylabel('y')
% plot(XX_hat(1,:,3),XX_hat(2,:,3),'g--')
% plot(XX_hat(1,1,3),XX_hat(2,1,3),'gx','MarkerSize',12)
% else
% plot(XX(1,:,3),XX(2,:,3),'g','LineWidth',2)
% legend('Robot 3')
% hold on
% plot(XX_hat(1,:,3),XX_hat(2,:,3),'g--','LineWidth',2)
% plot(XX_hat(1,1,3),XX_hat(2,1,3),'gx','MarkerSize',12)
% end



figure(f4);
plot(Tk,abs(XX(1,:,1)-XX_hat(1,:,1)),color_plot,'LineWidth',1)
hold on

figure(f5);
plot(Tk,abs(XX(1,:,2)-XX_hat(1,:,2)),color_plot,'LineWidth',1)
hold on


figure(f6);
plot(Tk,abs(XX(1,:,3)-XX_hat(1,:,3)),color_plot,'LineWidth',1)
hold on
%close(figure(f5))
%close(figure(f6))

%%
% figure(f5);
% if Missed_on_off==1
% plot(XX(1,:,4),XX(2,:,4),'k')
% hold on
% legend('Robot 4')
% xlabel('x')
% ylabel('y')
% plot(XX_hat(1,:,4),XX_hat(2,:,4),'k--')
% plot(XX_hat(1,1,4),XX_hat(2,1,4),'kx','MarkerSize',12)
% else
%     plot(XX(1,:,4),XX(2,:,4),'k','LineWidth',2)
%     legend('Robot 4')
% hold on
% plot(XX_hat(1,:,4),XX_hat(2,:,4),'k--','LineWidth',2)
% plot(XX_hat(1,1,4),XX_hat(2,1,4),'kx','MarkerSize',12)
% end
% %%
% figure(f6);
% if Missed_on_off==1
% plot(XX(1,:,5),XX(2,:,5),'c')
% hold on
% legend('Robot 5')
% xlabel('x')
% ylabel('y')
% plot(XX_hat(1,:,5),XX_hat(2,:,5),'c--')
% plot(XX_hat(1,1,5),XX_hat(2,1,5),'cx','MarkerSize',12)
% else
%     plot(XX(1,:,5),XX(2,:,5),'c','LineWidth',2)
%     legend('Robot 5')
% hold on
% plot(XX_hat(1,:,5),XX_hat(2,:,5),'c--','LineWidth',2)
% plot(XX_hat(1,1,5),XX_hat(2,1,5),'cx','MarkerSize',12)
% end

%%
figure(f7)

plot(Tk,XX(1,:,1)-XX_hat(1,:,1),color_plot,'LineWidth',1)
hold on
plot(Tk,+Three_sigma_robotx(:,1)',[color_plot,'--'],'LineWidth',1)
plot(Tk,-Three_sigma_robotx(:,1)',[color_plot,'--'],'LineWidth',1)
title('robot 1')

if Missed_on_off==0
    line([floor(10),floor(10)],[-30,30])
    line([floor(90),floor(90)],[-30,30])
    line([floor(110),floor(110)],[-30,30])
    line([floor(190),floor(190)],[-30,30])
    line([floor(240),floor(240)],[-30,30])
    axis([0 250 -10 10])
else
    axis([0 250 -30 30])
end
hold on


set(gca,...
    'Units','normalized',...
    'YTick',-30:10:30,...
    'Position',[.15 .25 .82 .6],...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',14,...
    'FontName','Times')

%ylabel('x^1')
ylabel({'x^1 error'},...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',16,...
    'FontName','Times')
%,...
%'Position',[0.1 0.1])
xlabel('t',...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',16,...
    'FontName','Times')%,...
%'Position',[10 -3.45])

% title('Robot 1','FontUnits','points',...
%     'FontWeight','normal',...
%     'FontSize',16,...
%     'FontName','Times');

grid off
% print -depsc2 /Users/Solmaz/Desktop/robot1_opti.eps

%%
figure(f8)

plot(Tk,XX(1,:,2)-XX_hat(1,:,2),color_plot,'LineWidth',1)
hold on
plot(Tk,+Three_sigma_robotx(:,2)',[color_plot,'--'],'LineWidth',1)
plot(Tk,-Three_sigma_robotx(:,2)',[color_plot,'--'],'LineWidth',1)
hold on
if Missed_on_off==0
    line([floor(10),floor(10)],[-30,30])
    line([floor(90),floor(90)],[-30,30])
    line([floor(110),floor(110)],[-30,30])
    line([floor(190),floor(190)],[-30,30])
    line([floor(240),floor(240)],[-30,30])
    axis([0 250 -10 10])
else
    axis([0 250 -30 30])
end

set(gca,...
    'Units','normalized',...
    'YTick',-30:10:30,...
    'Position',[.15 .25 .82 .6],...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',14,...
    'FontName','Times')

%ylabel('x^1')
ylabel({'x^2 error'},...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',16,...
    'FontName','Times')
%,...
%'Position',[0.1 0.1])
xlabel('t',...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',16,...
    'FontName','Times')%,...
%'Position',[10 -3.45])

% title('Robot 2','FontUnits','points',...
%     'FontWeight','normal',...
%     'FontSize',16,...
%     'FontName','Times');

grid off
% print -depsc2 /Users/Solmaz/Desktop/robot2_opti.eps

%%
figure(f9)

plot(Tk,XX(1,:,3)-XX_hat(1,:,3),color_plot,'LineWidth',1)
hold on
plot(Tk,+Three_sigma_robotx(:,3)',[color_plot,'--'],'LineWidth',1)
plot(Tk,-Three_sigma_robotx(:,3)',[color_plot,'--'],'LineWidth',1)
hold on
if Missed_on_off==0
    line([floor(10),floor(10)],[-30,30])
    line([floor(90),floor(90)],[-30,30])
    line([floor(110),floor(110)],[-30,30])
    line([floor(190),floor(190)],[-30,30])
    line([floor(240),floor(240)],[-30,30])
    axis([0 250 -10 10])
else
    axis([0 250 -30 30])
end

set(gca,...
    'Units','normalized',...
    'YTick',-30:10:30,...
    'Position',[.15 .25 .82 .6],...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',14,...
    'FontName','Times')

%ylabel('x^1')
ylabel({'x^3 error'},...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',16,...
    'FontName','Times')
%,...
%'Position',[0.1 0.1])
xlabel('t',...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',16,...
    'FontName','Times')%,...
%'Position',[10 -3.45])

% title('Robot 3','FontUnits','points',...
%     'FontWeight','normal',...
%     'FontSize',16,...
%     'FontName','Times');

grid off
% print -depsc2 /Users/Solmaz/Desktop/robot3_opti.eps

%%
%close(f10)

%%
%close(f11)

% 
% figure(f13)
%    plot(Tk,XX(3,:,2)*180/pi,'b','LineWidth',2)
%     hold on
%     plot(Tk,XX_hat(3,:,2)*180/pi,'k','LineWidth',2)
%     
%     
%     figure 
%     plot(Tk,Three_sigma_robotx(:,3))