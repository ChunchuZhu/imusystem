load('impedancewalk.mat');
load('impedancefollow.mat');


%% Walk
Freq = impedancewalk(1908:2960,1);
M_L = impedancewalk(1908:2960,2);
T_L = impedancewalk(1908:2960,3);
M_R = impedancewalk(1908:2960,4);
T_R = impedancewalk(1908:2960,5);

Ave_Freq = mean(Freq);
% plot(M_L)
% hold on
% plot(-M_R)
% plot(M_L)
% hold on
% plot(T_L)
% 
% figure
% plot(-M_R)
% hold on
% plot(T_R*2.2)
% lg = legend('Knee Angle','Torque');
% set(lg,'fontsize',20)

%% Follow
