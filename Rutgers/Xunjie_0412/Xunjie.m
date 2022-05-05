% Xunjie_0412_data_slip = Xunjie0412SmallSlip(1:2:end,:);
% Marker Order:
% Left foot: Heel, Toe, Side, Top
% Right foot: Heel, Toe, Side, Top
load('Xunjie_0412_data_slip.mat');
% IMU Order:
% Trunk, Right Thigh, Left Thigh, Right Shank, Left Shank, Right Heel, 
% Left Heel
IMU.RK = Xunjie_0412_data_slip(:,64:72);
IMU.LK = Xunjie_0412_data_slip(:,73:81);
IMU.RK_Z_Zeroed = mean(IMU.RK(1:100,7));
IMU.LK_Z_Zeroed = mean(IMU.LK(1:100,7));
IMU.RH = Xunjie_0412_data_slip(:,82:90);
IMU.LH = Xunjie_0412_data_slip(:,91:99);
IMU.RH_Z_Zeroed = mean(IMU.RH(1:100,7));
IMU.LH_Z_Zeroed = mean(IMU.LH(1:100,7));

l = 0.2;
L_hh = 1;
%% Slip detection data analysis
% Xunjie_0412_data_slip = Xunjie_0412_data_slip(8140:end,:);
l_heel = Xunjie_0412_data_slip(:,10:12);
l_toe = Xunjie_0412_data_slip(:,19:21);

r_heel = Xunjie_0412_data_slip(:,25:27);
r_toe = Xunjie_0412_data_slip(:,34:36);


freq = Xunjie_0412_data_slip(:,2);
r_gaitStage = Xunjie_0412_data_slip(:,3); 
l_gaitStage = Xunjie_0412_data_slip(:,4);
slipRight = Xunjie_0412_data_slip(:,5);
slipLeft = Xunjie_0412_data_slip(:,6);


IMU.TK = Xunjie_0412_data_slip(:,37:45);
IMU.RT = Xunjie_0412_data_slip(:,46:54);
IMU.LT = Xunjie_0412_data_slip(:,55:63);
IMU.RK = Xunjie_0412_data_slip(:,64:72);
IMU.LK = Xunjie_0412_data_slip(:,73:81);
IMU.RH = Xunjie_0412_data_slip(:,82:90);
IMU.LH = Xunjie_0412_data_slip(:,91:99);

plot(IMU.TK(:,9));hold on;plot(IMU.TK(:,8));plot(IMU.TK(:,7));legend('Euler X','Euler Y','Euler Z');

%%
pelvisAcc = IMU.TK(:,5);
forwardFootAcc_L = IMU.LH(:,4).*cosd(IMU.LH(:,7)-IMU.LH_Z_Zeroed) + IMU.LH(:,5).*sind(IMU.LH(:,7)-IMU.LH_Z_Zeroed);
forwardFootAcc_R = IMU.RH(:,4).*cosd(IMU.RH(:,7)-IMU.RH_Z_Zeroed) + IMU.RH(:,5).*sind(IMU.RH(:,7)-IMU.RH_Z_Zeroed);
a_a_l = 0;
a_a_r = 0;
for i =2:length(IMU.RK)
    a_a_l = (IMU.LK(i,3) - IMU.LK(i-1,3))/0.01;
    a_a_r = (IMU.RK(i,3) - IMU.RK(i-1,3))/0.01;
    ep_L(i) = atan( (IMU.LK(i,4) + a_a_l*l) / (IMU.LK(i,5) + IMU.LK(i,3)^2*l) ) - (IMU.LK(i,7)-IMU.LK_Z_Zeroed);
    ep_R(i) = atan( (IMU.RK(i,4) + a_a_r*l) / (IMU.LK(i,5) + IMU.RK(i,3)^2*l) ) - (IMU.RK(i,7)-IMU.RK_Z_Zeroed);
end 


%% Plot Vicon
for i = 1:50:10000
    line([l_heel(i,1) l_toe(i,1)],[l_heel(i,3) l_toe(i,3)],'Color','red','LineStyle','--');
    hold on
%     plot(r_heel(i,1),r_heel(i,3),'b');
%     line([l_heel(i,1) l_toe(i,1)],[l_heel(i,3) l_toe(i,3)],'Color','red','LineStyle','--');
end
%% Touch Down detection
% Touch down indicator
% -1 for touch down / 1 for walking
td_L(1) = -1;
td_R(1) = -1;
ep_L(1) = 0;
ep_R(1) = 0;
timeStmap_L = 0;
timeStmap_R = 0;
for i =2:length(IMU.RK)
    ep_L(i) = atan( (IMU.LK(i,4) + a_a_l*l) / (IMU.LK(i,5) + IMU.LK(i,3)^2*l) ) - (IMU.LK(i,7)-IMU.LK_Z_Zeroed);
    ep_R(i) = atan( (IMU.RK(i,4) + a_a_r*l) / (IMU.LK(i,5) + IMU.RK(i,3)^2*l) ) - (IMU.RK(i,7)-IMU.RK_Z_Zeroed);
%     if ep_L(i) - ep_L(i-1) > 200
%         td_L(i) = -td_L(i-1);
%     else
%         td_L(i) = td_L(i-1);
%     end
%     if ep_R(i) - ep_R(i-1) > 200
%         td_R(i) = -td_R(i-1);
%     else
%         td_R(i) = td_R(i-1);
%     end
    if ep_L(i) > 50
        td_L(i) = 300;      
    else
        td_L(i) = -1;
    end
    if ep_R(i)  > 50
        td_R(i) = 300;      
    else
        td_R(i) = -1;
    end
    
end

%%
for i=1:length(ep_R)
    if(ep_R(i)<80 & ep_R(i)>20)
        foot_R(i) = forwardFootAcc_R(i);
        foot_L(i) = 0;
    elseif(ep_L(i)<30 & ep_L(i)>-30)
        foot_L(i) = forwardFootAcc_L(i);
        foot_R(i) = 0;
    else
        foot_L(i) = 0;
        foot_R(i) = 0;
    end
end


%%  Slip indicator
% original
dd_q_hh_l = (pelvisAcc - forwardFootAcc_L) / L_hh;
dd_q_hh_r = (pelvisAcc - forwardFootAcc_R) / L_hh;
slip_indicator_l = forwardFootAcc_L ./ (2.718.^ (dd_q_hh_l - 40)) / 10^17;
slip_indicator_r = forwardFootAcc_R ./ (2.718.^ (dd_q_hh_r - 40)) / 10^17;
% 
% dd_q_hh_l = (pelvisAcc - foot_L') / L_hh;
% dd_q_hh_r = (pelvisAcc - foot_R') / L_hh;
% slip_indicator_l = foot_L' ./ (2.718.^ (dd_q_hh_l - 40)) / 10^17;
% slip_indicator_r = foot_R' ./ (2.718.^ (dd_q_hh_r - 40)) / 10^17;

plot(ep_L)
hold on
plot(ep_R)


line([0 1.7*10^4],[80 80],'Color','red','LineStyle','--')
line([0 1.7*10^4],[-30 -30],'Color','red','LineStyle','--')
legend("ep_L","ep_R")
hold on


% plot(forwardFootAcc_L);hold on
% plot(forwardFootAcc_R)
% plot(slip_indicator_l,'g')
% plot(slip_indicator_r,'k')

%   Serial.print(TKAVx);
%   Serial.print(",");
%   Serial.print(TKAVy);
%   Serial.print(",");
%   Serial.print(TKAVz);
%   Serial.print(",");
%   Serial.print(TKLAx);
%   Serial.print(",");
%   Serial.print(TKLAy);
%   Serial.print(",");
%   Serial.print(TKLAz);
%   Serial.print(",");
%   Serial.print(TKEulerz);
%   Serial.print(",");
%   Serial.print(TKEulery);
%   Serial.print(",");
%   Serial.print(TKEulerx);