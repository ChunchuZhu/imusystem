% Zhu_0414_data_slip = data20220414163308(1:2:end,:);
% Marker Order:
% Left foot: Heel, Toe, Side, Top
% Right foot: Heel, Toe, Side, Top
load('Chunchu_0416.mat');
% IMU Order:
% Trunk, Right Thigh, Left Thigh, Right Shank, Left Shank, Right Heel, 
% Left Heel
IMU.TK = Zhu_0414_data_slip(:,37:45);
IMU.RT = Zhu_0414_data_slip(:,46:54);
IMU.LT = Zhu_0414_data_slip(:,55:63);
IMU.RK = Zhu_0414_data_slip(:,64:72);
IMU.LK = Zhu_0414_data_slip(:,73:81);
IMU.RH = Zhu_0414_data_slip(:,82:90);
IMU.LH = Zhu_0414_data_slip(:,91:99);

IMU.RK_Z_Zeroed = mean(IMU.RK(1:100,7));
IMU.LK_Z_Zeroed = mean(IMU.LK(1:100,7));
IMU.RT_Z_Zeroed = mean(IMU.RT(1:100,7));
IMU.LT_Z_Zeroed = mean(IMU.LT(1:100,7));
IMU.RH_Z_Zeroed = mean(IMU.RH(1:100,7));
IMU.LH_Z_Zeroed = mean(IMU.LH(1:100,7));

IMU.RK_X_Zeroed = mean(IMU.RK(1:100,9));
IMU.LK_X_Zeroed = mean(IMU.LK(1:100,9));
IMU.RT_X_Zeroed = mean(IMU.RT(1:100,9));
IMU.LT_X_Zeroed = mean(IMU.LT(1:100,9));
IMU.RH_X_Zeroed = mean(IMU.RH(1:100,9));
IMU.LH_X_Zeroed = mean(IMU.LH(1:100,9));

IMU.RK_Y_Zeroed = mean(IMU.RK(1:100,8));
IMU.LK_Y_Zeroed = mean(IMU.LK(1:100,8));
IMU.RT_Y_Zeroed = mean(IMU.RT(1:100,8));
IMU.LT_Y_Zeroed = mean(IMU.LT(1:100,8));
IMU.RH_Y_Zeroed = mean(IMU.RH(1:100,8));
IMU.LH_Y_Zeroed = mean(IMU.LH(1:100,8));

l = 0.2;
L_hh = 1;
%% Slip detection data analysis
% Chunchu's Data
% Zhu_0414_data_slip = Zhu_0414_data_slip(8140:end,:);
l_heel = Zhu_0414_data_slip(:,7:9);
l_toe = Zhu_0414_data_slip(:,10:12);
l_top = Zhu_0414_data_slip(:,16:18);
r_heel = Zhu_0414_data_slip(:,19:21);
r_toe = Zhu_0414_data_slip(:,22:24);
r_top = Zhu_0414_data_slip(:,28:30);

freq = Zhu_0414_data_slip(:,2);
r_gaitStage = Zhu_0414_data_slip(:,3); 
l_gaitStage = Zhu_0414_data_slip(:,4);
slipRight = Zhu_0414_data_slip(:,5);
slipLeft = Zhu_0414_data_slip(:,6);

plot(IMU.TK(:,9));hold on;plot(IMU.TK(:,8));plot(IMU.TK(:,7));legend('Euler X','Euler Y','Euler Z');
% figure; plot(l_heel(:,3));hold on; plot(l_heel(:,1));
%%
% pelvisAcc = IMU.TK(:,5);
% forwardFootAcc_L = IMU.LH(:,4).*cosd(IMU.LT(:,7) - IMU.LT_Z_Zeroed + IMU.LH(:,7)-IMU.LH_Z_Zeroed) - IMU.LH(:,5).*sind( IMU.LT(:,7) - IMU.LT_Z_Zeroed + IMU.LH(:,7)-IMU.LH_Z_Zeroed);
% forwardFootAcc_R = IMU.RH(:,4).*cosd(IMU.RT(:,7) - IMU.RT_Z_Zeroed + IMU.RH(:,7)-IMU.RH_Z_Zeroed) - IMU.RH(:,5).*sind( IMU.RT(:,7) - IMU.RT_Z_Zeroed + IMU.RH(:,7)-IMU.RH_Z_Zeroed);
% a_a_l = 0;
% a_a_r = 0;
% for i =2:length(IMU.RK)
%     a_a_l = (IMU.LK(i,3) - IMU.LK(i-1,3))/0.01;
%     a_a_r = (IMU.RK(i,3) - IMU.RK(i-1,3))/0.01;
%     ep_L(i) = atan( (IMU.LK(i,4) + a_a_l*l) / (IMU.LK(i,5) + IMU.LK(i,3)^2*l) ) - (IMU.LK(i,7)-IMU.LK_Z_Zeroed);
%     ep_R(i) = atan( (IMU.RK(i,4) + a_a_r*l) / (IMU.LK(i,5) + IMU.RK(i,3)^2*l) ) - (IMU.RK(i,7)-IMU.RK_Z_Zeroed);
% end 
% 
% for i=1:length(ep_R)
%     if(ep_R(i)<80 & ep_R(i)>20)
%         foot_R(i) = forwardFootAcc_R(i);
%         foot_L(i) = 0;
%     elseif(ep_L(i)<30 & ep_L(i)>-30)
%         foot_L(i) = forwardFootAcc_L(i);
%         foot_R(i) = 0;
%     else
%         foot_L(i) = 0;
%         foot_R(i) = 0;
%     end
% end

%% Heel strike detection
pelvisAcc = IMU.TK(:,5); % acceleration in y direction pelvis

theta_p_l = IMU.LH(:,8)-IMU.LH_Y_Zeroed;
theta_p_r = IMU.RH(:,8)-IMU.RH_Y_Zeroed;
forwardFootAcc_L = abs(IMU.LH(:,4).*cosd(theta_p_l)) - abs(IMU.LH(:,5).*sind(theta_p_l));
verticalFootAcc_L = IMU.LH(:,4).*cosd(theta_p_l) + IMU.LH(:,5).*sind(theta_p_l);
forwardFootAcc_R = abs(IMU.RH(:,4).*cosd(theta_p_r)) - abs(IMU.RH(:,5).*sind(theta_p_r));
verticalFootAcc_R = IMU.RH(:,4).*cosd(theta_p_r) + IMU.RH(:,5).*sind(theta_p_r);

a_a_l = 0;
a_a_r = 0;
ep_L(1)=0;
ep_R(1)=0;
for i =2:length(IMU.RK)
    a_a_l = (IMU.LH(i,2) - IMU.LH(i-1,2))/0.01;
    a_a_r = (IMU.RH(i,2) - IMU.RH(i-1,2))/0.01;
    ep_L(i) = atan( (IMU.LH(i,5) + a_a_l*l) ./ (IMU.LK(i,4) + IMU.LH(i,3)^2*l) ) - theta_p_l(i);
    ep_R(i) = atan( (IMU.RH(i,4) + a_a_r*l) ./ (IMU.LK(i,5) + IMU.RH(i,3)^2*l) ) - theta_p_r(i);
end 
figure;
plot(ep_L)
hold on
plot(ep_R)
hold on;plot((l_heel(:,3)-20))
hold on;plot((r_heel(:,3)-20));legend('ep_l','ep_r','heel_l','heel_r')

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
plot(slip_indicator_l)
hold on;
plot(slip_indicator_r)

for i = 1:length(slip_indicator_r)
    if (r_gaitStage(i)~=0)
        slip_indicator_r(i) = 0;
    end
end



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



%% Analyze data from 10900 - 11200
% pelvisAcc = IMU.TK(:,5);
% forwardFootAcc_L = abs(IMU.LH(:,4).*cosd(IMU.LH(:,9)-IMU.LH_X_Zeroed)) - IMU.LH(:,5).*sind(IMU.LH(:,8)-IMU.LH_Y_Zeroed);
% forwardFootAcc_R = abs(IMU.RH(:,4).*cosd(IMU.LH(:,9)-IMU.LH_X_Zeroed)) - IMU.RH(:,5).*sind(IMU.RH(:,8)-IMU.RH_Y_Zeroed);
% % a_a_l = 0;
% % a_a_r = 0;
% for i =10900:11200
%     a_a_l(i) = (IMU.LH(i,3) - IMU.LH(i-1,3))/0.01;
%     a_a_r(i) = (IMU.RH(i,3) - IMU.RH(i-1,3))/0.01;
%     ep_L(i) = atan( (IMU.LH(i,5) + a_a_l*l) / (IMU.LH(i,4) + IMU.LH(i,3)^2*l) ) - (IMU.LH(i,7)-IMU.LH_Z_Zeroed);
%     ep_R(i) = atan( (IMU.RH(i,4) + a_a_r*l) / (IMU.LH(i,5) + IMU.RH(i,3)^2*l) ) - (IMU.RH(i,7)-IMU.RH_Z_Zeroed);
% end 
% 
% figure
% plot(l_heel(10900:11200,3));
% hold on
% plot(ep_L(10900:11200));
% plot(l_gaitStage(10900:11200))

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