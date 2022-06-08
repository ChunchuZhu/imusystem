%% Walking Impedance

Angle_E = AaronYoungImpedance(1,1);
Angle_L = AaronYoungImpedance(:,1);
Torque_L = AaronYoungImpedance(:,2);
Error = Angle_L - Angle_E;
plot(Error(1000:4000));
hold on;
plot(Torque_L(1000:4000));
legend('Angle Error','Torque/2.2(Current)');


%% Impedance_Tracking
Angle_Desired = ImpedanceTracking(1:3000,1);
Angle_Real = ImpedanceTracking(1:3000,2);
Torque = ImpedanceTracking(1:3000,3);
figure
plot(Angle_Desired);
hold on;
plot(Angle_Real);
plot(Torque*20);
legend('Desired Angle','Real Angle','Torque/2.2(Current)*50times');

%% PID_Tracking
Angle_Desired = PIDTracking(10:3000,1);
Angle_Real = PIDTracking(10:3000,2);
Torque = PIDTracking(10:3000,3);
figure
plot(Angle_Desired);
hold on;
plot(Angle_Real);
plot(Torque*20);
legend('Desired Angle','Real Angle','Torque/2.2(Current)*50times');
