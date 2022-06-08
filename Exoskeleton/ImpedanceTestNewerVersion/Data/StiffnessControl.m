close all
clear
clc

%% Stiffness Control Data
addpath Data

load("stiffness1.mat");
load("stiffness1_5.mat");
load("Stiffness2.mat");
load("stiffness3.mat");

data = Stiffness1;

theta_k_l = data(:,1);
theta_k_r = data(:,2);
torque_command_l = data(:,3);
torque_command_r = data(:,4);
current_l = data(:,5);
current_r = data(:,6);

figure(1)
plot(theta_k_l)
hold on
plot(torque_command_l)
plot(current_l)
lg = legend('Knee Angle','Torques','Motor Current');
set(lg,'Fontsize',20);
