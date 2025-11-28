clear all; clc; close all;

addpath('./utils')

%% Physics parameter
parameter.physics.gravitational_constant=9.81; % Gravity

% parameter.physics.sim2real_scale_factor=(13.3-11.6620+5.75)/5.75; % Real spirit
parameter.physics.sim2real_scale_factor=1; % Sim spirit or A1

parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*6.922; % Only body weight of go2
% parameter.physics.mass_body_body=parameter.physics.sim2real_scale_factor*6.0; % Only body weight of A1

parameter.physics.mass_body_leg=2.291; % Each leg weight of go2
% parameter.physics.mass_body_leg=1.935; % Each leg weight of A1

parameter.physics.mass_body=parameter.physics.mass_body_body+...
    4*parameter.physics.mass_body_leg; % Total body weight

parameter.physics.hip_offset=[0.1934; 0.0465; 0]; % Absolute hip offset from body COM of go2
% parameter.physics.hip_offset=[0.1805; 0.047; 0]; % Absolute hip offset from body COM of A1

parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
    [0.02448, 0.00012166, 0.0014849;
    0.00012166, 0.098077, -3.12E-05;
    0.0014849, -3.12E-05, 0.107]; % Body inertia of go2

% parameter.physics.inertia_body=parameter.physics.sim2real_scale_factor*...
%     [0.0158533, -3.66e-5, -6.11e-5;
%     -3.66e-5, 0.0377999, -2.75e-5;
%     -6.11e-5, -2.75e-5, 0.0456542]; % Body inertia of A1

parameter.physics.inertia_body=parameter.physics.inertia_body+...
    4*parameter.physics.mass_body_leg*...
    diag([parameter.physics.hip_offset(2)^2+parameter.physics.hip_offset(3)^2;
    parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(3)^2;
    parameter.physics.hip_offset(1)^2+parameter.physics.hip_offset(2)^2]); % Robot inertia (assume leg mass concentrated at hip)

% enable_rf = false;
% enable_l1 = true;
methods_select = 'l1' % 'nominal' ; 'rf' ; 'l1'
parameter.name = "go2"; % Model name
parameter.n = 12; % State dimension
parameter.m = 12; % Input dimension

%% Generate Dynamics Model
switch methods_select
    case 'rf'
        parameter.name = parameter.name + "_rf";
        parameter.n_rf = 50;
        parameter.size_target_mask = 6;
        parameter.n_z = 12 - 3 + 12 - 6;
        dynamicsModelRF(parameter);
    case 'l1'
        parameter.name = parameter.name + "_l1";
        parameter.size_target_mask = 6;
        dynamicsModelL1(parameter);
    otherwise
         dynamicsModel(parameter);  
end
%     if enable_rf
%         parameter.name = parameter.name + "_rf";
%         parameter.n_rf = 50;
%         parameter.size_target_mask = 6;
%         parameter.n_z = 12 - 3 + 12 - 6;
%         dynamicsModelRF(parameter);
%     else if enable_l1
%         parameter.name = parameter.name + "_l1";
%         parameter.size_target_mask = 6;
%         dynamicsModelL1(parameter);
%     else
%         dynamicsModel(parameter);  
%     end

