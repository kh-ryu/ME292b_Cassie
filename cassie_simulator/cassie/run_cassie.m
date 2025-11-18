function run_cassie()
close all ;

% add paths
startup;

% Load Cassie model and set Initial configuration
model = load('cassie_model.mat') ;

% Initial configuration
[x0, model] = getInitialState(model.model);


% Get STUDENT Control Parameters
params = studentParams(model);

% ODE options
time_inter = [0 5] ;
odeopts = odeset('Events', @falldetect);
externalForce_fun = @ExternalForce ;

%% Simulation 
disp('Simulating...') ;
tic

[t_vec, x_vec] = ode15s( @cassie_eom, time_inter, x0, odeopts, model, params, externalForce_fun) ;

toc
disp(['Simulated for ' num2str(t_vec(end)), 's'])

%% Calculate Score
score = calcScore(t_vec', x_vec', model);
disp(['Score: ', num2str(score)])

%%
r_com = zeros(length(t_vec), 3) ;
for i = 1 : size(x_vec,1)
    r_com(i,:) = compute_COM_pos(model, x_vec(i,1:model.NB))' ;
end

%% Calculate control signals
disp('Computing control signals...') ;
xdot_vec = zeros(size(x_vec)) ;
tau_vec = zeros(length(t_vec), 20) ;
for j=1:length(t_vec)
    [xdot_,tau_] = cassie_eom(t_vec(j),x_vec(j,:)', model,params,externalForce_fun) ;
    xdot_vec(j,:) = xdot_' ;
    tau_vec(j,:) = tau_' ;
end

%% Plots and animation

disp('Graphing...') ;
% Plot COM position, base orientation, joint angles
figure() ; 
    subplot(3,1,1);plot(t_vec, r_com) ;grid ; title('com positions x-y-z') ;hold; legend('x','y','z') ;
    subplot(3,1,2); plot(t_vec, x_vec(:,4:6)) ; grid ; title('base angles') ; 
    subplot(3,1,3); plot(t_vec, x_vec(:,7:model.n)) ; grid ; title('joint angles') ; 
    
% Plot Base (Pelvis) Position
figure ; plot(t_vec, x_vec(:,1:3)) ; grid on ;
    title('Base (Pelvis) Translation') ; legend('x','y','z') ;
    
% Plot Base (Pelvis) Orientation
figure ; plot(t_vec, x_vec(:,4:6)*180/pi) ; grid on ;
    title('Base (Pelvis) Orientation') ; legend('r','p','y') ;
    
% Plot Torques
figure ; 
    subplot(2,1,1) ;
        plot(t_vec, tau_vec(:, [model.jidx.hip_abduction_left; model.jidx.hip_rotation_left; model.jidx.hip_flexion_left; model.jidx.knee_joint_left; model.jidx.toe_joint_left])) ;
        grid on ; title('Left Torques') ; legend('abduction','rotation','flexion','knee','toe') ;
    subplot(2,1,2) ;
        plot(t_vec, tau_vec(:, [model.jidx.hip_abduction_right; model.jidx.hip_rotation_right; model.jidx.hip_flexion_right; model.jidx.knee_joint_right; model.jidx.toe_joint_right])) ;
        grid on ; title('Right Torques') ; legend('abduction','rotation','flexion','knee','toe') ;
        
%% Animation
stateData = getVisualizerState(x_vec, model);
vis = CassieVisualizer(t_vec, stateData);