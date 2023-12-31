clear; clc; close all;

%% Set Simulation Parameters
enable_control = true;
drawCSM = false;  % Draws CSM on the plot, requires a points3D.mat file to exist
Velocity_Mode = false; % Enables theta_dot only based control
Sim_Exact = false; % sets if the simulated geometry is exact (true) or slightly different (false)
simTime = 52; %Run the simulation for the specified number of seconds
makeMovie = false;
uses_geometry = true;  %if true it makes a link-list for use in your functions

%% Define Theta space Trajectory
% [ time_end, th1->th6;...]
trajectory = [0, [1.52696476906926,-2.03010727357431,0.731598816110416,-3.09608676102519,1.84281602691039,1.58303056857332];
    2,[1.52696476906926, -2.03010727357431, 0.731598816110416, -3.09608676102519, 1.84281602691039, 1.58303056857332];
    4,[1.39712512845332,	-2.03064590837639,	0.721395529389311,	-2.96193577054707,	1.82831776432121,	2.40241609071958];
    6,[1.35491547778574,	-2.07309668322019,	0.740488280419015,	-2.91964101727917,	1.80335229011481,	2.72990725660181]; %random angle set between +/- pi/3
    8,[1.31347261182381,	-2.15896539588566,	0.780012096991369,	-2.87967506646971,	1.75624810912514,	-3.09220218463655]; %random angle set between +/- pi/3
    10,[1.31347261182381,	-2.24702978436925,	0.822685770077810,	-2.88160765421716,	1.71239357560481,	-2.64042275844857];
    12, [1.35491547778574,	-2.33856136077730,	0.868864192624755,	-2.92463818149378,	1.66954103162287,	-2.33446758373059];
    14, [1.39712512845332,	-2.38600061975137,	0.892934131465172,	-2.96740603607754,	1.64735456982248,	-1.55733834721666];
    16, [1.52696476906926,	-2.38823663615200,	0.904373912328849,	-3.09759515920750,	1.65764622380978,	-1.56697750502571];
    18,[1.61462788452053,	-2.38823663615200,	0.904373912328849,	3.09759515920750,	1.65764622380978,	-1.57461514856408];
    20,[1.74446752513647,	-2.38600061975137,	0.892934131465172,	2.96740603607754,	1.64735456982248,	-0.798856142975687];
    22, [1.78667717580405,	-2.33856136077730,	0.868864192624755,	2.92463818149378,	1.66954103162287,	-0.0217269064617597];
    24, [1.78667717580405,	-2.24784629351069,	0.830641345467981,	2.92322070984912,	1.72079524586413,	1.07399920113651];
    26, [1.70162372286895,	-2.20397679528500,	0.821243913565419,	3.00844453870740,	1.75723366923667,	1.08232670547993];
    28, [1.61462788452053,	-2.15975718809925,	0.804761519930512,	3.09672167356449,	1.78638644623109,	-0.00960512812039774];
    30, [1.61462788452053,	-2.07267719352257,	0.757018103877522,	3.09629671468234,	1.82568306654537,	-0.796826211216253];
    32, [1.65829166759729,	-2.03022774281197,	0.729553606472357,	3.05082293375059,	1.83985949282290,	-1.59498726564477];
    34, [1.78667717580405,	-2.03089467334971,	0.715300424931752,	2.91871916765578,	1.81994737508708,	-2.83864874357433];
    36, [1.90825239602268,	-2.37830316237937,	0.856990790450868,	2.80375374661787,	1.61748729503543,	-0.0163954458853738];
    38, [1.90825239602268,	-2.28882221908444,	0.821964666240235,	2.80244241574567,	1.66885352065396,	-0.0345235900713673];
    40, [1.90825239602268,	-2.15782143117547,	0.760481823303652,	2.79938506467456,	1.73437915502919,	-0.0579474214442261];
    42, [1.90825239602268,	-2.03137661869714,	0.689109994629797,	2.79582168077608,	1.78622735939067,	-2.57494853662416];
    44, [2.02031060130244,	-2.19638519820301,	0.742545994317316,	2.68938982628872,	1.67608918610401,	-0.694510979883813];
    46, [2.12147187770715,	-2.02714580731234,	0.602796155247180,	2.58610665622224,	1.69547110144480,	3.06456268025029];
    48, [2.12147187770715,	-2.14763232817683,	0.671177231823620,	2.58892779475006,	1.65115847537618,	3.09212079692580];
    50, [2.12147187770715,	-2.27078810261237,	0.729553606472357,	2.59072221438101,	1.59598706337729,	3.12612076748596]]; %random angle set between +/- pi/3

%% Load ABB Arm Geometry
if uses_geometry
    assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
    % assert(exist('newtonEuler.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('constAccelInterp.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('createLink.m','file')==2,'Simulation Error:  Need to add project files to path');
    run('Geometry.m'); 
end

if drawCSM 
    assert(exist('points3D.mat','file')==2,'Simulation Error: Need to make sure a file points3D.mat that has the transformed 2D points in it is on the path');
end 

if Velocity_Mode
    assert(exist('velocityJacobian.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('transError.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('cpMap.m','file')==2,'Simulation Error:  Need to add project files to path');
    assert(exist('dhFwdKine.m','file')==2,'Simulation Error:  Need to add project files to path');
end

%% Controller Parameter Definitions
Sim_Name = 'System_Theta_Space';
%run Geometry.m; % creates a linkList in the workspace for you and for the simulation
if drawCSM
    load points3D; % loads the CSM trajectory points
    sizeCSM = size(points3D,1);
else
    sizeCSM = 0;
end
open([Sim_Name '.slx']);



%% Define Kp and Kd gains
% If yoru going to tweak, make sure you save these initial values.
Kp = [500;500;500;300;10;10];
Kd = [75;75;75;40;1;1];
Ki = [0;500;300;300;0;0];


%% Get num points for simulink
traj_length = size(trajectory,1);

%% Choose Simulation System (perfect model or realistic model)
set_param([Sim_Name '/Theta Controller/ABB Arm Dynamics/sim_exact'], 'sw', int2str(Sim_Exact))

%% Enable/Disable Velocity Mode (Ignores Desired Theta Values)
set_param([Sim_Name '/Theta Controller/Velocity_Mode'], 'sw', int2str(~Velocity_Mode))


%% Enable/Disable Control
set_param([Sim_Name '/Theta Controller/control_enable'], 'sw', int2str(enable_control))


%% Run Simulation
simOut =  sim(Sim_Name,'SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');

%% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');

%% Plot theta as a function of time
figure(1)
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end

%% Display Arm Motion Movie

if makeMovie
    obj = VideoWriter('csm_arm_motion','MPEG-4');
    obj.FrameRate = 30;
    obj.Quality = 50;
    obj.open();
end

figure(2)
plot(0,0); ah = gca; % just need a current axis handel
fh = gcf;
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
    plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
    hold on;
    plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
        reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
        reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
    if drawCSM
        plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
    end
    hold off;
    if makeMovie
        obj.writeVideo(getframe(fh));
    end
    pause(1/30)
end
if makeMovie
    obj.close();
end

