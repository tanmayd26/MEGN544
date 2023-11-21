clear;clc;
A = [0 1; -3 -0.5]; %fill in
B = [0;1]; %fill in
open_loop_poles = eig(A);

Ar = A;
% add model error if desired
% Ar(end,:) = Ar(end,:)+2*(rand(size(Ar(end,:))-.5)).*Ar(end,:)*.1;

%%
% Ts = 1.5 sec; Tr = .75 sec;
Tr = 0.1;
Ts = 0.5;
os = 0.05; 

zeta = sqrt((log(os)^2)/(pi^2+log(os)^2));%
wn = max(4.6/(Ts*0.95),(1.53+2.31*zeta^2)/(Tr*0.95));
sigma = zeta*wn;% sigma = zeta*wn

poles = -wn*(zeta+[1;-1]*sqrt(1-zeta^2)*1j);
K = place(A,B,poles) ;%continuous time K

% Bss = 0; % Steady State Control to reduce steady state error
Bss = (B'*B)^-1*B'*A;

% %%  Bessel Filter Placement
Tp = 0.35; % desired peak time
[~,poles_b,~] = besself(size(A,1),  2*pi/Tp);
K = place(A,B,poles_b);


%% Discrete Time Design 
dt = 0.01;
Ad = exp(A*dt);
Bd = inv(A)*(Ad-eye(size(A)))*B;
ns = size(A,1);
nc=size(B,2);
tmp = [A B; zeros(nc,ns) zeros(nc,nc)];
tmpd = expm(tmp*dt);
Ad = tmpd(1:ns,1:ns);
Bd = tmpd(1:ns,ns+(1:nc));

Pd = exp(poles*dt);

Kd = place(Ad,Bd,Pd);

%% Bessel Filter Placement
Pd = exp(poles_b*dt);
Kd = place(Ad,Bd,Pd);

%% Design Observer
C = [1 0];
obs_poles = poles;
obs_poles_disc = exp(obs_poles*dt);

L = place(Ad',Ad'*C',obs_poles_disc)';



%%  Bessel Filter Placement
freq_cuttoff = 10;% in hz
[~,poles_b_obs,~] = besself(size(A,1),2*pi*freq_cuttoff );

obs_poles_descrete = exp(poles_b_obs*dt);
L = place(Ad',Ad'*C',obs_poles_descrete)';

