%--------------------------------------------------------------------------
% PMVStest_Allison2014b.m
% Solve the co-design problem for the suspension in the following reference
% Allison, et al. 'Co-Design of an Active Suspension Using Simultaneous 
% Dynamic Optimization.' ASME Journal of Mechanical Design, 136(8),
% Aug 2014, DOI: 10.1115/1.4027335
%--------------------------------------------------------------------------
%
%--------------------------------------------------------------------------
% Primary Contributor: Daniel R. Herber, Graduate Student, University of 
% Illinois at Urbana-Champaign
% Link: https://github.com/danielrherber/pm-suspensions
%--------------------------------------------------------------------------
function PMVStest_Allison2014b
% close and clear
close all; clc; closeallbio; bdclose('all');

% co-design problem specification
p = PMVS_ex1_codesign;

% set current example name
p.examplename = 'PMVS_Allison2014b';

% final time
p.tf = 3;

% number of DT discretization points
p.nt = 5000;

% 
lb(1) = p.comp(2).min;
lb(2) = p.comp(3).min;
ub(1) = p.comp(2).max;
ub(2) = p.comp(3).max;

% solve the co-design problem
[a,F] = PMVS_OuterLoop2(p,lb,ub);

end

function [p,F] = PMVS_OuterLoop2(p,lb,ub,varargin)
% number of optimization variables
nxp = length(lb);

% ensure column vectors
lb = lb(:); ub = ub(:);

% scale
lb = log10(lb);
ub = log10(ub);

LB = PMVS_Scaling(lb,lb,ub,1);
UB = PMVS_Scaling(ub,lb,ub,1);

% initialize plant design
X = LB + (UB-LB).*rand(nxp,1);

% patternsearch options
options = psoptimset('Display','Iter','MaxIter',10,'Cache','on',...
    'UseParallel',true,'CompletePoll','on','CompleteSearch','on',...
    'SearchMethod', {@searchlhs,1,nxp*30});

% global search method
X = patternsearch(@(x) PMVS_Objective2(x,p,lb,ub),X,[],[],[],[],LB,UB,[],options);

% fmincon optimization
options = optimoptions('fmincon','Display','Iter','Algorithm','sqp',...
    'UseParallel',true,'MaxIter',20,...
    'FiniteDifferenceStepSize',100*sqrt(eps),'OptimalityTolerance',1e-4);

% local search method
[X,F] = fmincon(@(x) PMVS_Objective2(x,p,lb,ub),X,[],[],[],[],LB,UB,[],options);

% unscale from linear 0 to 1 mapping
X = PMVS_Scaling(X,lb,ub,2); 

% unscale from log mapping
X = 10.^(X);

% add unscaled plant variables
p.xpopt = X;

end
% 
function F = PMVS_Objective2(xp,p,lb,ub)

% unscale from linear 0 to 1 mapping
xp = PMVS_Scaling(xp,lb,ub,2); 

% unscale from log mapping
xp = 10.^(xp);

% update matrices
bs = xp(1); ks = xp(2);
A = [0,1,0,0;
    -p.kt/p.mu,-(bs+p.bt)/p.mu,ks/p.mu,bs/p.mu;
    0,-1,0,1;
    0,bs/p.ms,-ks/p.ms,-bs/p.ms];
Bu = [0;-1/p.mu;0;1/p.ms];
Bz = [-1;p.bt/p.mu;0;0];
    
% find inner-loop solution
F = PMVS_InnerLoop2(p,A,Bu,Bz);

end
%
function F = PMVS_InnerLoop2(p,A,Bu,Bz)
% number of states
ns = 4;

% 
C1 = [1,0,0,0];
D1u = Bu(1,:);
% D1z = Bz(1,:);

C2 = [0,0,1,0];

C3 = A(4,:);
D3u = Bu(4,:);
% D3z = Bz(4,:);

w1 = p.w1;
w2 = p.w2;
w3 = p.w3;

% disturbance
d = cell(ns,1);
for idx = 1:ns
    d{idx,1} = @(t,p) Bz(idx)*p.z0dot(t)';
end

% Lagrange term
L(1).left = 2; % states
L(1).right = 2; % states
L(1).matrix = w1*(C1'*C1) + w2*(C3'*C3);

L(2).left = 1; % controls
L(2).right = 1; % controls
L(2).matrix = w1*(D1u'*D1u) + w2*(D3u'*D3u) + w3;

L(3).left = 1; % controls
L(3).right = 2; % states
L(3).matrix = 2*w1*(D1u'*C1) + 2*w2*(D3u'*C3);

% initial value constraints
LB(1).right = 4; % initial states
LB(1).matrix = zeros(ns,1);

UB(1).right = 4; % initial states
UB(1).matrix = zeros(ns,1);

% rattlespace constraints
Z(1).linear(1).right = 2; % states
Z(1).linear(1).matrix = C2';
Z(1).b = p.rmax;

Z(2).linear(1).right = 2; % states
Z(2).linear(1).matrix = -C2';
Z(2).b = p.rmax;

% combine
setup.A = A; setup.B = Bu; setup.d = d;
setup.L = L; setup.UB = UB; setup.LB = LB; setup.Z = Z;

% DT QP options
opts.dt.defects = 'HS';
opts.dt.quadrature = 'CQHS';
opts.dt.mesh = 'ED';
opts.dt.nt = 1000;
opts.general.displevel = 1;
opts.solver.tolerance = 1e-16;

setup.t0 = 0;
setup.tf = p.tf;
setup.auxdata = p; 

% form and solve problem
[T,U,Y,~,F,~,~] = DTQP_solve(setup,opts);

% % disp(F)
% % % disp(eig(A))
% % 
% z0 = p.z0(T);
% for idx = 1:length(T)
%     zu(idx) = C1*Y(idx,:)';
%     zs(idx) = C2*Y(idx,:)';
%     zsdd(idx) = C3*Y(idx,:)' + D3u*U(idx,:)';
% end
% % 
% % close all
% 
% figure
% plot(T,z0); hold on
% plot(T,zu); hold on
% plot(T,zs); hold on
% legend('z0','zu','zs')
% 
% figure
% plot(T,zs-zu); hold on
% 
% figure
% plot(T,U); hold on
% 
% figure
% plot(T,Y); hold on
% 
% figure
% plot(T,zsdd); hold on
end
% co-design problem specification
function p = PMVS_ex1_codesign
% fixed suspension parameters
p.bt = 0; % N/(m/s)
p.kt = 232.5e3; % N/m
p.mu = 65; % kg
p.ms = 325; % kg

% rattlespace and stroke constraint
p.rmax = 0.04; % m
p.smax = 0.04; % m

% objective function weights
p.w1 = 1e5;
p.w2 = 0.5;
p.w3 = 1e-5;

% plant variable upper and lower bounds
p.comp(1,1).min = 1e-2; % mass min
p.comp(1,1).max = 1e1; % mass max
p.comp(2,1).min = 1e2; % damper min
p.comp(2,1).max = 1e5; % damper max
p.comp(3,1).min = 1e2; % spring min
p.comp(3,1).max = 1e6; % spring max
p.comp(4,1).min = nan; % force min
p.comp(4,1).max = nan; % force max
p.comp(5,1).min = nan; % sprung mass min
p.comp(5,1).max = nan; % sprung mass max
p.comp(6,1).min = nan; % unsprung mass min
p.comp(6,1).max = nan; % unsprung mass max

% road profile
p.v = 10; % m/s vehicle velocity
load('IRI_737b','road_x','road_z'); % road profile data
road_t = road_x./p.v; % time vector for road profile
z0dot = diffxy(road_t,road_z); % approximate derivative of road velocity
p.z0dot = @(t) interp1(road_t,z0dot,t);
p.z0 = @(t) interp1(road_t,road_z,t); 
p.t0 = road_t(1); % initial time

end

% NOTE: need to add force stroke constraint