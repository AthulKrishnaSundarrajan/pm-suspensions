%--------------------------------------------------------------------------
% PMVStest_SingleArchitecture.m
% 
%--------------------------------------------------------------------------
%
%--------------------------------------------------------------------------
% Primary Contributor: Daniel R. Herber, Graduate Student, University of 
% Illinois at Urbana-Champaign
% Link: https://github.com/danielrherber/pm-suspensions
%--------------------------------------------------------------------------
function PMVStest_SingleArchitecture
% close and clear
close all; clc; closeallbio; bdclose('all');

% architecture specification
L = {'s','u','f','b','p','p'};
A = [     0     0     0     0     1     0;
     0     0     0     0     0     1;
     0     0     0     0     1     1;
     0     0     0     0     1     1;
     1     0     1     1     0     0;
     0     1     1     1     0     0];
Graph.A = A; Graph.L = L;

% L = {'s','u','f'};
% A = [0     0     1;
%      0     0     1;
%      1     1     0];
% Graph.A = A; Graph.L = L;

% L = {'s','u','f','b'};
% A = [0     0     1     0;
%      0     0     0     1;
%      1     0     0     1;
%      0     1     1     0];
% Graph.A = A; Graph.L = L;

% L = {'s','u','m','k','k','b','f','p','p'};
% A = [0     0     0     0     0     0     0     0     1;
%      0     0     0     1     0     0     0     0     0;
%      0     0     0     0     0     1     0     0     0;
%      0     1     0     0     0     0     0     0     1;
%      0     0     0     0     0     0     0     1     1;
%      0     0     1     0     0     0     0     1     0;
%      0     0     0     0     0     0     0     1     1;
%      0     0     0     0     1     1     1     0     0;
%      1     0     0     1     1     0     1     0     0];
% Graph.A = A; Graph.L = L;

% L = {'s','u','m','k','b','f','p','p'};
% L = {'s','u','m','k','k','f','p','p'};
% L = {'u','s','m','k','k','f','p','p'};
% A = [0     0     0     0     0     0     0     1;
%      0     0     0     1     0     0     0     0;
%      0     0     0     0     0     0     1     0;
%      0     1     0     0     0     0     0     1;
%      0     0     0     0     0     0     1     1;
%      0     0     0     0     0     0     1     1;
%      0     0     1     0     1     1     0     0;
%      1     0     0     1     1     1     0     0];
% Graph.A = A; Graph.L = L;

% L = {'u','s','m','k','f','p'};
% A = zeros(length(L));
% A(6,1) = 1; % p-s
% A(4,2) = 1; % k-u
% A(5,3) = 1; % f-m
% A(6,4) = 1; % p-k
% A(6,5) = 1; % p-f
% A = A + A';
% Graph.A = A; Graph.L = L;

% load('A:\google-drive\Projects\ID053-pm-suspensions\pm-suspensions\examples\PMVS_ex1_graphs.mat','SuspensionGraphs')
% I = 635;
% Graph.A = SuspensionGraphs(I).A; Graph.L = SuspensionGraphs(I).L; 

% co-design problem specification
p = PMVS_ex1_codesign;

% set current example name
p.examplename = 'PMVS_SingleArchitecture';

% final time
p.tf = 3;

% number of DT discretization points
p.nt = 500; % 500

% solve the co-design problem
[~,a] = PMVS_Codesign(p,Graph);

% get the outputs
[F,a,O] = PMVS_Codesign(p,Graph,a.xpopt);

% visualize graph
Grph = graph(Graph.A); plot(Grph,'NodeLabel',Graph.L)
keyboard
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

% Grph = graph(Graph.A); plot(Grph,'NodeLabel',Graph.L)
