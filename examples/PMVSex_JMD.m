function [F,P,SuspensionGraphs] = PMVSex_JMD
% close and clear
close all; clc; closeallbio; try bdclose('all'); catch, end

% architecture specification
[C,R,P,NSC,opts] = PMVS_ex1_architecture;

% co-design problem specification
p = PMVS_ex1_codesign;

% set current example name
p.examplename = 'PMVSex_JMD';

% indices to evaluate (optional)
% p.Ieval = 1:N; % evaluate all
% p.Ieval = 1:50; % evaluate only some
% p.Ieval = -10; % evaluate random

% number of 
% p.parallel = 0; % no parallel computing
p.parallel = 12; % 12 threads

% final time
p.tf = 3;

% number of DT discretization points
p.nt = 500;

% generate the graphs and solve the co-design problems
[F,P,SuspensionGraphs] = PMVS_Main(C,R,P,NSC,opts,p);

end
% architecture specification
function [C,R,P,NSC,opts] = PMVS_ex1_architecture
% problem specification
P = [1 1 1 2 2 2 3 4]; % ports vector 
R.min = [1 1 0 0 0 1 0 0]; % replicate vector 
R.max = [1 1 2 2 2 1 2 2]; % replicate vector 
% R.max = [1 1 1 2 1 1 2 2]; % replicate vector 
C = {'s','u','m', 'k', 'b', 'f', 'p3', 'p4'}; % label vector

% constraints
NSC.M = [1 1 0 0 0 0 0 0]; % mandatory components
NSC.counts = 1;

% potential adjacency matrix
A = ones(length(P)); % initialize
A(2,1) = 0; % u-s
A(3,1) = 0; % m-s
A(3,2) = 0; % m-u
A(4,4) = 0; % k-k
A(5,5) = 0; % b-b
A(7,7) = 0; % p-p
A(8,7) = 0; % p-p
A(8,8) = 0; % p-p
NSC.A = A;

% line-connectivity constraints
NSC.Bind(1,:) = [1,7,2]; % s-p-u
NSC.Bind(2,:) = [2,7,1]; % u-p-s
NSC.Bind(3,:) = [1,8,2]; % s-p-u
NSC.Bind(4,:) = [2,8,1]; % u-p-s
NSC.Bind(5,:) = [3,8,3]; % m-p-m
NSC.Bind(6,:) = [3,7,3]; % m-p-m

% options
opts.algorithm = 'tree_v8_mex';
opts.Nmax = 1e7; % maximum number of graphs to preallocate for
opts.parallel = 12; % 1 to enable parallel computing, 0 to disable it
opts.filterflag = 1; % 1 is on, 0 is off
opts.customfun = @(pp,A,feasibleFlag) PMVSex_JMDConstraints(pp,A,feasibleFlag);
opts.isomethod = 'python'; % option 'Matlab' is available in 2016b or later versions

opts.plots.plotfun = 'matlab'; % 'circle' % 'bgl' % 'bio' % 'matlab'
opts.plots.plotmax = 0; % maximum number of graphs to display/save
opts.plots.name = mfilename; % name of the example
% opts.plots.path = mfoldername(mfilename('fullpath'),[opts.plots.name,'_figs']); % path to save figures to
opts.path = [];
opts.plots.labelnumflag = 0; % add replicate numbers when plotting

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