%--------------------------------------------------------------------------
% PMVStest_PlantSens.m
% 
%--------------------------------------------------------------------------
%
%--------------------------------------------------------------------------
% Primary Contributor: Daniel R. Herber, Graduate Student, University of 
% Illinois at Urbana-Champaign
% Link: https://github.com/danielrherber/pm-suspensions
%--------------------------------------------------------------------------
close all; clear; clc; closeallbio; bdclose('all');

% % architecture 1
% Graph.L = {'s','u','b','f','p','p'};
% Graph.A = [0     0     0     0     1     0;
%      0     0     0     0     0     1;
%      0     0     0     0     1     1;
%      0     0     0     0     1     1;
%      1     0     1     1     0     0;
%      0     1     1     1     0     0];
%  
% % architecture 2
% % Graph.L = {'s','u','m','k','f','p'};
% % A = zeros(6);
% % A(1,5) = 1; % s-f
% % A(5,6) = 1; % f-p
% % A(2,6) = 1; % u-p
% % A(6,4) = 1; % p-k
% % A(3,4) = 1; % m-k
% % A = A + A';
% % Graph.A = A;
% 
% % architecture 3
% % Graph.L = {'s','u','k','k','p','p'};
% % Graph.A = [0     0     0     0     1     0;
% %      0     0     0     0     0     1;
% %      0     0     0     0     1     1;
% %      0     0     0     0     1     1;
% %      1     0     1     1     0     0;
% %      0     1     1     1     0     0];
%  
% % architecture 4
% % Graph.L = {'u','f','s'};
% % Graph.A = [0 1 0;...
% %            1 0 1;...
% %            0 1 0];
% 
% 
% %%
% 
% % 
% p.bt = 0;
% p.kt = 232.5e3;
% p.mu = 65;
% p.ms = 325;
% 
% p.comp(1).min = 1e-2; % mass min
% p.comp(1).max = 1e1; % mass max
% p.comp(2).min = 1e2; % damper min
% p.comp(2).max = 1e5; % damper max
% p.comp(3).min = 1e2; % spring min
% p.comp(3).max = 1e6; % spring max
% p.comp(4).min = nan; % force min
% p.comp(4).max = nan; % force max
% p.comp(5).min = nan; % sprung mass min
% p.comp(5).max = nan; % sprung mass max
% p.comp(6).min = nan; % unsprung mass min
% p.comp(6).max = nan; % unsprung mass max
% 
% p.rmax = 0.03;
% 
% p.w1 = 1e5;
% p.w2 = 0.5;
% p.w3 = 1e-5;
% 
% p.v = 10; % m/s vehicle velocity
% load([mfoldername(mfilename('fullpath'),''),'IRI_737b']); % road profile data
% road_t = road_x./p.v; % time vector for road profile
% z0dot = diffxy(road_t,road_z); % approximate derivative of road velocity
% p.z0dot = @(t) interp1(road_t,z0dot,t);
% p.z0 = @(t) interp1(road_t,road_z,t); 
% 
% p.t0 = road_t(1); % initial time
% p.tf = 3; % final time
% p.nt = 500;

%%
% [f,p] = PMVS_Codesign(p,Graph)


load('A:\google-drive\Projects\ID053-pm-suspensions\pm-suspensions\examples\PMVS_ex1_graphs.mat','SuspensionGraphs')
I = 3543; % canoncial active <- (a)
% % I = 3534; % best Nc=3
% % I = 3537; % reverse dynamic absorber
% % I = 3533;
% I = 3535; % <- (b) this one
Graph.A = SuspensionGraphs(I).A; Graph.L = SuspensionGraphs(I).L; 

% co-design problem specification
p = PMVS_ex1_codesign;

% set current example name
p.examplename = 'PMVS_SingleArchitecture';

% final time
p.tf = 3;

% number of DT discretization points
p.nt = 500; % 500

%%

% create simscape model
a = PMVS_CreateSimscapeModel(p,Graph);

% create plant variable information
[a,lb,ub] = PMVS_CreatePlantInfo(a);

% number of optimization variables
nxp = length(lb);

% ensure column vectors
lb = lb(:); ub = ub(:);

% scale
lb = log10(lb);
ub = log10(ub);

LB = PMVS_Scaling(lb,lb,ub,1);
UB = PMVS_Scaling(ub,lb,ub,1);

Nx = 20;
% Nx = 300;
% Nx = 60;

% generate linearly spaced vectors for each variable
for idx = 1:nxp
    x{idx,1} = linspace(LB(idx),UB(idx),Nx);
end
% x{1,1} = linspace(0.5,1,Nx);
% x{2,1} = linspace(0,0.5,Nx);

% generate the rectangular grid 
[Y{1:nxp}] = meshgrid(x{:});

% convert cell array entry to matrix for each variable
X = zeros(numel(Y{1}),nxp);
for idx = 1:nxp
    xyz = Y{idx}; % temporary variable
    X(:,idx) = xyz(:);  % save column vector
end

F = nan(numel(Y{1}),1);

spmd
    % Setup tempdir and cd into it
    currDir = pwd;
    addpath(currDir);
    tmpDir = tempname;
    mkdir(tmpDir);
    cd(tmpDir);
    % Load the model on the worker
    load_system(a.sys);
end

% str = ['pctRunOnAll load_system(',p.sys,')'];
% eval(str);

clear xyz

method = 1;
switch method
    %--------------------------------------------------------------------
    case 1
        tic
        parfor idx = 1:numel(Y{1})
            F(idx) = PMVS_Objective(X(idx,:),a,p,lb,ub)
            disp([num2str(idx),' F: ',num2str(F(idx)), ' t: ',string(datetime('now'))])
        end
    %--------------------------------------------------------------------
    case 2

        for idx = 1:numel(Y{1})
            output(idx) = parfeval(@(XX)PMVS_Objective(XX,a,p,lb,ub),1,X(idx,:)); % Square size determined by idx
        end
        % Collect the results as they become available.
        F = zeros(1,numel(Y{1}));
        tic
        for idx = 1:numel(Y{1})
            % fetchNext blocks until next results are available.
            [completedIdx,value] = fetchNext(output);
            F(completedIdx) = value;
            disp([num2str(idx),' F: ',num2str(value), ' t: ',num2str(toc)])
        end
end

% F = reshape(F,[],Nx);

figure; hold on
contourf(Y{1},Y{2},reshape(log10(F),[],Nx),30)
[Fmin,I] = min(F);
plot(Y{1}(I),Y{2}(I),'.')
disp(Fmin)

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

% return

%%
% function F = PMVS_Objective(xp,p,lb,ub)
% 
% % unscale from linear 0 to 1 mapping
% xp = PMVD_Scaling(xp,lb,ub,2); 
% 
% % unscale from log mapping
% xp = 10.^(xp);
% 
% % update matrices
% [A,B,C,D] = PMVS_UpdateMatrices(p,xp);
%     
% % find inner-loop solution
% F = PMVS_InnerLoop(p,A,B,C,D);
% 
% end
% %%
% function x_out = PMVD_Scaling(x,l,u,type)
%     x = reshape(x,[],1);
%     if type == 1
%         x_out = (x-l)./(u-l); % scale
%     elseif type == 2
%         x_out = l + x.*(u-l); % 
%     else
%         error('wrong type')
%     end
% end