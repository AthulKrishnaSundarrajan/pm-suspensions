%--------------------------------------------------------------------------
% PMVS_InnerLoop.m
% Formulate and solve the inner-loop control problem
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [F,varargout] = PMVS_InnerLoop(p,A,B,C,D)

% number of function outputs
nout = max(nargout,1);

% number of controls
nu = size(B,2)-1;

if nu > 0
    % find the location of the control variable in A
    Iu = find(C(4,:));

    % remove control derivative row from (A,B,C,D)
    A(Iu,:) = [];
    B(Iu,:) = [];
    C(4,:) = [];
    D(4,:) = [];

    % move matrices for the control variable to (A,C)
    B(:,2) = A(:,Iu); 
    D(:,2) = C(:,Iu); 

    % remove control derivative column from (A,B,C,D)
    A(:,Iu) = [];
    C(:,Iu) = [];
end

% separate the road input and control input
if nu > 0
    Bz = B(:,1);
    Bu = B(:,2);
    D1z = D(1,1);
    D1u = D(1,2);
    D2z = D(2,1);
    D2u = D(2,2);
    D3z = D(3,1);
    D3u = D(3,2);
else % no control
    Bz = B(:,1);
    Bu = [];
    D1z = D(1,1);
    D1u = [];
    D2z = D(2,1);
    D2u = [];
    D3z = D(3,1);
    D3u = [];
end

% separate output matrices
C1 = C(1,:);
C2 = C(2,:);
C3 = C(3,:);
Cs = C(4:end,:); % stroke constraints, now output 4 since we remove one output

% update output 3 since it is defined as a force, not acceleration
% C3 = C3/p.ms;
% D3z = D3z/p.ms;
% D3u = D3u/p.ms;

% number of states
ns = length(A);

% disturbance
d = cell(ns,1);
for idx = 1:ns
    d{idx,1} = @(t,p) Bz(idx)*p.z0dot(t)';
end

% extract objective function weights
w1 = p.w1;
w2 = p.w2;
w3 = p.w3;

% Lagrange term
L(1).left = 2; % states
L(1).right = 2; % states
L(1).matrix = w1*(C1'*C1) + w2*(C3'*C3);

L(2).left = 0; % singleton
L(2).right = 2; % states
for idx = 1:length(C1)
    L(2).matrix{1,idx} = @(t,p) -2*w1*C1(idx)*p.z0(t);
end

L(3).left = 0; % singleton
L(3).right = 0; % singleton
L(3).matrix{1} = @(t,p) (w1*p.z0(t).^2)';

if nu > 0
    L(4).left = 1; % controls
    L(4).right = 1; % controls
    L(4).matrix = w1*(D1u'*D1u) + w2*(D3u'*D3u) + w3;

    L(5).left = 1; % controls
    L(5).right = 2; % states
    L(5).matrix = 2*w1*(D1u'*C1) + 2*w2*(D3u'*C3);

    L(6).left = 0; % singleton
    L(6).right = 1; % controls
    for idx = 1:length(D1u)
        L(6).matrix{1,idx} = @(t,p) -2*w1*D1u(idx)*p.z0(t);
    end
end

% initial value constraints
LB(1).right = 4; % initial states
LB(1).matrix = zeros(ns,1);

UB(1).right = 4; % initial states
UB(1).matrix = zeros(ns,1);

% rattlespace constraints
Z(1).linear(1).right = 2; % states
Z(1).linear(1).matrix = (C2-C1)';
Z(1).b = p.rmax;

Z(2).linear(1).right = 2; % states
Z(2).linear(1).matrix = -(C2-C1)';
Z(2).b = p.rmax;

% stroke constraints
zidx = 2;
for idx = 1:size(Cs,1)
    zidx = zidx + 1;
    Z(zidx).linear(1).right = 2; % states
    Z(zidx).linear(1).matrix = Cs(idx,:)';
    Z(zidx).b = p.smax;

    zidx = zidx + 1;
    Z(zidx).linear(1).right = 2; % states
    Z(zidx).linear(1).matrix = -Cs(idx,:)';
    Z(zidx).b = p.smax;
end

% combine
setup.A = A; setup.B = Bu; setup.d = d;
setup.L = L; setup.UB = UB; setup.LB = LB; setup.Z = Z;
setup.t0 = 0; setup.tf = p.tf; setup.p = p;

% DT QP options
opts = [];
opts.dt.defects = 'HS'; % Hermite-Simpson 
opts.dt.quadrature = 'CQHS'; % composite quadratic Hermite-Simpson
opts.dt.mesh = 'ED'; % equidistant nodes
% opts.dt.nt = 1000; % normally set outside of this function
opts.dt.nt = p.nt; % normally set outside of this function
opts.general.displevel = 0; % silent

% form and solve problem
[T,U,Y,~,F,~,~] = DTQP_solve(setup,opts);

% control plotting internally
plotflag = 0;

% compute the positions
if nout > 1 || plotflag
    z0 = p.z0(T);
    for idx = 1:length(T)
        zu(idx,1) = C1*Y(idx,:)';
        zs(idx,1) = C2*Y(idx,:)';
        zsdd(idx,1) = C3*Y(idx,:)' + D3u*U(idx,:)';
        for k = 1:size(Cs,1)
            zf(idx,k) = Cs(k,:)*Y(idx,:)';
        end
    end
end

% store the outputs
if nout > 1 % evaluate a single plant design and get the trajectories
    O.T = T;
    O.U = U;
    O.Y = Y; 
    O.F = F; 
    O.z.u = zu; 
    O.z.s = zs; 
    O.z.sdd = zsdd; 
    O.z.f = zf; 
    O.matrix.A = A;
    O.matrix.B = B;
    O.matrix.C = C;
    O.matrix.D = D;
    varargout{1} = O;
end

% optional plotting
if plotflag
    % display objective function value
    disp(F)
    
    % display eigenvalues of A matrix
    disp(vpa(eig(A)))

    % create some plots
    close all

    figure
    plot(T,z0); hold on
    plot(T,zu); hold on
    plot(T,zs); hold on
    legend('z0','zu','zs')

    figure
    plot(T,zs-zu); hold on
    ylabel('z_s-z_u')

    figure
    plot(T,U); hold on
    ylabel('U')

    figure
    plot(T,Y); hold on
    ylabel('Y')

    figure
    plot(T,zsdd); hold on
    ylabel('zsdd')
    
    figure
    plot(T,zf); hold on
    ylabel('zf')
end

end