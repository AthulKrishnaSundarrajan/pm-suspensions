%--------------------------------------------------------------------------
% PMVS_Main.m
% Enumerate and evaluate all vehicle suspension architectures given
% a problem instance
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function [F,A,SuspensionGraphs] = PMVS_Main(C,R,P,NSC,opts,p)

% get the current path of the test file
mypath = mfoldername(p.examplename,[]);

% generate suspension architectures as graphs
if exist([p.examplename,'_graphs.mat'],'file')
    load([p.examplename,'_graphs'],'SuspensionGraphs');
else
    SuspensionGraphs = PMA_UniqueFeasibleGraphs(C,R,P,NSC,opts);
end

% number of suspension architectures
N = length(SuspensionGraphs);

% change pX to p in the graphs
for idx = 1:N
    Ip = contains(SuspensionGraphs(idx).L,'p','IgnoreCase',true); % determine p locations
    SuspensionGraphs(idx).L(Ip) = {'p'};
    SuspensionGraphs(idx).L = SuspensionGraphs(idx).L(:)'; % row vector
end

% indices to evaluate
if isfield(p,'Ieval')
    if p.Ieval < 0
        I = randi([1 N],-p.Ieval,1); % evaluate random
    else
        I = p.Ieval;
    end
else
    I = 1:N; % evaluate all
    % I = 1:50; % evaluate only some
end

% load the foundation library
load_system('fl_lib')
load_system('nesl_utility')
load_system('built-in')
load_system('PMVS_Components_lib')

% initialize
N = length(I); F = nan(N,1); % A = cell(N,1);

% turn off all warnings on the workers
if p.parallel ~= 0 
    PMA_ParallelToggle(p,'start')
    pctRunOnAll warning off;
end

%--------------------------------------------------------------------------
% determine the performance of each architecture
%--------------------------------------------------------------------------
% start the timer
tic

if p.parallel ==  0 
    % for loop version
    for idx = 1:N
        Ic = I(idx);
        
        % evaluate
        [F(Ic),A(Ic)] = PMVS_Codesign(p,SuspensionGraphs(Ic));
        
        % display to command window 
        dispfun(idx,F(Ic),A(Ic).exitflag);
    end
else
    % get the current parallel pool
    currentPool = gcp();

    % request a solution for each architecture
    for idx = 1:N
        parJobs(idx) = parfeval(currentPool,@(x) PMVS_Codesign(p,x),2,SuspensionGraphs(I(idx))); %#ok<AGROW>
    end

    % collect the results as they become available
    counter = 0;
    for idx = 1:N
        % fetchNext blocks until next results are available.
        [completedIdx,B1,B2] = fetchNext(parJobs);

        % outputs
        Ic = I(completedIdx);
        F(Ic) = B1;
        A(Ic) = B2;

        % display to command window 
        counter = dispfun(counter,F(Ic),B2.exitflag);
    end
end

%--------------------------------------------------------------------------
% save the results
%--------------------------------------------------------------------------
save([mypath,p.examplename,'_results.mat'],'F','A','I','SuspensionGraphs',...
    'C','R','P','NSC','opts','p');

end
% display to command window
function counter = dispfun(counter,f,exitflag)
    t = toc;
    counter = counter + 1;
    disp([num2str(counter),' F:',num2str(f),' exit:',num2str(exitflag),' t:',num2str(t),' s'])
end