%--------------------------------------------------------------------------
% PMVS_CreateSimscapeModel.m
% Given a graph, create the corresponding simscape model
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function a = PMVS_CreateSimscapeModel(p,Graph)

%--------------------------------------------------------------------------
% START: Graph modifications
%--------------------------------------------------------------------------
% remove fields
if isfield(Graph,'removephi')
    Graph = rmfield(Graph,'removephi');
    Graph = rmfield(Graph,'N');
    Graph = rmfield(Graph,'Am');
    Graph = rmfield(Graph,'Ln');
end

% extract
L = Graph.L;
A = Graph.A;

% number of components
N = length(L);

% split up series components
Lt = L; % temporary copies
for idx = N:-1:1
    % if the current component has multiple components in series
    if length(Lt{idx}) > 1
        % split up the labels
        l = num2cell(Lt{idx});
        
        % number of individual components in this supercomponent
        n = length(l);
        
        % add the labels
        L = [L{1:idx-1},l,L{idx+1:end}];
      
        % find the connection indices for the original component
        I = find(A(idx,:));
        
        % expand with zeros
        A = [A(1:idx-1,:);zeros(n,length(A));A(idx+1:end,:)];
        A = [A(:,1:idx-1),zeros(length(A),n),A(:,idx+1:end)];

        % shift larger indices
        I = I + (I>idx)*(n-1);
        
        % add initial connection
        A(I(1),idx) = 1;
        A(idx,I(1)) = 1;
        
        % add final connection
        A(I(2),idx+n-1) = 1;
        A(idx+n-1,I(2)) = 1;
        
        % add internal connections
        for i = idx:idx+n-2
            A(i,i+1) = 1;
            A(i+1,i) = 1;
        end        
    end
end

% convert to uppercase, but not for the b component
L = upper(L);
L = strrep(L,'B','b');

% replace b with parallel K/B
while any(contains(L,'b'))
    % find the location of the first component
    I = find(contains(L,'b'),1);
    
    % find connections to original
    Ib = find(A(I,:));
    
    % zero the initial connections
    A(I,Ib(1)) = 0;
    A(Ib(1),I) = 0;
    A(I,Ib(2)) = 0;
    A(Ib(2),I) = 0;
    
    % add a component for the additional K
    nadd = 1; % for K
    L{end+1} = 'K'; % append to the labels
    IK = length(L); % store the location of the additional K component

    % check if connected to P (any type) already
    P1flag = 0; % initialize not connected
    if strcmp(L{Ib(1)},'P') % connected
        P1flag = 1;
    else % not connected
        nadd = nadd + 1; % for P
        L{end+1} = 'P'; % append to the labels
        IP1 = length(L); % store the location of the additional P component
    end
    P2flag = 0; % initialize not connected
    if strcmp(L{Ib(2)},'P') % connected
        P2flag = 1;
    else % not connected
        nadd = nadd + 1; % for P
        L{end+1} = 'P'; % append to the labels
        IP2 = length(L); % store the location of the additional P component
    end
    
    % add rows and columns
    A(end+1:end+nadd,:) = 0;
    A(:,end+1:end+nadd) = 0;
    
    % P1 connections
    if P1flag
        A(Ib(1),I) = 1; % B-P1
        A(I,Ib(1)) = 1; % B-P1
        A(Ib(1),IK) = 1; % K-P1
        A(IK,Ib(1)) = 1; % K-P1
    else
        A(IP1,I) = 1; % B-P1
        A(I,IP1) = 1; % B-P1
        A(IP1,IK) = 1; % K-P1
        A(IK,IP1) = 1; % K-P1
        A(IP1,Ib(1)) = 1; % IB1-P1
        A(Ib(1),IP1) = 1; % IB1-P1
    end
    
    % P2 connections
    if P2flag
        A(Ib(2),I) = 1; % B-P2
        A(I,Ib(2)) = 1; % B-P2
        A(Ib(2),IK) = 1; % K-P2
        A(IK,Ib(2)) = 1; % K-P2
    else
        A(IP2,I) = 1; % B-P2
        A(I,IP2) = 1; % B-P2
        A(IP2,IK) = 1; % K-P2
        A(IK,IP2) = 1; % K-P2
        A(IP2,Ib(2)) = 1; % IB2-P2
        A(Ib(2),IP2) = 1; % IB2-P2
    end
    
    % modify original component label now that we have completed the task
    L{I} = 'B';
end

% number of components (update)
N = length(L);

% find the locations of additional m/k/b/f components 
a.comp = p.comp;
a.comp(1).I = cellfun(@sum,strfind(L,'M'));
a.comp(2).I = cellfun(@sum,strfind(L,'B'));
a.comp(3).I = cellfun(@sum,strfind(L,'K'));
a.comp(4).I = cellfun(@sum,strfind(L,'F'));
a.comp(1).name = 'M';
a.comp(2).name = 'B';
a.comp(3).name = 'K';
a.comp(4).name = 'F';

% find indices of the sprung and unsprung masses
a.comp(5).I = cellfun(@sum,strfind(L,'S'));
a.comp(6).I = cellfun(@sum,strfind(L,'U'));
a.comp(5).name = 'S';
a.comp(6).name = 'U';
%--------------------------------------------------------------------------
% END: Graph modifications
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% START: Create simscape model file
%--------------------------------------------------------------------------

% load the foundation library (currently loaded outside this function)
% load_system('fl_lib')
% load_system('nesl_utility')
% load_system('built-in')

% create a unique name for the model
a.mainpath = mfoldername('PMVS_Main',[]);
a.modelpath = msavename(mfilename('fullpath'),'tempmodels');
timestr = datestr(datetime('now'),'yyyymmddTHHMMSSFFF');
basename = 'PMVS_Model'; % base name
a.sys = [basename,'_',timestr,'_',num2str(round(1000000*rand(1)))];

% copy base model to models folder
copyfile(fullfile(a.mainpath,'models',[basename,'_R','2018b','.slx']),...
    fullfile(a.modelpath,[a.sys,'.slx']));

% load the new model in the original workspace
load_system([a.modelpath,a.sys,'.slx']);

%--------------------------------------------------------------------------
% END: Create simscape model file
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% START: Add blocks and connections to the simscape model
%--------------------------------------------------------------------------
% add blocks from graph
nspring = 0; % spring counter
for idx = 1:N
    if strcmp(L{idx},'K')
        nspring = nspring + 1; % increment the spring counter
    end
    AddComponents(a,L{idx},idx,nspring);
end

% add road input
libName = 'fl_lib/Mechanical/Mechanical Sources/Ideal Translational Velocity Source';
blockName = [a.sys,'/z0'];
add_block(libName,blockName);

% add tire spring
libName = 'fl_lib/Mechanical/Translational Elements/Translational Spring';
blockName = [a.sys,'/kt'];
add_block(libName,blockName);
add_line(a.sys,'kt/RConn1','mu/LConn1','autorouting','on')
add_line(a.sys,'kt/LConn1','z0/LConn1','autorouting','on')

% add tire damper
libName = 'fl_lib/Mechanical/Translational Elements/Translational Damper';
blockName = [a.sys,'/bt'];
add_block(libName,blockName);
add_line(a.sys,'bt/RConn1','mu/LConn1','autorouting','on')
add_line(a.sys,'bt/LConn1','z0/LConn1','autorouting','on')

% add ground
blockName = [a.sys,'/g3'];
add_block('fl_lib/Mechanical/Translational Elements/Mechanical Translational Reference',blockName);
add_line(a.sys,'z0/RConn2','g3/LConn1','autorouting','on')

% add input 2
blockName = [a.sys,'/c5'];
add_block('nesl_utility/Simulink-PS Converter',blockName);
add_line(a.sys,'c5/RConn1','z0/RConn1','autorouting','on')
blockName = [a.sys,'/i1'];
add_block('built-in/Inport',blockName,'Port','1');
add_line(a.sys,'i1/1','c5/1','autorouting','on')

% connect the solver
add_line(a.sys,'Solver/RConn1','g3/LConn1','autorouting','on')

% set fixed parameters
set_param([a.sys,'/bt'],'D',num2str(p.bt));
set_param([a.sys,'/kt'],'spr_rate',num2str(p.kt));
set_param([a.sys,'/mu'],'mass',num2str(p.mu));
set_param([a.sys,'/ms'],'mass',num2str(p.ms));

% create a copy
Al = A;

% find all parallel components and replace with first connected component
Ip = find(cellfun(@sum,strfind(L,'P')));
record = zeros(size(L));
for idx = Ip
    I2 = find(Al(idx,:));
    leftI = I2(1);
    I2(1) = [];
    [leftBlock,record] = GetConnection(leftI,L,record);
    for k = 1:length(I2)
        try
        [rightBlock,record] = GetConnection(I2(k),L,record);
        catch
            
            disp(1)
        end
        add_line(a.sys,leftBlock,rightBlock,'autorouting','on')
    end
    Al(idx,:) = 0;
    Al(:,idx) = 0;
end

% get connection indices
[I,J] = find(tril(Al));

% add the remaining series connections
for idx = 1:length(I)
    [leftBlock,record] = GetConnection(I(idx),L,record);
    [rightBlock,record] = GetConnection(J(idx),L,record);
    add_line(a.sys,leftBlock,rightBlock,'autorouting','on')
    Al(I(idx),J(idx)) = 0;
end

% % ensure input/output numbers are correct
% for idx = 1:nspring+5
%     set_param([a.sys,'/y',num2str(idx)],'Port',num2str(idx));
% end

% save the system
save_system(a.sys)

%--------------------------------------------------------------------------
% END: Add blocks and connections to the simscape model
%--------------------------------------------------------------------------
end
% determine the connection name for the current component
function [conName,record] = GetConnection(idx,L,record)
% determine the block name associated with the current component
switch L{idx}
    case 'K'
        conName = ['k',num2str(idx)];
    case 'B'
        conName = ['b',num2str(idx)];
    case 'M'
        conName = ['m',num2str(idx)];
    case 'F'
        conName = ['f',num2str(idx)];
    case 'S'
        conName = 'ms';
    case 'U'
        conName = 'mu';
end

% determine what current requested connection is name
if record(idx) == 0
%     if strcmp(L{idx},'S')
%         conName = [conName,'/RConn1'];
%     else
        conName = [conName,'/LConn1'];
%     end
elseif record(idx) == 1
    if strcmp(L{idx},'F')
        conName = [conName,'/RConn2'];
    else
        conName = [conName,'/RConn1'];
    end
end

% increment the record for this component
record(idx) = record(idx) + 1;    
end
% add the required blocks and connections for the current component
function AddComponents(a,str,num,nspring)
% load the new model in the original workspace
load_system([a.modelpath,a.sys,'.slx']);

switch str
    %----------------------------------------------------------------------
    case 'K'
        libName = 'PMVS_Components_lib/Translational Spring';
        blockName = [a.sys,'/k',num2str(num)];
        add_block(libName,blockName);

        % outputs y6+ (spring positions)
        blockName = [a.sys,'/c',num2str(nspring+6)];
        add_block('nesl_utility/PS-Simulink Converter',blockName);
        add_line(a.sys,['k',num2str(num),'/RConn2'],...
            ['c',num2str(nspring+6),'/LConn1'],'autorouting','on')
        
        blockName = [a.sys,'/y',num2str(nspring+5)];
        add_block('built-in/Outport',blockName,'Port',num2str(nspring+5));
        add_line(a.sys,['c',num2str(nspring+6),'/1'],...
            ['y',num2str(nspring+5),'/1'],'autorouting','on')
        
    %----------------------------------------------------------------------
    case 'B'
        libName = 'fl_lib/Mechanical/Translational Elements/Translational Damper';
        blockName = [a.sys,'/b',num2str(num)];
        add_block(libName,blockName);
    %----------------------------------------------------------------------
    case 'M'
        libName = 'fl_lib/Mechanical/Translational Elements/Mass';
        blockName = [a.sys,'/m',num2str(num)];
        add_block(libName,blockName);
    %----------------------------------------------------------------------
    case 'F'
        libName = 'PMVS_Components_lib/Ideal Force Source';
        blockName = [a.sys,'/f',num2str(num)];
        add_block(libName,blockName);
        
        % inputs u1 (control force)
        blockName = [a.sys,'/c4'];
        add_block('nesl_utility/Simulink-PS Converter',blockName,'UdotUserProvided','1');
        add_line(a.sys,'c4/RConn1',['f',num2str(num),'/RConn1'],'autorouting','on')

        blockName = [a.sys,'/x1'];
        add_block('built-in/Integrator',blockName);
        add_line(a.sys,'x1/1','c4/1','autorouting','on')
        
        blockName = [a.sys,'/i2'];
        add_block('built-in/Inport',blockName,'Port','2');
        add_line(a.sys,'i2/1','x1/1','autorouting','on')

        add_line(a.sys,'i2/1','c4/2','autorouting','on')
        
        % outputs y4 and y5 (control force and position)
        blockName = [a.sys,'/y4'];
        add_block('built-in/Outport',blockName,'Port','4');
        add_line(a.sys,'x1/1','y4/1','autorouting','on')
        
        blockName = [a.sys,'/c6'];
        add_block('nesl_utility/PS-Simulink Converter',blockName);
        add_line(a.sys,['f',num2str(num),'/RConn3'],'c6/LConn1','autorouting','on')
        
        blockName = [a.sys,'/y5'];
        add_block('built-in/Outport',blockName,'Port','5');
        add_line(a.sys,'c6/1','y5/1','autorouting','on')
    %----------------------------------------------------------------------
    case 'S' % sprung mass + outputs
        libName = 'PMVS_Components_lib/Mass';
        blockName = [a.sys,'/ms'];
        add_block(libName,blockName);
        
        % outputs y2 and y3 (ms position and acceleration)
        blockName = [a.sys,'/c2'];
        add_block('nesl_utility/PS-Simulink Converter',blockName);
        add_line(a.sys,'ms/RConn1','c2/LConn1','autorouting','on')

        blockName = [a.sys,'/y2'];
        add_block('built-in/Outport',blockName,'Port','2');
        add_line(a.sys,'c2/1','y2/1','autorouting','on')
        
        blockName = [a.sys,'/c3'];
        add_block('nesl_utility/PS-Simulink Converter',blockName);
        add_line(a.sys,'ms/RConn2','c3/LConn1','autorouting','on')
        
        blockName = [a.sys,'/y3'];
        add_block('built-in/Outport',blockName,'Port','3');
        add_line(a.sys,'c3/1','y3/1','autorouting','on')
    %----------------------------------------------------------------------  
    case 'U' % unsprung mass + outputs
        libName = 'PMVS_Components_lib/Mass';
        blockName = [a.sys,'/mu'];
        add_block(libName,blockName);

        % output y1 (mu position)
        blockName = [a.sys,'/c1'];
        add_block('nesl_utility/PS-Simulink Converter',blockName);
        add_line(a.sys,'mu/RConn1','c1/LConn1','autorouting','on')
        
        blockName = [a.sys,'/y1'];
        add_block('built-in/Outport',blockName,'Port','1');
        add_line(a.sys,'c1/1','y1/1','autorouting','on')
end

end