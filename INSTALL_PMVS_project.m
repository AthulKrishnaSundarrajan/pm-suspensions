%--------------------------------------------------------------------------
% INSTALL_PMVS_project
% Project link: https://github.com/danielrherber/pm-suspensions
% This scripts helps you get the PM Suspensions Project up and running
%--------------------------------------------------------------------------
% Automatically adds project files to your MATLAB path, downloads the
% required files, and opens an example
%--------------------------------------------------------------------------
% Install script based on MFX Submission Install Utilities
% https://github.com/danielrherber/mfx-submission-install-utilities
% https://www.mathworks.com/matlabcentral/fileexchange/62651
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function INSTALL_PMVS_project

    % add contents to path
    AddSubmissionContents(mfilename)
    
    % download required web files
    % RequiredWebFiles

    % download required web zips
    RequiredWebZips
    
    % add contents to path (files have been downloaded)
    AddSubmissionContents(mfilename)
    
    % run install function for PM Architectures Project
    disp('--- Running INSTALL_PMA_project')
    evalc('INSTALL_PMA_project');
    disp(' ')
    
    % run install function for DT QP Project
    disp('--- Running INSTALL_DTQP')
    evalc('INSTALL_DTQP');
    disp(' ')
    
    % open an example
    OpenThisFile('PMVSex_JMD')
    
    % close this file
    CloseThisFile(mfilename) % this will close this file

end
%--------------------------------------------------------------------------
function RequiredWebFiles
    disp('--- Obtaining required web files')

    % initialize index
    ind = 0;

    % initialize structure
    files = struct('url','','folder','');

    % file 1
	%   ind = ind + 1; % increment 
	% 	files(ind).url = '';
	% 	files(ind).folder = '';

    % obtain full function path
    full_fun_path = which(mfilename('fullpath'));
    outputdir = fullfile(fileparts(full_fun_path),'include');

    % download
    DownloadWebFiles(files,outputdir)

    disp(' ')
end
%--------------------------------------------------------------------------
function RequiredWebZips
    disp('--- Obtaining required web zips')

    % initialize index
    ind = 0;

    % initialize structure
    zips = struct('url','','folder','','test','');

    % zip 1
	ind = ind + 1; % increment
	zips(ind).url = 'https://www.mathworks.com/matlabcentral/mlc-downloads/downloads/submissions/29312/versions/3/download/zip/diffxy.zip';
	zips(ind).folder = 'MFX 29312';
	zips(ind).test = 'diffxy';

    % zip 2
	ind = ind + 1; % increment
	zips(ind).url = 'https://github.com/danielrherber/pm-architectures-project/archive/refs/heads/master.zip';
	zips(ind).folder = 'MFX 58799';
	zips(ind).test = 'INSTALL_PMA_project';

    % zip 3
	ind = ind + 1; % increment
	zips(ind).url = 'https://github.com/danielrherber/dt-qp-project/archive/master.zip';
	zips(ind).folder = 'MFX 65434';
	zips(ind).test = 'INSTALL_DTQP';

    % obtain full function path
    full_fun_path = which(mfilename('fullpath'));
    outputdir = fullfile(fileparts(full_fun_path),'include');

    % download and unzip
    DownloadWebZips(zips,outputdir)

    disp(' ')
end
%--------------------------------------------------------------------------
function AddSubmissionContents(name)
	disp('--- Adding submission contents to path')
	disp(' ')

	% current file
	fullfuncdir = which(name);

	% current folder 
	submissiondir = fullfile(fileparts(fullfuncdir));

	% add folders and subfolders to path
	addpath(genpath(submissiondir)) 
end
%--------------------------------------------------------------------------
function CloseThisFile(name)
    disp(['--- Closing ', name])
    disp(' ')

    % get editor information
    h = matlab.desktop.editor.getAll;

    % go through all open files in the editor
    for k = 1:numel(h)
        % check if this is the file
        if ~isempty(strfind(h(k).Filename,name))
            % close this file
            h(k).close
        end
    end
end
%--------------------------------------------------------------------------
function OpenThisFile(name)
	disp(['--- Opening ', name])

	try
	    % open the file
	    open(name);
	catch % error
	    disp(['Could not open ', name])
	end

	disp(' ')
end
%--------------------------------------------------------------------------
function DownloadWebFiles(files,outputdir)

    % store the current directory
    olddir = pwd;
    
    % create a folder for outputdir
    if ~exist(outputdir, 'dir')
        mkdir(outputdir); % create the folder
    else
        addpath(genpath(outputdir)); % add folders and subfolders to path
    end
    
    % change to the output directory
    cd(outputdir)
    
    % go through each file
    for k = 1:length(files)
        
        % get data
        url = files(k).url;
        folder = files(k).folder;
        [~,nameurl,exturl] = fileparts(url);
        name = [nameurl,exturl];
        
        % first check if the test file is in the path
        if exist(name,'file') == 0
            
            try
                % download file
                outfilename = websave(name,url);
            
                % create a folder utilizing name as the foldername name
                if ~exist(fullfile(outputdir,folder), 'dir')
                    mkdir(fullfile(outputdir,folder));
                end

                % move the file
                movefile(outfilename,fullfile(outputdir,folder))

                % output to the command window
                disp(['Downloaded ',folder,'/',name])

            catch % failed to download
                % output to the command window
                disp(['Failed to download ',folder,'/',name])
                
                % remove the html file
                delete([name,'.html'])
            end
            
        else
            % output to the command window
            disp(['Already available ',name])
        end
    end
    
    % change back to the original directory
    cd(olddir)
end
%--------------------------------------------------------------------------
function DownloadWebZips(zips,outputdir)

    % store the current directory
    olddir = pwd;
    
    % create a folder for outputdir
    if ~exist(outputdir, 'dir')
        mkdir(outputdir); % create the folder
    else
        addpath(genpath(outputdir)); % add folders and subfolders to path
    end
    
    % change to the output directory
    cd(outputdir)

    % go through each zip
    for k = 1:length(zips)

        % get data
        url = zips(k).url;
        folder = zips(k).folder;
        test = zips(k).test;

        % first check if the test file is in the path
        if exist(test,'file') == 0

            try
                % download zip file
                zipname = websave(folder,url);

                % save location
                outputdirname = fullfile(outputdir,folder);

                % create a folder utilizing name as the foldername name
                if ~exist(outputdirname, 'dir')
                    mkdir(outputdirname);
                end

                % unzip the zip
                unzip(zipname,outputdirname);

                % delete the zip file
                delete([folder,'.zip'])

                % output to the command window
                disp(['Downloaded and unzipped ',folder])
            
            catch % failed to download
                % output to the command window
                disp(['Failed to download ',folder])
                
                % remove the html file
                delete([folder,'.html'])
                
            end
            
        else
            % output to the command window
            disp(['Already available ',folder])
        end
    end
    
    % change back to the original directory
    cd(olddir)
end