%--------------------------------------------------------------------------
% PMVS_Scaling.m
% Scale or unscale a given input
%--------------------------------------------------------------------------
% 
%--------------------------------------------------------------------------
% Primary contributor: Daniel R. Herber (danielrherber), University of 
% Illinois at Urbana-Champaign
%--------------------------------------------------------------------------
function X = PMVS_Scaling(x,l,u,type)

% reshape to column vector
x = reshape(x,[],1);

% scale or unscale
if type == 1 % scale
    X = (x-l)./(u-l); 
elseif type == 2 % scaling
    X = l + x.*(u-l);
end

end