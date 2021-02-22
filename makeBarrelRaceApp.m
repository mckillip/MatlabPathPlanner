function makeBarrelRaceApp(ah)
% Puts image of field onto a surface in MATLAB, with the coordinates matching
% the supplied diagram in the 2021 documentation.
% X goes length of field (left to right), +Y goes up.  Positive angles
% follow RH rule.

    % Point to axes
    if nargin < 1
        ah = gca;
    end
    
    % Make the field!
    C = imread('BarrelRace.png');
    C = flipud(C);
    XData = [-4.3644 31.6017; -4.3644 31.6017];
    YData = [-5.1407 -5.1407; 16.0119 16.0119];
    ZData = zeros(2,2);
    sh = surface(ah,XData,YData,ZData,C,...
        'FaceColor','texturemap',...
        'EdgeColor','none',...
        'CDataMapping','direct');
    axis(ah,'equal');
    axis(ah,[-4.3644 31.6017 -5.1407 16.0119]);
%     fh = gcf;
%     fh.Position = [189 253 852 574];
    
end