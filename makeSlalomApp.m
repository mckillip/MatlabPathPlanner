function makeSlalomApp(ah)
% Puts image of field onto a surface in MATLAB, with the coordinates matching
% the supplied diagram in the 2021 documentation.
% X goes length of field (left to right), +Y goes up.  Positive angles
% follow RH rule.

    % Point to axes
    if nargin < 1
        ah = gca;
    end
    
    % Make the field!
    C = imread('SlalomPath.png');
    C = flipud(C);
    XData = [-4.1253 30.99; -4.1253 30.99];
    YData = [-5.089 -5.089; 15.657 15.657];
    ZData = zeros(2,2);
    sh = surface(ah,XData,YData,ZData,C,...
        'FaceColor','texturemap',...
        'EdgeColor','none',...
        'CDataMapping','direct');
    axis(ah,'equal');
    axis(ah,[-4.1253 30.99 -5.089 15.657]);
%     fh = gcf;
%     fh.Position = [189 253 852 574];
    
    
end