function makeBouncePathApp(ah)
% Puts image of field onto a surface in MATLAB, with the coordinates matching
% the supplied diagram in the 2021 documentation.
% X goes length of field (left to right), +Y goes up.  Positive angles
% follow RH rule.

    % Point to axes
    if nargin < 1
        ah = gca;
    end
    
    % Make the field!
    C = imread('BouncePath.png');
    C = flipud(C);
    XData = [-4.083 30.968; -4.083 30.968];
    YData = [-4.9153 -4.9153; 15.8644 15.8644];
    ZData = zeros(2,2);
    sh = surface(ah,XData,YData,ZData,C,...
        'FaceColor','texturemap',...
        'EdgeColor','none',...
        'CDataMapping','direct');
    axis(ah,'equal');
    axis(ah,[-4.083 30.968 -4.083 15.8644]);
%     fh = gcf;
%     fh.Position = [189 253 852 574];
    
end