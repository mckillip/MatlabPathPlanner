function [rr,w_des] = smoothControl2(V,currentPos,targetPos)
% Executes algorithm from Park and Kuipers for Smooth Control
% but using a JAVA class as a test!

    persistent isInit % flag for initialization section
    persistent realSmooth % saved Java object
    
    % Initialization section
    if isempty( isInit ) % true if our first time
        isInit = 1;
        javaaddpath(pwd);
        realSmooth = Smoothcontrol; % create an instance of class
    end
    
    % Copy parameters, compute values, and set to outputs
    realSmooth.setSpeed( V );
    realSmooth.setPose( currentPos(1), currentPos(2), currentPos(3) );
    realSmooth.setTarget( targetPos(1), targetPos(2), targetPos(3) );
    realSmooth.computeTurnRate;
    rr = realSmooth.getRange;
    w_des = realSmooth.getWdes;

end

