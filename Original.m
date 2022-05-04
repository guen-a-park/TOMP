% ==============================================================================
% MATLAB Source Codes for Time-Optimal Maneuver Planning in Automatic Parallel Parking
% Using a Simultaneous Dynamic Optimization Approach
% ==============================================================================

%   Copyright (C) 2016 Bai Li
%   Useers must cite the following article when utilizing this source codes. 
%   Bai Li, et al. "Time-Optimal Maneuver Planning in Automatic Parallel Parking
%   Using a Simultaneous Dynamic Optimization Approach", IEEE Transactions on
%   Intelligent Transportation Systems, in press. 

% ==============================================================================
clear all;close all;clc
% Note that the following three settings should be specified in the file
% Case1.mod AS WELL.

SL = 6.75; % Parallel parking spot length.
SW = 2; % Parallel parking spot width.
CL = 3.5; % Road wideth.

% Start the NLP-solving process.
!AMPL rr.run 

% Result demonstration: static parking motions:
figure (1)
global colorpool
colorpool = [51,161,201]./255;
global car_n
global car_l
global car_m
global car_b

car_n = 0.839;
car_l = 2.588;
car_m = 0.657;
car_b = 0.8855;

load tf.txt
load NENE.txt

for ind = 1:1
    
    num = NENE(ind)*3;
    temp = NENE(ind)*4;
    
    NE = NENE(ind);
    
    load x.txt
    x = x(((temp*(ind-1))+1):((temp*(ind-1)) + temp));
    tempp = x(1,1);
    x = reshape(x,4,NE);
    x = x';
    x = x(1:NE,2:4);
    x = reshape(x',1,num);
    x = [tempp,x];
    
    load y.txt
    y = y(((temp*(ind-1))+1):((temp*(ind-1)) + temp));
    tempp = y(1,1);
    y = reshape(y,4,NE);
    y = y';
    y = y(1:NE,2:4);
    y = reshape(y',1,num);
    y = [tempp,y];

    load t.txt
    t = t(((temp*(ind-1))+1):((temp*(ind-1)) + temp));
    tempp = t(1,1);
    t = reshape(t,4,NE);
    t = t';
    t = t(1:NE,2:4);
    t = reshape(t',1,num);
    t = [tempp,t];
        
    drawfigure(x,y,t,ind);
end
Figure_decoration

% Result demonstration: the optimized profiles:
figure (2)
Profile_plot

pause(10)
% Result demonstration: the dynamic parking maneuvers:
figure (3)
Dynamic_movie