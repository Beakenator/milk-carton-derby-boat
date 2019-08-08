%demo Milk Carton Derby boat design

%actual measured size 97mm x 196mm x 97mm + 25mm top (without tange)
%converts to 3.82 x 7.72 x 3.82 + 0.98 inches

density.water = 8.0;  %lbm/gal
density.wood = 5;  %estimate

%create a carton definition
cartDfn(1) = MilkCarton('tag','Measured','size',[3.82 7.72 3.82 0.98]); %standard size
cartDfn(2) = MilkCarton('tag','European','size',[3 10 3 1]);

%% axes

f.fig(1)=figure(1);
f.ax(1)=axes;
%f.ax(1)= subplot(2,1,1);
f.hgT = hgtransform('tag','Global');
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

plotWater = true;

if plotWater
  %the extents of the water surface
    e = [-30 60; -10 220; 0 0];
    srfc.x = [e(1,1) e(1,2); e(1,1) e(1,2)];
    srfc.y = [e(2,1) e(2,1); e(2,2) e(2,2)];
    srfc.z = [e(3,2) e(3,2); e(3,2) e(3,2)];
    cdata=[0.5 0.5;0.5 0.5];
    obj.hgWaterline = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
         'CData',cdata,'Tag','Waterline',...
         'FaceAlpha',0.5,'Parent',f.hgT);
end
clear('srfc','cdata','e')

%% start building boat hulls
nHull = 0; 

%% boat hull #1
%{
%layers{depth, row from back}
%offset [from rear]  assumes centered
nCart = 1; nHull = nHull + 1;
nLayer = 1;
layer{nLayer}.count = [2; 3; 4; 5; 6; 6; 6; 7; 7; 5; 3; 2; 1];
layer{nLayer}.offset = [0 0 0];
nLayer = nLayer + 1;
layer{nLayer}.count = [2; 3; 4; 5; 6; 6; 6; 5; 4; 3; 2; 1];
layer{nLayer}.offset = [0 cartDfn(nCart).size(2)*0.5*(nLayer-1) 0];  %half carton length
nLayer = nLayer + 1;
layer{nLayer}.count = [1; 2; 3; 4; 5; 5; 5; 5; 4; 3; 2; 1];
layer{nLayer}.offset = [0 cartDfn(nCart).size(2)*0.5*(nLayer-1) 0];  %carton length
% nLayer = nLayer + 1;
% layer{nLayer}.count = [1; 2; 3; 3; 3; 3; 3; 3; 3; 2; 1];
% layer{nLayer}.offset = [0 cartDfn(nCart).size(2)*0.5*(nLayer-1) 0];  %carton length

hull(nHull) = MilkCartonHull('tag',['Hull#',num2str(nHull)],...
    'layers',layer,'carton',cartDfn(nCart));
hull(nHull).build

hull(nHull).plot(f.hgT);
hull(nHull).hgT.Matrix(1,4) = (nHull-1)*40;
hull(nHull).getReport()
clear('layer')
%% boat hull #2

nCart = 1; nHull = 2;
layer{1}.count = [2; 3; 4; 5; 6; 7; 8; 7; 5; 3; 2];
layer{1}.offset = [0 0 0];
layer{2}.count = [3; 4; 5; 6; 6; 6; 6; 4; 3; 2];
layer{2}.offset = [0 cartDfn(nCart).size(2)/2 0];  %half carton length
layer{3}.count = [2; 3; 4; 5; 5; 4; 3; 3; 2];
layer{3}.offset = [0 cartDfn(nCart).size(2) 0];  %carton length

hull(nHull) = MilkCartonHull('tag',['Hull#',num2str(nHull)],...
    'layers',layer,'carton',cartDfn(nCart));
hull(nHull).build

hull(nHull).plot(f.hgT)
hull(nHull).hgT.Matrix(1,4) = (nHull-1)*40;
hull(nHull).getReport()

clear('layer')
%}
%% hull #3 long hull
% 30" wide plywood can accomodate 7 cartons wide with 3.25" border

nCart = 1; nHull = nHull + 1;
%aft -> fore
nLayer = 1;
layer{nLayer}.count = [2; 3; 4; 5; 6; 6; 6; 7; 7; 7; 7; 7; 7; 7; 6; 5; 3; 1];
layer{nLayer}.offset = [0 0 0];
nLayer = nLayer + 1;
layer{nLayer}.count = [2; 3; 4; 5; 6; 6; 6; 6; 6; 6; 6; 6; 6; 6; 5; 4; 2; 1];
layer{nLayer}.offset = [0 0 0];
%nLayer = nLayer + 1;
% layer{nLayer}.count = [1; 2; 3; 4; 5; 5; 5; 5; 5; 5; 5; 4; 4; 3; 2; 2; 1; ];
% layer{nLayer}.offset = [0 1 0]; %inches [dx dy dz]

hull(nHull) = MilkCartonHull('tag',['Hull#',num2str(nHull)],...
    'layers',layer,'carton',cartDfn(nCart));
hull(nHull).build

hull(nHull).plot(f.hgT)
hull(nHull).hgT.Matrix(1,4) = (nHull-1)*40;
hull(nHull).getReport()
clear('layer')
%% boat hull #4 (Roans pontoon)

%layers{depth, row from back}
%offset [from rear]  assumes centered
nCart = 1; nHull = nHull + 1;
nLayer = 1;
layer{nLayer}.count = [2; 3; 3; 3; 3; 3; 2; 1];
layer{nLayer}.offset = [0 0 0];
nLayer = nLayer + 1;
layer{nLayer}.count = [2; 3; 3; 3; 3; 3; 2; 1];
layer{nLayer}.offset = [0 0 0];  %

hull(nHull) = MilkCartonHull('tag',['Hull#',num2str(nHull)],...
    'layers',layer,'carton',cartDfn(nCart));
hull(nHull).build

hull(nHull).plot(f.hgT);
hull(nHull).hgT.Matrix(1,4) = (nHull-1)*40;
hull(nHull).hgT.Matrix(2,4) = 50;%(nHull-1)*40;
hull(nHull).getReport()
clear('layer')
%}
%% create hull buoyancy chart

figure(2)
axes
xlabel('Depth [in]')
ylabel('Buoyancy [lbm]')
title('Hull Buoyancy')
grid on
hold on

str = {};
for ii = 1:nHull
    total = hull(ii).getBuoyancy([0 13]);
    total(2,:) = total(2,:) *density.water;
    analysis(ii).force = plot(total(1,:),total(2,:)); %#ok

    str = [str ['Hull#',num2str(ii)]]; %#ok
end    
legend(str,'location','northwest');

%% run some buoyancy analysis

dh = hull(1).float(500)  %put 500# on hull#1 and get the height out of water
dh = hull(2).float(500)
% dh = hull(3).float(650)
% dh = hull(4).float(80)

%% loads

person(1)=170;
person(2)=190;
person(3)=100;
person(4)=100;
person(5)=60;

structure(1) = 40;  %wood and 
structure(2) = 15; %wood & rudder

equipment(1) = 20; %oars, lifejackets
equipment(2) = 5; %oars, lifejackets

load(1) = sum(person(1:4)) + structure(1) + equipment(1);
load(2) = person(4) + structure(2) + equipment(2);

dh(1) = hull(1).float(load(1)*1.0)
dh(2) = hull(2).float(load(2)*1.0)
