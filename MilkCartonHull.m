classdef MilkCartonHull < handle
%UNTITLED Summary of this class goes here
%   Detailed explanation goes here

properties
    tag

    cartonDefn  %MilkCarton[1] template object
    layers  %definitions for layers
    cartons     %MilkCarton[]
    loads
    
    hgParent
    hgWaterline %blue surface nominally at z=0
    hgLoads
    hgT         %transform to raise/lower hull relative to waterline
end

properties (Hidden)
    densityWater = 8.0;
end

methods
function obj = MilkCartonHull(varargin)
    obj.tag = 'Boat Hull';
    if nargin == 1
        obj.layers = varargin{1};
    else
        for ii=1:2:length(varargin)
            switch varargin{ii}
                case 'tag'
                    obj.tag = varargin{ii+1};
                case 'layers'
                    obj.layers = varargin{ii+1};
                case 'carton'
                    obj.cartonDefn = varargin{ii+1};
                otherwise
                    warning('constructor:badInput','Unrecognized Input')
            end
        end
    end
end %constructor
function build(obj,varargin)
    %build cartons from the layers definition
    %[optional] provide layers definition as input
    
    if nargin > 1
        
        if ~isempty(obj.cartons)
            warning('build:overwrite','Overwriting existing cartons')
            obj.cartons.remove()
            obj.cartons = [];
        end
        obj.layers = varargin{1};
    end
    
    %position about the X centerline with -Z coord
    nCart = 0;
    for nZ = 1:length(obj.layers)
        for nY= 1:length(obj.layers{nZ}.count) %rows
            for nX = 1:obj.layers{nZ}.count(nY) %columns
                nCart = nCart + 1;

                obj.cartons = [obj.cartons; ...
                    MilkCarton('tag',['Hull:',obj.tag,',Layer:',num2str(nZ),...
                        ',Row/Col:',num2str(nX),'/',num2str(nX),...
                        ',Carton:',num2str(nCart)])];
                siz = obj.cartons(nCart).getLength();
                rowWidth = siz(1) * obj.layers{nZ}.count(nY);
                dX = -rowWidth / 2; %step
                x = (nX-1) * siz(1) + obj.layers{nZ}.offset(1) + dX;
                y = (nY-1) * siz(2) + obj.layers{nZ}.offset(2);
                z = - (nZ-1) * siz(3) + obj.layers{nZ}.offset(3);

                obj.cartons(nCart).setPosition([x y z],[0 0 0]);
            end
        end
    end
    
end %build
function vargout = float(obj,weight)
    %move the hull in the z-direction for the given weight
    draft = obj.getProperty('draft');
    total = obj.getBuoyancy([0 ceil(draft)]); %must pass integers
    unq = unique(total(2,:));
    len = length(unq);
    tot = total(:,1:len);
    
    pp = interp1(tot(2,:)* obj.densityWater,tot(1,:),weight);
    if isnan(pp)
        dh = 0;%-draft;
    else
        dh = draft - pp;
    end
    obj.hgT.Matrix(3,4) = dh;
    
    if nargout == 1
        vargout(1) = dh;
    end
end %float

%--------------------- analysis functions ---------------------------------
function prop = getProperty(obj,varargin)
    
    if nargin > 1
        select = varargin{1};
    else
        select = 'cartons';
    end
    
    switch select
        case 'cartons'
            prop = 0;
            for ii =1:length(obj.layers)
                prop = prop + sum(obj.layers{ii}.count);
            end        
        case 'draft'
            prop = length(obj.layers)* obj.cartonDefn.getLength(3);            
        case 'length'
            prop = length(obj.layers{1}.count) * obj.cartonDefn.getLength(2);            
        case 'width'
            prop = max(obj.layers{1}.count) * obj.cartonDefn.getLength(1);            
        otherwise
            
            prop = [];
    end

end %getProperty
function getReport(obj,varargin)
    %output statistics on boat
    %dimensions, #cartons, weight, 
    %obj.layers{depth, row from back}
%offset [from rear]  assumes centered
% obj.layers{1}.count = [2; 3; 4; 5; 6; 7; 7; 6;5; 3; 2];
% obj.layers{1}.offset = [0 0 0];
% obj.layers{2}.count = [3; 4;5;6;6;6;6;4;3;2];
% obj.layers{2}.offset = [0 size(2)/2 0];  %half carton length
% obj.layers{3}.count = [2;3;4;5;5;4;3;2];
% obj.layers{3}.offset = [0 size(2) 0];  %carton length

    if isempty(obj.layers)
        warning('getReport:missingData','Layers not defined')
        return
    end
total = 0;
for ii =1:length(obj.layers)
    total = total + sum(obj.layers{ii}.count);
end

len = length(obj.layers{1}.count) * obj.cartonDefn.getLength(2);
wid = max(obj.layers{1}.count) * obj.cartonDefn.getLength(1);
draft = length(obj.layers)* obj.cartonDefn.getLength(3);

fprintf('Hull:%s\n',obj.tag)
fprintf('Total Cartons:%d\n',total)
fprintf('Length Overall:%1.0fft-%2.1fin\n',floor(len/12),rem(len,12))
fprintf('Widest point:%1.0fft-%2.1fin\n',floor(wid/12),rem(wid,12))
fprintf('Depth:%2.1fin\n',draft)

%report = [];
end %getReport
function total = getBuoyancy(obj,varargin)
    %return function of force = f(depth)
    if nargin > 1
        depthLim = varargin{1};
    else
        depthLim = [0 11];
    end
    
    incr = 1;
    nPts = (depthLim(2) - depthLim(1))/incr;
    curve = zeros(nPts,length(obj.cartons));
    for ii=1:length(obj.cartons)
        %for each layer, compute #cartons * displacement=f(depth) from the reference
        %carton
        for jj = 1:nPts+1%depthLim(1):incr:depthLim(2)
            dh = (jj-1) * (depthLim(2) - depthLim(1))/nPts;
            curve(jj,ii)= obj.cartons(ii).getCoverage(dh);
        end
    end
%     draft = length(obj.layers)* obj.cartonDefn.getLength(3);
%     for ii=1:length(obj.cartons)
%         %for each layer, compute #cartons * displacement=f(depth) from the reference
%         %carton
%         for jj = nPts+1:-1:1
%             dh = ((jj-1) * (depthLim(1) - depthLim(2))/nPts);
%             curve(jj,ii)= obj.cartons(ii).getCoverage(dh);
%         end
%     end
    total = [depthLim(1):incr:depthLim(2);sum(curve,2)'];
end %getBuoyancy
%--------- plot
function plot(obj,varargin)
    %plot boat
    %plot waterline
    %plot load/buoyancy graph
if nargin > 1
    obj.hgParent = varargin{1};
end
if isempty(obj.hgParent)
    warning()
    return
end
if isempty(obj.hgT)
    obj.hgT = hgtransform('Parent',obj.hgParent,'tag',[obj.tag,':hgT']);
end
%p1 = obj.position(1); p2 = obj.position(2); p3 = obj.position(3);
p = [0 0 0];%obj.getPosition();
obj.hgT.Matrix = makehgtform('translate',p(1),p(2),p(3));

%plotWater = true;
plotHull = true;

if plotHull
    for nCart = 1:length(obj.cartons)
        obj.cartons(nCart).plot(obj.hgT)
    end
%     for nZ = 1:length(obj.layers)
%         for ii = 1:length(obj.layers{nZ}.carton)
%             obj.layers{nZ}.carton(ii).plot(obj.hgT)
%         end
%     end

end
% if plotWater
%   %the extents of the water surface
%     e = [-50 50; -20 140; 0 0];
%     srfc.x = [e(1,1) e(1,2); e(1,1) e(1,2)];
%     srfc.y = [e(2,1) e(2,1); e(2,2) e(2,2)];
%     srfc.z = [e(3,2) e(3,2); e(3,2) e(3,2)];
%     cdata=[0.5 0.5;0.5 0.5];
%     obj.hgWaterline = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
%          'CData',cdata,'Tag',[obj.tag,':Waterline'],...
%          'FaceAlpha',0.5,'Parent',obj.hgT);
% end

end %plot
function remove(obj,varargin)
    delete(obj.hgWaterline)
    obj.hgWaterline = [];
    
    if ~isempty(obj.cartons)
        %MilkCarton remove() function supports array - no need for for loop
        obj.cartons.remove()        
    end
end %remove
end %methods
    
end %MilkCartonHull

