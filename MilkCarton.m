classdef MilkCarton < handle
%UNTITLED2 Summary of this class goes here
%   Detailed explanation goes here

properties
    position
    rotation
    tag 
    size
    colors
    matrix
    
    hgT
    hgFront
    hgLeft
    hgBack
    hgRight
    hgBottom
    hgTop
    hgAngleFront
    hgAngleBack
    hgForceVector
end

methods %------------------------------------------------------------------
function obj = MilkCarton(varargin)
    obj.size = [3.5 9 3.5 2];  %inches [wid len depth h_angle]
    obj.position = [0 0 0];
    obj.rotation = [0 0 0];
    obj.colors = [0.1 0.1; 0.2 0.2; 0.3 0.3; 0.4 0.4; 0.5 0.5; 0.6 0.6; 0.7 0.7];
    obj.tag = 'carton';
    
    if nargin > 0
        for ii=1:2:length(varargin)
            switch varargin{ii}
                case 'tag'
                    obj.tag = varargin{ii+1};
                case 'position'
                    obj.position = varargin{ii+1};
                case 'size'
                    obj.size = varargin{ii+1};
                otherwise
                    warning('constructor:badInput','Unrecognized attribute name')
            end
        end
    end
       
end

%graphic functions
function plot(obj, parent,varargin)

if isempty(parent)
    error()
end

plotBox = true;
plotForce = true;

if isempty(obj.hgT)
    obj.hgT = hgtransform('Parent',parent,'tag',['hgT Carton']);
end
%p1 = obj.position(1); p2 = obj.position(2); p3 = obj.position(3);
p = obj.getPosition();
obj.hgT.Matrix = makehgtform('translate',p(1),p(2),p(3));

s = obj.size;
%the corners of the carton
e = [0 s(1); 0 s(2); 0 s(3)];

if plotBox
    hg = obj.plotBox(e,s,obj.hgT);
    obj.hgFront = hg.front;
    obj.hgLeft = hg.left;
    obj.hgBack = hg.back;
    obj.hgRight = hg.right;
    obj.hgBottom = hg.bottom;
    obj.hgTop = hg.top;
    obj.hgAngleFront = hg.angleFront;
    obj.hgAngleBack = hg.angleBack;
end

%plot vector
if plotForce
    force = obj.getForce();
    obj.hgForceVector = line([force(1,1),force(1,1)+force(2,1)],...
        [force(1,2),force(1,2)+force(2,2)],[force(1,3),force(1,3)+force(2,3)],...
        'Parent',obj.hgT);
end

end %plot
function remove(obj)
    for ii=1:length(obj)
       try
           delete(obj(ii).hgTop)
           delete(obj(ii).hgBottom)
           delete(obj(ii).hgFront)
           delete(obj(ii).hgLeft)
           delete(obj(ii).hgBack)
           delete(obj(ii).hgRight)
           delete(obj(ii).hgAngleFront)
           delete(obj(ii).hgAngleBack)
           obj(ii).hgTop =[];
           obj(ii).hgBottom = [];
           obj(ii).hgFront = [];
           obj(ii).hgLeft = [];
           obj(ii).hgBack = [];
           obj(ii).hgRight = [];
           obj(ii).hgAngleFront = [];
           obj(ii).hgAngleBack = [];
       catch
           warning('remove:failed',...
               'Failed to delete one or more objects.  Objects may already been removed.')
       end
       try
           delete(obj(ii).hgForceVector)
           obj(ii).hgForceVector = [];
       catch
           warning('remove:failed',...
               'Failed to delete one or more objects.  Objects may already been removed.')
       end
    end
end %remove
function setVisible(obj,state,varargin)
    %state {'on','off'}
    obj.hgFront.Visible = state;
    obj.hgLeft.Visible = state;
    obj.hgBack.Visible = state;
    obj.hgRight.Visible = state;
    obj.hgBottom.Visible = state;
    obj.hgTop.Visible = state;
    obj.hgAngleFront.Visible = state;
    obj.hgAngleBack.Visible = state;
    obj.hgForceVector.Visible = state;
end
%property functions
function resize(obj, newsize)
    
end %resize
function pos = getPosition(obj,varargin)
    pos = [obj.position(1), obj.position(2), obj.position(3)- obj.size(3)];
end %getPosition
function setPosition(obj, posit,rotate)
    
    obj.matrix = makehgtform('translate',posit(1),posit(2),-posit(3));
    obj.position = posit;
    if ~isempty(obj.hgT)
        obj.hgT.Matrix = obj.matrix;
    end
end %position
function siz = getLength(obj,varargin)
    %return length of sides as [1x3]
    %[option] return length of given dimension {1-4}
    dim  = [];
    if nargin > 1
        dim = varargin{1};
    end
    
    if isempty(dim)
        siz = [obj.size(1) obj.size(2) + obj.size(4) obj.size(3)];
    elseif dim == 2
        siz = obj.size(2) + obj.size(4);
    elseif dim < 5 && dim > 0
        siz = obj.size(dim);
    else
        warning('getLength:badInput','Dimension must be {1,2,3,4,[]}')
        return
    end
end %getLength
function vol = getVolume(obj)
    %return volume in gallons
    
    vol = obj.getArea(1) * obj.size(3); %in^3
%      vol = obj.size(1) * obj.size(2) * obj.size(3);
%      %triangle area = 1/2 * b * h
%      area = obj.size(2) * obj.size(4) * 0.5;
%      vol = vol + area * obj.size(1);
     vol = vol / 231;  %in^3 -> gal
end %getVolume
function loc = getCg(obj)
    %returns cg [x,y,z]
%**************Does not accurately compute cg for triangle
%     area = [obj.size(1)*obj.size(2),...
%         obj.size(2) * obj.size(3),...
%         obj.size(1) * obj.size(3),...
%         obj.size(4)*obj.size(1)*0.5]; %[xy1 yz xz xy2]
    len = [obj.size(1)/2,...
        obj.size(2)/2,...
        obj.size(3)/2,...
        obj.size(2)/2 + obj.size(4)*3/5]; %****** not centroid of a triangle+
    centX = len(1);% (area(1)*len(1) + area(4)*len(4))/(area(1)+area(4));
    %centY = (area(2)*len(2) + area(4)*len(4))/(area(2)+area(4));
    centZ = len(3);%area(3)*len(3) / area(3);

    centY = obj.getArea(2);
    x = obj.position(1) + centX;%obj.size(1)/2;
    y = obj.position(2) + centY;% obj.size(2)/2 + ...
           % + obj.size(2) + obj.size(4)/2;
    z = obj.position(3) - centZ;%obj.size(3)/2;
    
    %force = cg.* obj.position;
    loc = [x y z];
end %getCg
function centroid = getCentroid(obj,varargin)
    %return centroid location (local) in inches
    centX = obj.size(1)/2;
    centY = obj.size(2)/2 + obj.size(4)*3/5; %****** not centroid of a triangle+
    centZ = -obj.size(3)/2;
    
    centroid = [centX centY centZ];
end %getCentroid
function area = getArea(obj,varargin)
    %return areas [aXY aXZ aYZ] in in^2
    %[optional] return any single dimension
    dim  = [];
    if nargin > 1
        dim = varargin{1};
    end
    
    aXY = obj.size(1) * obj.size(2) + ...
            (0.5 * obj.size(1)*obj.size(4));
    aXZ = obj.size(1) * obj.size(3);
    aYZ = (obj.size(2)+obj.size(4)) * obj.size(3);
    
    area = [aXY aXZ aYZ];
    if isempty(dim)
        area = [aXY aXZ aYZ];
    elseif dim < 4 && dim > 0
        area = area(dim);
    else
        warning('getArea:badInput','Dimension must be {aXY,aXZ,aYZ,[]}')
        return
    end
   
end %getArea
function displ = getDisplacement(obj,depth)
    %returns displacement in gallons
    if nargin == 1
        depth = obj.size(3); %fully submersed
    end
    if depth > obj.size(3)
        warning('getForce:badInput',...
            'Depth exceeds carton height')
        return
    end
    
    %vol = obj.getArea(1) * obj.size(3);
%     area = obj.size(1) * obj.size(2) + ...
%         (0.5 * obj.size(1)*obj.size(4));
    displ = obj.getArea(1) * depth / 231; %gal
    %displ = obj.getVolume * depth;%gal
end %getDisplacement
function displ = getCoverage(obj,depth)
    %return total displacement [gal] as a function of height of waterline to the position
    %of the carton
    %e.g. carton at z=-7 with waterline of z=-2 would be fully displaced
    
    dh = depth + obj.position(3);% - obj.size(3); 
    %eg pos=-7, size3 = 3.5, depth = 11 ->full displacement ~0.5gal
    %eg pos-7, size3= 3.5, depth = 6 -> no displacment 0 gal
    dh = min(obj.size(3), dh);
    displ = max(0,obj.getDisplacement(dh));
    
%     dh = depth + obj.position(3);% - obj.size(3); 
%     %eg pos=-7, size3 = 3.5, depth = 11 ->full displacement ~0.5gal
%     %eg pos-7, size3= 3.5, depth = 6 -> no displacment 0 gal
%     dh = min(obj.size(3), dh);
%     displ = max(0,obj.getDisplacement(dh));
end %getCovereage
function vec = getForce(obj,depth)
    %returns a vector, direction and magnitude [x0,y0,z0;Fx,Fy,Fz]
    if nargin == 1
        depth = obj.size(3);
    end
    density = 8.0;  %lbm/gal
    force = obj.getDisplacement(depth) * density;
    loc = obj.getCentroid();
    vec = [loc; [0 0 force]];
end %getForce
end %methods
%--------------- Static Functions ----------------------------
methods (Static)
function hg = plotBox(e,s,parent)

%front
srfc.x = [e(1,1) e(1,1); e(1,2) e(1,2)];
srfc.y = [e(2,1) e(2,1); e(2,1) e(2,1)];
srfc.z = [e(3,1) e(3,2); e(3,1) e(3,2)];
cdata=[0.5 0.5;0.5 0.5];
hg.front = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
     'CData',cdata,'Tag','Front','Parent',parent);
%left side
srfc.x = [e(1,1) e(1,1); e(1,1) e(1,1)];
srfc.y = [e(2,1) e(2,1); e(2,2) e(2,2)];
srfc.z = [e(3,1) e(3,2); e(3,1) e(3,2)];
cdata=[0.55 0.55;0.55 0.55];
hg.left = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Left','Parent',parent);
%back side
srfc.x = [e(1,1) e(1,1); e(1,2) e(1,2)];
srfc.y = [e(2,2) e(2,2); e(2,2) e(2,2)];
srfc.z = [e(3,1) e(3,2); e(3,1) e(3,2)];
cdata=[0.6 0.6;0.6 0.6];
hg.back = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Back','Parent',parent);
%right side
srfc.x = [e(1,2) e(1,2); e(1,2) e(1,2)];
srfc.y = [e(2,1) e(2,1); e(2,2) e(2,2)];
srfc.z = [e(3,1) e(3,2); e(3,1) e(3,2)];
cdata=[0.65 0.65;0.65 0.65];
hg.right = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Right','Parent',parent);
%top side
srfc.x = [e(1,1) e(1,2); e(1,1) e(1,2)];
srfc.y = [e(2,1) e(2,1); e(2,2) e(2,2)];
srfc.z = [e(3,2) e(3,2); e(3,2) e(3,2)];
cdata=[0.7 0.7;0.7 0.7];
hg.top = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Top','Parent',parent);
%bottom side
srfc.x = [e(1,1) e(1,2); e(1,1) e(1,2)];
srfc.y = [e(2,1) e(2,1); e(2,2) e(2,2)];
srfc.z = [e(3,1) e(3,1); e(3,1) e(3,1)];
cdata=[0.75 0.75;0.75 0.75];
hg.bottom = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Bottom','Parent',parent);

             %******test - only create these shapes if s(4) > 0
%top back angle side
srfc.x = [e(1,2)/2 e(1,2)/2; e(1,2) e(1,2)];
srfc.y = [e(2,2)+s(4) e(2,2)+s(4); e(2,2) e(2,2)];
srfc.z = [e(3,1) e(3,2); e(3,1) e(3,2)];
cdata=[0.75 0.75;0.75 0.75];
hg.angleFront = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Top Front Angle','Parent',parent);       
%top front angle side
srfc.x = [e(1,1) e(1,1); e(1,2)/2 e(1,2)/2];
srfc.y = [e(2,2) e(2,2); e(2,2)+s(4) e(2,2)+s(4)];
srfc.z = [e(3,1) e(3,2); e(3,1) e(3,2)];
cdata=[0.5 0.5;0.5 0.5];
hg.angleBack = surface('xdata',srfc.x,'ydata',srfc.y,'zdata',srfc.z,...
                 'CData',cdata,'Tag','Top Back Angle','Parent',parent);            

end %plotBox
end %----------- Static functions

end %MilkCarton

