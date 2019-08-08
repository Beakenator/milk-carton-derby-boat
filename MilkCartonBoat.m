classdef MilkCartonBoat < handle
%UNTITLED Summary of this class goes here
%   Detailed explanation goes here

properties
    tag

    layerDefns  %definitions for layers
    cartons     %MilkCarton[]
    loads
    
    hgWaterline
    hgLoads
end

methods
function obj = MilkCartonBoat(varargin)

end %constructor
function float(obj,weight)
    
end %float
function report = getReport(obj,varargin)
    %output statistics on boat
    %dimensions, #cartons, weight, 
end %getReport

%--------- plot
function plot(obj,varargin)
    %plot boat
    %plot waterline
    %plot load/buoyancy graph
    
end %plot
function remove(obj,varargin)
    
end %remove
end %methods
    
end %MilkCartonBoat

