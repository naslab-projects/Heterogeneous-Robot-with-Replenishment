classdef Map < handle
    properties
        matrix;
        location_matrix;
        mission_location;
        mission_num;
        res;
        
        
    end
    properties(SetAccess = private)
        image_length;
        image_hieght;
        resolution;
        
    end
    methods
        function obj = Map(file_address, varargin)
            switch nargin
                case 5
                    original_matrix = im2bw(flipud(imread(file_address)));
                    original_hieght = size(original_matrix,1);
                    original_length = size(original_matrix,2);
                    
                    parser_obj = inputParser;
                    addRequired(parser_obj,'file_address',@ischar);
                    addParameter(parser_obj,'length',[]);
                    addParameter(parser_obj,'hieght',[]);
                    addParameter(parser_obj,'resolution',1);
                    parse(parser_obj,file_address,varargin{:});
                    obj.resolution = parser_obj.Results.resolution;
                    obj.image_hieght = parser_obj.Results.hieght;
                    obj.image_length = parser_obj.Results.length;
                    
                    if(isempty(obj.image_hieght)&&isempty(obj.image_length))
                        scale = 1/obj.resolution;
                        obj.image_hieght = original_hieght;
                        obj.image_length = original_length;
                    elseif(~isempty(obj.image_hieght)&& ~isempty(obj.image_length))
                        scale = [obj.image_hieght,obj.image_length]./[original_hieght,original_length]./obj.resolution;
                    elseif(isempty(obj.image_hieght))
                        scale = obj.image_length/original_length/obj.resolution;
                        obj.image_hieght = floor(original_hieght*obj.image_length/original_length);
                    elseif(isempty(obj.image_length))
                        scale = obj.image_hieght/original_hieght/obj.resolution;
                        obj.image_length = floor(original_length*obj.image_hieght/original_hieght);
                    end
                    
                    obj.matrix = imresize(original_matrix,scale,'nearest');
            end
        end
        
        function show(obj, varargin)
            
            BaseFigure = figure;
            set(BaseFigure,'name','Figure of the workspace','numbertitle','off');
            
            switch nargin
                case 1
                    contour([1:size(obj.matrix,2)]*obj.resolution,[1:size(obj.matrix,1)]*obj.resolution,obj.matrix,1);
                    caxis([0 1])
                    colormap(gray)
                case 2
                    if (ismember('border',varargin) || ismember('Border',varargin))
                        contour([1:size(obj.matrix,2)]*obj.resolution,[1:size(obj.matrix,1)]*obj.resolution,obj.matrix,1)
                        caxis([0 1])
                        colormap(gray)
                    elseif (ismember('fill',varargin) || ismember('Fill',varargin))
                        contourf([1:size(obj.matrix,2)]*obj.resolution,[1:size(obj.matrix,1)]*obj.resolution,obj.matrix,1)
                        caxis([-1 1])
                        colormap(bone)
                    end
            end
            
        end
        
        function represent(obj)
            
            % Establish the location matrix and find the location of mission area
            obj.location_matrix=zeros(size(obj.matrix,1)*size(obj.matrix,2),2);
            % location_matrix(1:size(obj.matrix,1))=1:size(obj.matrix,1);
            count=1;
            n=1;
            
            for i=1:size(obj.matrix,2)
                for j=1:size(obj.matrix,1)
                    obj.location_matrix(j+size(obj.matrix,1)*(i-1),:)=[i,j];
                    if obj.matrix(j,i)==0
                        obj.mission_num(count)=n;
                        
                        count=count+1;
                    end
                    n=n+1;
                end
                
            end
            
            for i=1:length(obj.mission_num)
                obj.mission_location(i,:)=obj.location_matrix(obj.mission_num(i),:);
            end
            
        end% represent
        
        function lawn_mower(obj)
            
            x_min = min(obj.mission_location(:,1));
            x_max = max(obj.mission_location(:,1));
            for i = x_min:obj.res*2:x_max
                sm = find (obj.mission_location(:,1) == i);
                obj.mission_location(sm,2)= flip(obj.mission_location(sm,2));
                
            end
        end
        
        function color_map(obj,color_matrix) % Visualize the map
            obj.matrix = +obj.matrix;  % Turn the logical matrix into double matrix
            n = 1;
            for i=1:size(obj.matrix,2)
                for j=1:size(obj.matrix,1)
                    
                    if obj.matrix(j,i)==0
                        obj.matrix(j,i)=color_matrix(n);
                        
                        n=n+1;
                    end
                    
                end
                
            end
        end
        
        function obstacle(obj,varargin)
            
        end
        
        function priority(obj,varargin)
            
        end
        
        function current(obj,varargin)
            
        end
        
    end % methods
    
    
end % classdef