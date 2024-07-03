classdef Map_mh < handle
    properties
        matrix;
        location_matrix;
        chromo;
        chromo_number;
        chromo_location;
        mission_location;
        mission_num;
        
       
    end
    properties(SetAccess = private)
        image_length;
        image_hieght;
        resolution;
        
    end
    methods
        function obj = Map_mh(file_address, varargin)
            
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
            
            obj.chromo_number=size(obj.matrix,1)*size(obj.matrix,2);
            
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
        function encoding(obj)
            
            
            % Set the location matrix
            % Generate random order permutation of grids
            % obj.chromo=randperm(obj.mission_num);
            
            obj.chromo=zeros(size(obj.mission_num,1),1);
            count=1;
            while obj.chromo(end)==0
                obj.chromo(count,1)=obj.mission_num(ceil(rand*size(obj.mission_num,1)));
                count=count+1;
            end
            % Sort the chromosome
            for i=1:size(obj.chromo,1)
                
                obj.chromo_location(i,:)=obj.location_matrix(obj.chromo(i),:);
            end
            
            % Plot the current chromosome
            % figure
            % plot(obj.chromo_location(:,1),obj.chromo_location(:,2))
            % hold on
            % plot(obj.chromo_location(:,1),obj.chromo_location(:,2),'r.')
            % contour(obj.matrix)
            
            
            
        end
        
        
        function evaluating(obj)
            % Set the speed limit
            % Simplify the battery drain rate
            speed_max=0.5;
            speed_min=0.1;
            
            dis_per_battery_max=speed_max*2;
            dis_per_battery_min=speed_min*5;
            % Calculate Euclidean between each nodes
            for i= 1: obj.chromo_number-1
                nodes_dis(i)=norm(obj.chromo_location(i,:)-obj.chromo_location(i+1,:));
                % Insert charging period in each travel
                charging_number_node(i)=randi([floor(nodes_dis(i)/dis_per_battery_max),floor(nodes_dis(i)/dis_per_battery_min)]);
                if i==1
                    chromo_charging=obj.chromo(1);
                else
                    chromo_charging=cat(1,chromo_charging,[obj.chromo(i);zeros(charging_number_node(i),1)]);
                end
            end
            chromo_charging=[chromo_charging;obj.chromo(end)];
            % Cost function (distance)
            
            cost_dis=sum(nodes_dis);
            
            % Cost function (time)
            
            % Fitness evaluation
            
            
        end
        
        function lawn_mower(obj)
            
            x_min = min(obj.mission_location(:,1))+1;
            x_max = max(obj.mission_location(:,1));
            for i = x_min:2:x_max
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