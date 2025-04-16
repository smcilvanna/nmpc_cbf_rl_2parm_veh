classdef Normalizer
    methods (Static)
        %% Normalize to [0, 1]
        function [normalized, minVal, maxVal] = normalize01(data)
            % Normalize data to [0, 1] range
            % [n, minVal, maxVal] = Normalizer.normalize01(data)
            minVal = min(data(:));
            maxVal = max(data(:));
            
            if maxVal == minVal
                error('Normalizer:ConstantData', 'Cannot normalize constant data');
            end
            
            normalized = (data - minVal) / (maxVal - minVal);
        end
        
        function denormalized = denormalize01(normalized, minVal, maxVal)
            % Denormalize from [0, 1] to original range
            denormalized = normalized * (maxVal - minVal) + minVal;
        end
        
        %% Normalize to [-1, 1]
        function [normalized, minVal, maxVal] = normalize11(data)
            % Normalize data to [-1, 1] range
            % [n, minVal, maxVal] = Normalizer.normalize11(data)
            minVal = min(data(:));
            maxVal = max(data(:));
            
            if maxVal == minVal
                error('Normalizer:ConstantData', 'Cannot normalize constant data');
            end
            
            normalized = 2 * ((data - minVal) / (maxVal - minVal)) - 1;
        end
        
        function denormalized = denormalize11(normalized, minVal, maxVal)
            % Denormalize from [-1, 1] to original range
            denormalized = ((normalized + 1) / 2) * (maxVal - minVal) + minVal;
        end
    end
end