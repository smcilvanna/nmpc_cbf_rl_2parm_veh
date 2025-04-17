classdef Normalizer
    methods (Static)
        %% Normalise to [0 1]
        function [normalized, minVal, maxVal] = normalize01(data, minVal, maxVal, noclamp)
            % NORMALIZE01 Normalize data to [0,1] using explicit min/max ranges
            %   [normalized, minVal, maxVal] = normalize01(data, minVal, maxVal)
            %
            % Inputs:
            %   data    - Input array (any numeric type/dimension)
            %   minVal  - Scalar minimum value for normalization
            %   maxVal  - Scalar maximum value for normalization (must be > minVal)
            %   noclamp - Boolean value to flag not to clamp output, clamped by default
            %
            % Output:
            %   normalized - Data scaled to [0,1] based on minVal/maxVal
            %   minVal     - Same as input minVal (for API consistency)
            %   maxVal     - Same as input maxVal (for API consistency)
        
            % Validate inputs
            validateattributes(minVal, {'numeric'}, {'scalar'});
            validateattributes(maxVal, {'numeric'}, {'scalar'});

            if ~exist("noclamp","var")
                noclamp = false;
            end
            
            if maxVal <= minVal
                error('Normalizer:InvalidRange', 'maxVal must be greater than minVal');
            end
        
            % Preserve input data type and shape
            normalized = (data - minVal) ./ (maxVal - minVal);
            
            % clamp values within normalized range unless noclamp=true arg is passed
            if ~noclamp 
                normalized = max(0, min(normalized, 1));
            end

        end
        
        %% Denormalise from [0 1]
        function denormalized = denormalize01(normalized, minVal, maxVal)
            % Denormalize from [0, 1] to original range
            denormalized = normalized * (maxVal - minVal) + minVal;
        end
        
        %% Normalize to [-1, 1]
        function [normalized, minVal, maxVal] = normalize11(data, minVal, maxVal, noclamp)
            % NORMALIZE11 Normalize data to [-1,1] using explicit min/max ranges
            %   [normalized, minVal, maxVal] = normalize11(data, minVal, maxVal)
            %
            % Inputs:
            %   data    - Input array (any numeric type/dimension)
            %   minVal  - Scalar minimum value for normalization
            %   maxVal  - Scalar maximum value for normalization (must be > minVal)
            %   noclamp - Boolean value to flag not to clamp output, clamped by default
            %
            % Output:
            %   normalized - Data scaled to [-1,1] based on minVal/maxVal
            %   minVal     - Same as input minVal (for API consistency)
            %   maxVal     - Same as input maxVal (for API consistency)
        
            % Validate inputs
            validateattributes(minVal, {'numeric'}, {'scalar'});
            validateattributes(maxVal, {'numeric'}, {'scalar'});

            if ~exist("noclamp","var")
                noclamp = false;
            end
            
            if maxVal <= minVal
                error('Normalizer:InvalidRange', 'maxVal must be greater than minVal');
            end
        
            % Preserve input data type and shape
            normalized = 2 * ((data - minVal) ./ (maxVal - minVal)) - 1;
            
            % clamp values within normalized range unless noclamp=true arg is passed
            if ~noclamp 
                normalized = max(-1, min(normalized, 1));
            end
        end

        %% Denormalise from [-1 1]
        function denormalized = denormalize11(normalized, minVal, maxVal)
            % Denormalize from [-1, 1] to original range
            denormalized = ((normalized + 1) / 2) * (maxVal - minVal) + minVal;
        end
    end
end