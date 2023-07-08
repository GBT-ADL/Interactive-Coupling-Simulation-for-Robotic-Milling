classdef Robot_GCC < handle
    % This class is adopted to calculate Robot_Gravity_Compensation
    % * calculate gravity compensation based on given q2
    % * [q2] is namely the generalized coordinate of joint 2
    % * Gravity Compensation is constructed by [limited Fourier series], 
    %   vector [B] stores coefficients of [limited Fourier series].

    properties
        B
    end

    methods
        function obj = Robot_GCC(B)
            % Determine coefficients of [limited Fourier series]
            % input [B] coefficients of [limited Fourier series],[row,scalar]

            % Default [B] definition
            if (nargin<1),B = [-7938.1 -72.8 257.1]; end 
            % input check
            if ~isrow(B),error('Error: Wrong input type');end

            obj.B = B;
        end

        function [Gravity_Compensation] = Robot_Gravity_Compensation(obj,q2)
            % This function can compute gravity compensation based on given q2
            % input:[q2] angular deflection on joint 2, [scalar]
            % output:[Gravity_Compensation], Gravity compensation, [col,N.m]
            % input check
            if ~isscalar(q2)
                error('Error: Wrong input type')
            end
            % function body
            G_compensation_2 = 0;
            for i = 1:3
                G_compensation_2 = obj.B(i)*cos(i*q2);
            end
            Z = zeros(1,6);
            Z(2) = G_compensation_2;
            Gravity_Compensation = Z.';
        end
    end
end