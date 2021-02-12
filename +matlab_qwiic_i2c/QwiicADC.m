classdef QwiicADC < matlab_qwiic_i2c.QwiicI2CBase
    %QWIICADC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        %% Constructor
        function obj = QwiicADC(rpi)
            device = i2cdev(rpi, rpi.AvailableI2CBuses{1}, '0x48');
            obj = obj@matlab_qwiic_i2c.QwiicI2CBase(device);

            % Initialize I2C here
            
        end
        
        %% Read funciton
        function value = getSingleVal(channel)
            
        end
    end
end

