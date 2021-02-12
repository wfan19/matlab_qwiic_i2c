classdef QwiicI2CBase
    %QWIIC_I2C_COMPONENT Base class for Qwiic I2C Device
    %   Detailed explanation goes here
    
    properties
        i2c_dev
    end
    
    methods
        %% Constructor
        function obj = QwiicI2CBase(device)
            obj.i2c_dev = device;
        end
        
        %% Reading and writing words
        % WORD: A word is two bytes.
        % When you write a word to an i2c device, those two bytes are
        % actually stored in two separate registers. Thus we need to
        % break up the 16 byte data into two separate bytes - the lower
        % 8, and the upper 8
        
        % Params
        % registers: a scalar or 2-vector of register(s) to set.
            % If scalar: Sets the bytes from the specified register, and 
            % the one immediately after (autoincrement). Assumes the
            % register specified is the lower byte
            
            % if 2-vector: Sets the bytes from the specified registers.
            % Assumes the register vector is ordered low-to-high
        function writeWord(self, registers, data)
            l_registers = length(registers);
            if l_registers > 1
                % 
                error('A word consists of two bytes stored in two registers. %s', ...
                "You are attempting to access %d registers", l_registers);
            elseif l_registers == 1
                % If only one register id is passed in, assume autoincrement
                registers(2) = registers(1) + 1;
            end
            
            % Split 16bit data into 8 bit low and high bytes
            low_byte_mask = 255;
            data_low_byte = bitand(data, low_byte_mask);
            data_hi_byte = bitshift(data, -8);
            
            % Send it!
            writeRegister(self.i2c_dev, registers(1), data_low_byte);
            writeRegister(self.i2c_dev, registers(2), data_hi_byte);
        end
        
        % Get two bytes from two separate registers, and then combine them
        % into one uint16 and output it.
        
        % Params
        % registers: a scalar or 2-vector of register(s) to access.
            % If scalar: Retrieves the bytes from the specified register,
            % and the one immediately after (autoincrement). Assumes the
            % register specified is the lower byte
            
            % if 2-vector: Retrieves the bytes from the specified
            % registers, and combines the two. Assumes the register vector
            % is ordered low-to-high
        function data = readWord(self, registers)
            l_registers = length(registers);
            if l_registers > 1
                % 
                error('A word consists of two bytes stored in two registers. %s', ...
                "You are attempting to access %d registers", l_registers);
            elseif l_registers == 1
                % If only one register id is passed in, assume autoincrement
                registers(2) = registers(1) + 1;
            end
            
            data_low_byte = readRegister(self.i2c_dev, registers(1));
            data_high_byte = readRegister(self.i2c_dev, registers(2));
            
            data = uint16(data_low_byte) + bitshift(uint16(data_high_byte), 8);
        end
        
    end
end

