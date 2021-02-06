classdef QwiicServoPHat < matlab_qwiic_i2c.QwiicI2CBase
    % QwiicServoPHat: A class for interfacing with the Qwiic Servo PHat
    % i2c servo conrtoller board
    % See datasheet here: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
    
    properties
        %% Properties
        % Inherited properties:
        % i2c_dev: matlab i2c follower object
        
        frequency
    end

    methods
        %% Constructor
        function obj = QwiicServoPHat(rpi)
            device = i2cdev(rpi, rpi.AvailableI2CBuses{1}, '0x40');
            obj = obj@matlab_qwiic_i2c.QwiicI2CBase(device);
            obj.frequency = 50;

            % Initialize i2c device
            obj = obj.setFrequency(obj.frequency);
        end

        %% Restart the IC
        function restart(self)
            data = readRegister(self.i2c_dev, 0);
            % Note: matlab bitget index starts at 1
            if bitget(data, 7+1) == 1
                data = bitset(data, 4+1, 0);
                writeRegister(self.i2c_dev, 0, data);
                pause(0.1)
            end
            writeRegister(self.i2c_dev, 0, bin2dec('10100000'));
            pause(0.1)
            self.setFrequency(50);
        end
        
        %% Shutdown all servos
        function shutdown(self)
            self.writeWord(0xfc, 0);
        end
        
        %% Get status
        function bin_status = getStatus(self)
            bin_status = dec2bin(readRegister(self.i2c_dev, 0));
        end
        
        %% Configure IC with usual configuration
        % - Make sure that the IC isn't asleep (bit 4)
        % - Turn on auto-increment (bit 5)
        function configure(self)
            data = readRegister(self.i2c_dev, 0);
            % Note: matlab bitget index starts at 1
            if bitget(data, 4+1) == 1 || bitget(data,5+1) == 1
                writeRegister(self.i2c_dev, 0, bin2dec('00100000'))
            end
        end

        %% Move a servo to targeted position
        function setServoPosition(self, channel, position)
            % Double-check settings
            self.configure()
            
            % Calculate the targeted buffer IDs
            % buffer_id_start: The register for configuring the time of the
            % rising edge of the pwm signal
            reg_id_start = 0x06 + 4*channel;

            % buffer_id_end: The register for configuring the time of the
            % falling edge of the pwm signal
            reg_id_end = 0x08 + 4*channel;

            % Write new pwm signal times to target buffers
            self.writeWord(reg_id_start, 0)

            % TODO: Wrap the position so it takes servo angles?
            % Need to figure out the weird behavior related to the ranges
            self.writeWord(reg_id_end, position);
        end

        %% Retrieve a servo's last target position
        function position = getServoPosition(self, channel)
            % Calculate the targeted buffer IDs
            % buffer_id_start: The register for configuring the time of the
            % rising edge of the pwm signal
            reg_id_start = 0x06 + 4*channel;

            % buffer_id_end: The register for configuring the time of the
            % falling edge of the pwm signal
            reg_id_end = 0x08 + 4*channel;
            start_val = self.readWord(reg_id_start);
            end_val = self.readWord(reg_id_end);
            
            position = end_val - start_val;
        end

        %% Set the PWM frequency of the IC
        function self = setFrequency(self, frequency)
            % TODO: calculate frequency command
            %{ 
            From sparkfun qwiic_pca9685.py:
                                          osc_clock
            prescale value = round(----------------------) - 1
                                    (4096 * desired_freq)
            %}
            % 25MHz is the board's built in oscillator frequency
            % TODO (Low priority): Support external oscillators
            osc_clock = 25*10^6; %MHz
            prescale_val = round(osc_clock / (4096 * frequency)) - 1;
            
            % Turn on auto-increment (bit #5) and place in sleep mode (bit
            % #4). The IC needs to be in sleep mode for the frequency to be
            % set
            writeRegister(self.i2c_dev, 0, bin2dec('00110000'))
            pause(0.1)

            % Write frequency to register:
            
            writeRegister(self.i2c_dev, 0xfe, prescale_val) 

            % Wake again
            writeRegister(self.i2c_dev, 0, bin2dec('00100000')) % Turn on 
            pause(0.1)

            % Return the object
            self.frequency = frequency;
        end
    end
end