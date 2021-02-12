classdef QwiicServoPHat < matlab_qwiic_i2c.QwiicI2CBase
    % QwiicServoPHat: A class for interfacing with the Qwiic Servo PHat
    % i2c servo conrtoller board
    % See datasheet here: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
    % 
    % TODO: Documentation on basic functions?
    
    properties
        %% Properties
        % Inherited properties:
        % i2c_dev: matlab i2c follower object
        
        frequency
        min_max_times
    end

    methods
        %% Constructor
        function obj = QwiicServoPHat(rpi)
            device = i2cdev(rpi, rpi.AvailableI2CBuses{1}, '0x40');
            obj = obj@matlab_qwiic_i2c.QwiicI2CBase(device);
            obj.frequency = 50;
            
            obj.min_max_times = repelem([1, 2], 16, 1);

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
        
        %% Set servo timing range
        % Default timing range: 1ms - 2ms
        % However
        function self = setServoRange(self, channel, range)
            self.min_max_times(channel+1, :) = range;
        end
        
        %% Get servo timing range
        function range = getServoRange(self, channel)
            range = self.min_max_times(channel+1, :);
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
            
            % Calculate desired servo position:
            % Implementation of the math found in the Sparkfun Qwiic
            % pi_servo_hat.py file
            % The only difference is here our Position is not in angle but
            % as percent in 0 to 1
            
            % Calculate single register value pulse duration:
            % reg_val_duration is the amount of pulse time (secs) that each 
            % increment in register value represents.
            % resolution = period / 4096 (4096 = range of possible pwm values)
            reg_val_duration = 1/self.frequency/4096;
            
            % Min position wave high time (defines min position)
            min_ms = self.min_max_times(channel+1, 1);
            
            % Max position wave high time (defines max position)
            max_ms = self.min_max_times(channel+1, 2);
            
            % Calculate desired time in ms:
            % By default, time range is 1ms - 2ms, with midpoint at 1.5ms.
            % We make this configurable from min_ms - max_ms.
            % The position is a percentage of the total range of
            % milliseconds, so the position time is just those multiplied
            % together, converted to seconds.
            position_time = (position * (max_ms - min_ms) + min_ms); % ms
            position_time = position_time/1000; % Convert to seconds
            
            % Calculate final desired register value:
            % This is the amount of single register value durations that
            % put together the total position_time that we desire.
            % Since we have derived the desired position time in
            % seconds, we can divide to calculate the number of registers
            % needed.
            reg_val_count = round(position_time/ reg_val_duration);

            % Write new pwm signal times to target buffers
            self.writeWord(reg_id_start, 0)
            self.writeWord(reg_id_end, reg_val_count);
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
            
            % Read current pulse stard and end times
            start_val = self.readWord(reg_id_start);
            end_val = self.readWord(reg_id_end);
            
            % Calculate position as a 1-4096 register value
            posn_val = double(end_val - start_val);
            
            % Convert the register value (1-4096) to a percentage of
            % movement range.
            % The inverse of the algorithm implemented in
            % setServoPosition()
            reg_val_duration = 1/self.frequency/4096;
            min_ms = self.min_max_times(channel+1, 1);
            max_ms = self.min_max_times(channel+1, 2);
            time_range = (max_ms - min_ms) / 1000;
            
            % Get time the pulse is set to high, by multiplying the
            % register value by the amount of ms corresponding to the value
            pulse_length = posn_val*reg_val_duration;
            
            % Output final ratio of the pulse length to total range of
            % possible pulse lengths
            position = pulse_length / time_range - 1;
        end

    end
end