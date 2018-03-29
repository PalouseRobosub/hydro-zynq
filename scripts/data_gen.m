% Specifies the frequency of the ping in Hz.
freq = [30000, 25000, 40000, 35000];
start_time = [0.0, 0.5, 1.0, 1.4];

% Specifies the sampling frequency of the analog signal
sample_freq = 5000000;

% Calculate the sampling period (time step).
sample_period = 1/sample_freq;

% Specifies the length of the sample in total (seconds).
sample_len = 2.0;

% Specifies the maximum ADC reading for the sample (AKA 3.3V).
max_val = 2^14;

output = zeros([4, length([0:sample_period:sample_len])]);

for i = 1:length(freq)
    
    % Specifies how much data channel 0-3 will be left shifted (in seconds)
    % from the reference
    left_shift_1 = -.000001;
    left_shift_2 = -.000002;
    left_shift_3 = -.000003;
    
    % Calculate the frequency in terms of omega.
    omega = 2 * pi * freq(i);
    
    % Generate the discrete time steps.
    shift = start_time(i);
    t = [0:sample_period:sample_len];
    
    % Generate the windowing functions for each data channel.
    window = heaviside(t - 0.002) .* heaviside(0.004 - t);
    window_1 = heaviside(t - left_shift_1 - .002) .* heaviside(0.004 - (t - left_shift_1));
    window_2 = heaviside(t - left_shift_2 - .002) .* heaviside(0.004 - (t - left_shift_2));
    window_3 = heaviside(t - left_shift_3 - .002) .* heaviside(0.004 - (t - left_shift_3));
    
    % Calculate the pings by creating a decaying sinusoid, windowed and scaled up.
    x = (max_val / 2) .* ((sin(omega .* t).* exp(-1000*t) .* window));
    x1 = (max_val / 2) .* ((sin(omega .* (t - left_shift_1)) .* exp(-1000*(t - left_shift_1)) .* window_1));
    x2 = (max_val / 2) .* ((sin(omega .* (t - left_shift_2)) .* exp(-1000*(t - left_shift_2)) .* window_2));
    x3 = (max_val / 2) .* ((sin(omega .* (t - left_shift_3)) .* exp(-1000*(t - left_shift_3)) .* window_3));
    
    % Shift the signal right by the required number of seconds.
    samples_to_shift = start_time(i) * sample_freq;
    x = [x(length(x) - samples_to_shift + 1:length(x)), x(1:length(x) - samples_to_shift)];
    x1 = [x1(length(x1) - samples_to_shift + 1:length(x1)), x1(1:length(x1) - samples_to_shift)];
    x2 = [x2(length(x2) - samples_to_shift + 1:length(x2)), x2(1:length(x2) - samples_to_shift)];
    x3 = [x3(length(x3) - samples_to_shift + 1:length(x3)), x3(1:length(x3) - samples_to_shift)];
    
    fprintf(1, 'Frequency %d done\n', freq(i))
    
    output(1, :) = output(1, :) + x;
    output(2, :) = output(2, :) + x1;
    output(3, :) = output(3, :) + x2;
    output(4, :) = output(4, :) + x3;
end

output(1, :) = output(1, :) + max_val / 2;
output(2, :) = output(2, :) + max_val / 2;
output(3, :) = output(3, :) + max_val / 2;
output(4, :) = output(4, :) + max_val / 2;

plot(t, output(1, :), t, output(2, :), t, output(3, :), t, output(4, :));
legend('R', 'X', 'Y', 'Z');

% Save the waveforms to a file for loading into emulation.
file_out = fopen('data.csv', 'w');

fprintf(file_out, 'Sample number, C1, C2, C3, C4\n');

h = waitbar(0, 'Writing to file.');
for i = 1:length(output(1, :))
    if mod(i, 10000) == 0
        waitbar(i/length(output(1,:)), h);
    end
    
    fprintf(file_out, '%d, %d, %d, %d, %d\n', i, output(1, i), output(2, i), output(3, i), output(4, i));
end
fclose(file_out);