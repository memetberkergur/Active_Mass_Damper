clear;
clc;

deviceIp = "192.168.2.246";
devicePort = 9036;

AMD1=AMD7003D(deviceIp,devicePort);

[status, message] = AMD1.connect();

% Filter Parameters
Fs = 250;
low_fc = 0.2;
high_fc = 10;
order = 3;
[b,a] = butter(order,[low_fc,high_fc]/(Fs/2),'bandpass');

% PID Parameters
Kp = 1;
Ki = 100;




if ~status
    disp(message)

    control_flag = true;
    
    current_value = [];
    time_buffer = [];
    velocity_signal = 0;

    
    % Plot Parameters 
    subplot(3,1,1);
    acc_line = animatedline;
    subplot(3,1,2)
    vel_line = animatedline;
    subplot(3,1,3)
    cont_line = animatedline;
    index = 0;

    while ishandle(vel_line)
        [ACCX,~,~,DT,~,~] = AMD1.readData();
        if ~DT == 0

            index = index + 1;
            current_value(end + 1) = ACCX;
            time_buffer (end + 1) = DT*index;

            if length(current_value) < 2500 & length(current_value) > order

                acc_signal_with_filter = filter(b,a,current_value);
                addpoints(acc_line,time_buffer(end),acc_signal_with_filter(end));
                velocity_signal = velocity_signal + acc_signal_with_filter(end)*DT;
                addpoints(vel_line,time_buffer(end),velocity_signal(end));
                drawnow limitrate;

            elseif length(current_value) >= 2501
                
                current_value(1) = [];
                time_buffer(1) = [];
                
                
                acc_signal_with_filter = filter(b,a,current_value);
                addpoints(acc_line,time_buffer(end),acc_signal_with_filter(end));
                %xlim(subplot(3,1,1),[time_buffer(1), time_buffer(2500)]);

                % Integration
                velocity_signal = velocity_signal + acc_signal_with_filter(end)*DT;
                velocity_signal(end);
                addpoints(vel_line,time_buffer(end),velocity_signal(end));
                %xlim(subplot(3,1,2),[time_buffer(1), time_buffer(2500)]);

                if control_flag == true

                    %control_signal = control_loop(integral_value);
                    set_velocity = 0;
                    errror_signal = (Kp*acc_signal_with_filter(end)) + (Ki*velocity_signal);
                    
                    % Its PI control
                    control_signal = set_velocity - errror_signal;
                    
                    % Added line
                    addpoints(cont_line,time_buffer(end),control_signal);

                    % Set calculated control signal 
                    %AMD1.writeData(control_signal);

                    %xlim(subplot(3,1,3),[time_buffer(1), time_buffer(2500)]);
                    %fprintf('\nFiltered ACC : %.7f Velocity : %.7f Control Signal : %.7f ',acc_signal_with_filter(end),velocity_signal(end),control_signal);
                end
                drawnow limitrate;

            end

        end
    end
else
    disp(message)
    delete(AMD1)
end