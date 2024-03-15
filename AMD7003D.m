classdef AMD7003D
    properties (Access = private)

        deviceParams
        destination_ip;
        destination_port;
        udp_socket;

        deviceData
        temperature;
        acceleration_X;
        acceleration_Y;
        acceleration_Z;
        dt;
        encoder;
    end

    %Public Methods
    methods (Access = public)

        % Constructor for AMD7003D
        function obj = AMD7003D(destination_ip, destination_port)
            obj.deviceParams = struct( ...
                'destination_ip', destination_ip, ...
                'destination_port', destination_port, ...
                'udp_socket',udpport('Timeout',1));
            obj.deviceData = struct( ...
                'temperature',[] ...
                ,'acceleration_X',[] ...
                ,'acceleration_Y',[] ...
                ,'acceleration_Z',[] ...
                ,'dt',[] ...
                ,'encoder',[]);
        end

        % Show Device Parameters
        function getParams(obj)
            obj.deviceData
        end

        % Read Connected Device Data
        function [ACCX,ACCY,ACCZ,DT,ENCODER,TEMPERATURE] = readData(obj)
            data        = read(obj.deviceParams.udp_socket(),100,"string");
            data        = obj.parseData(data);
            ACCX        = data.acceleration_X;
            ACCY        = data.acceleration_Y;
            ACCZ        = data.acceleration_Z;
            DT          = data.dt;
            ENCODER     = data.encoder;
            TEMPERATURE = data.temperature;
        end

        % Connect Device
        function [status ,error_message] = connect(obj)
            disp('Waiting for connection!');
            status = obj.chechkTheConnection();
            if status == 0
                error_message = 'Connection is succesful';
                %disp(error_message)
                message = uint8(2);
                write(obj.deviceParams.udp_socket,message,"uint8",obj.deviceParams.destination_ip,obj.deviceParams.destination_port);
            else
                error_message = 'Connection failed';
                %disp(error_message)
            end
        end

        % Write Data
        function writeData(obj,data)
            message = uint8([3 data]);
            write(obj.deviceParams.udp_socket ...
                ,message ...
                ,"uint8" ...
                ,obj.deviceParams.destination_ip ...
                ,obj.deviceParams.destination_port);
            %fprintf('%f setted to AMD.\n',data);
        end

    end
    methods (Access = private)

        % Parse Data
        function updatedStruct = parseData(obj,data)
            raw_data = char(data);
            data = uint8(raw_data(12:end));
            data = char(data);
            data = (strsplit(data,'\n'));
            data = uint8(data{2});
            array_size = size(data);
            if array_size(2) == 72
                obj.deviceData.temperature      = typecast(data(45:48),'single');
                obj.deviceData.acceleration_Z   = typecast(data(41:44),'single');
                obj.deviceData.acceleration_Y   = typecast(data(37:40),'single');
                obj.deviceData.acceleration_X   = typecast(data(33:36),'single');
                obj.deviceData.dt               = typecast(data(29:32),'single');
                % encoder datasından emin değilim ??
                obj.deviceData.encoder          = typecast(data(15:18),'single');
                updatedStruct = obj.deviceData;
            else
                obj.deviceData.temperature      = 0;
                obj.deviceData.acceleration_Z   = 0;
                obj.deviceData.acceleration_Y   = 0;
                obj.deviceData.acceleration_X   = 0;
                obj.deviceData.dt               = 0;
                % encoder datasından emin değilim ??
                obj.deviceData.encoder          = 0;
                updatedStruct = obj.deviceData;
            end

        end

        %Ping to Device
        function err = chechkTheConnection(obj)
            ip_address = obj.deviceParams.destination_ip;
            command = sprintf('ping -n 2 %s' ,ip_address);
            [~ , result] = system(command);
            err = contains(result,'Destination host unreachable');
        end

    end

end
