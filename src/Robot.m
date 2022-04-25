classdef Robot < handle

    properties
        %hidDevice;
        %hidService;
        myHIDSimplePacketComs
        pol
        trajPlanner = Traj_Planner();

    end

    properties (SetAccess = private)
        goal = [0, 0, 0]
    end

    methods
        %The is a shutdown function to clear the HID hardware connection
        function shutdown(packet)
            %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end

        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
            packet.myHIDSimplePacketComs = dev;
            packet.pol = java.lang.Boolean(false);
        end

        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
            com = zeros(15, 1, 'single');

            try
                ds = javaArray('java.lang.Double', length(values));

                for i = 1:length(values)
                    ds(i) = java.lang.Double(values(i));
                end

                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeFloats(intid, ds);
                ret = packet.myHIDSimplePacketComs.readFloats(intid);

                for i = 1:length(com)
                    com(i) = ret(i).floatValue();
                end

                %class(com)
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end

        end

        function com = read(packet, idOfCommand)
            com = zeros(15, 1, 'single');

            try

                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                ret = packet.myHIDSimplePacketComs.readFloats(intid);

                for i = 1:length(com)
                    com(i) = ret(i).floatValue();
                end

                %class(com)
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end

        end

        function write(packet, idOfCommand, values)

            try
                ds = javaArray('java.lang.Double', length(values));

                for i = 1:length(values)
                    ds(i) = java.lang.Double(values(i));
                end

                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeFloats(intid, ds, packet.pol);

            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end

        end

        function gripperOpen(self)
            GRIPPER_ID = 1962;

            somePacket = javaArray('java.lang.Byte', 1);
            % somePacket(1) = java.lang.Byte(0);
            somePacket(1) = java.lang.Byte(180);
            

            try
                % disp("ATTEMPTING SERVO");
                packetID = java.lang.Integer(GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(packetID, somePacket, self.pol);
            catch exception
                getReport(exception);
                disp("DIDNT WORK")
            end

            pause(0.25);

        end

        function gripperClose(self)
            GRIPPER_ID = 1962;

            somePacket = javaArray('java.lang.Byte', 1);
            somePacket(1) = java.lang.Byte(180);

            try
                % disp("ATTEMPTING SERVO");

                for i = 0:180
                    % somePacket(1) = java.lang.Byte(i); % FLIP 1
                    somePacket(1) = java.lang.Byte(180-i); % FLIP 2

                    packetID = java.lang.Integer(GRIPPER_ID);
                    self.myHIDSimplePacketComs.writeBytes(packetID, somePacket, self.pol);
                    % pause(0.1)
                end

            catch exception
                getReport(exception)
                disp("DIDNT WORK")
            end

            pause(0.25);
        end

        % send joint values directly to servos without interpolation
        function servojp(self, jointValues)
            %SERVOJP takes a 1x3 array of joint values in degrees to be sent directly to actuators w/o interpolation
            %Define Server ID to send Motor values
            self.interpolate_jp(jointValues, 0);
        end

        function interpolate_jp(self, jointValues, interpolationTime)
            % arguments
            %     interpolationTime (1,:) double = 0
            % end
            %disp(jointValues)
            %SERVOJP takes a 1x3 array of joint values in degrees to be sent directly to actuators w/o interpolation
            %Define Server ID to send Motor values
            SERV_ID = 1848;
            SERVER_ID_READ = 1910;
            DEBUG = true;

            apacket = zeros(15, 1, 'single');
            apacket(1) = interpolationTime; %zero second time (ignore interpolation)
            apacket(2) = 0; %linear interpolation
            apacket(3) = jointValues(1); % Set first link
            apacket(4) = jointValues(2); % Set second link
            apacket(5) = jointValues(3); % Set third link

            self.goal = [jointValues(1), jointValues(2), jointValues(3)];

            % Send packet to the server and get the response
            % write sends a 15 float packet to the micro controller
            self.write(SERV_ID, apacket);
            % read reads a returned 15 float backet from the micro controller.
            returnPacket = self.read(SERVER_ID_READ);
            % Debug statements
            if DEBUG
                % disp('Sent Packet:');
                % disp(apacket);
                % disp('Received Packet:');
                % disp(returnPacket);
            end

        end

        function servo_vel(self, q, q_dot, dur)
            target_pos = q + (q_dot * (dur * 1000));
            self.interpolate_jp(target_pos, dur * 1000);
        end

        function returnmat = setpoint_js(self)
            SERVER_ID_READ = 1910;

            returnmat = zeros(1, 3);
            returnPacket = self.read(SERVER_ID_READ);
            returnmat(1) = returnPacket(2);
            returnmat(2) = returnPacket(4);
            returnmat(3) = returnPacket(6);
        end

        function transform = setpoint_cp(self)
            transform = self.fk3001(self.setpoint_js());
        end

        function goal = goal_js(self)
            goal = self.goal;
        end

        function goal_transform = goal_cp(self)
            goal_transform = self.fk3001(self.goal_js());
        end

        function getData = measured_js(self, GETPOS, GETVEL)
            %% param: GETPOS [bool] Whether to get position data
            %% param: GETVEL [bool] WHether to get velocity data
            GETPOSSET_ID = 1910;
            GETVEL_ID = 1822;
            DEBUG = false;

            getData = zeros(2, 3, 'single');

            if GETPOS
                pos_read = self.read(GETPOSSET_ID);
                getData(1, 1) = pos_read(3);
                getData(1, 2) = pos_read(5);
                getData(1, 3) = pos_read(7);
            end

            if GETVEL
                vel_read = self.read(GETVEL_ID);
                getData(2, 1) = vel_read(3);
                getData(2, 2) = vel_read(6);
                getData(2, 3) = vel_read(9);
            end

            if DEBUG
                disp(getData)
            end

        end

        function tf = measured_cp(self)
            position = self.measured_js(true, false);
            converted_position = deg2rad(position);
            tf = self.fk3001(converted_position(1, :));
        end

        %% TAKES IN DEG
        function tf = measured_any_cp(self, q)
            converted_position = deg2rad(q);
            tf = self.fk3001(converted_position(1, :));
        end

        function tip = measured_tip(self)
            tip = zeros(1, 3);
            tf = self.measured_cp();
            tip(1) = tf(1, 4);
            tip(2) = tf(2, 4);
            tip(3) = tf(3, 4);
        end

        function tip = measured_anytip(self, q)
            tip = zeros(1, 3);
            tf = self.measured_any_cp(q);
            tip(1) = tf(1, 4);
            tip(2) = tf(2, 4);
            tip(3) = tf(3, 4);
        end

        function arrived = atGoal(self, values)
            threshold = 3; % threshold to match setpoints in degrees
            thisGoal = self.goal_js();
            arrived = abs(values(1) - thisGoal(1)) < threshold & abs(values(2) - thisGoal(2)) < threshold & abs(values(3) - thisGoal(3)) < threshold;
        end

        function transform = dh2mat(self, dh)
            d = dh(1);
            th = dh(2);
            a = dh(3);
            al = dh(4);
            transform = [
                    cos(th), -sin(th) * cos(al), sin(th) * sin(al), a * cos(th);
                    sin(th), cos(th) * cos(al), -cos(th) * sin(al), a * sin(th);
                    0, sin(al), cos(al), d;
                    0, 0, 0, 1
                    ];
        end

        function fk = dh2fk(self, dh_table)
            num_rows = size(dh_table, 1);
            fk = eye(4);

            for i = 1:num_rows
                Ti = self.dh2mat(dh_table(i, :));
                fk = fk * Ti;
            end

        end

        function tip = fk3001(self, joint_values)
            q1 = joint_values(1);
            q2 = -joint_values(2);
            q3 = -joint_values(3);
            % disp(joint_values);

            tip = [cos(q1) * cos(q2 + pi / 2) * cos(q3 - pi / 2) - cos(q1) * sin(q2 + pi / 2) * sin(q3 - pi / 2), sin(q1), cos(q1) * cos(q2 + pi / 2) * sin(q3 - pi / 2) + cos(q1) * cos(q3 - pi / 2) * sin(q2 + pi / 2), 100 * cos(q1) * cos(q2 + pi / 2) - 100 * cos(q1) * sin(q2 + pi / 2) * sin(q3 - pi / 2) + 100 * cos(q1) * cos(q2 + pi / 2) * cos(q3 - pi / 2);
                cos(q2 + pi / 2) * cos(q3 - pi / 2) * sin(q1) - sin(q1) * sin(q2 + pi / 2) * sin(q3 - pi / 2), -cos(q1), cos(q2 + pi / 2) * sin(q1) * sin(q3 - pi / 2) + cos(q3 - pi / 2) * sin(q1) * sin(q2 + pi / 2), 100 * cos(q2 + pi / 2) * sin(q1) + 100 * cos(q2 + pi / 2) * cos(q3 - pi / 2) * sin(q1) - 100 * sin(q1) * sin(q2 + pi / 2) * sin(q3 - pi / 2);
                cos(q2 + pi / 2) * sin(q3 - pi / 2) + cos(q3 - pi / 2) * sin(q2 + pi / 2), 0, sin(q2 + pi / 2) * sin(q3 - pi / 2) - cos(q2 + pi / 2) * cos(q3 - pi / 2), 100 * sin(q2 + pi / 2) + 100 * cos(q2 + pi / 2) * sin(q3 - pi / 2) + 100 * cos(q3 - pi / 2) * sin(q2 + pi / 2) + 95;
                0, 0, 0, 1];
        end

        function q_dot = idk3001(self, x_dot, q)
            q = deg2rad(q);
            J = self.jacob3001(q);
            J_inv = pinv(J(1:3, :));
            q_dot = J_inv * transpose(x_dot);
            q_dot = transpose(q_dot);
        end

        function target_q = ik_3001_numerical(self, curr_q, goal_pos)
            p = 2;
            % Calculate target direction
            fk = self.fk3001(deg2rad(curr_q));
            curr_pos = transpose(fk(1:3, 4));
            direction = goal_pos - curr_pos;

            target_vel = direction

            %% Get Joint velocities (inverse differential kinematics)
            q_dot = self.idk3001(target_vel, curr_q) * p

            target_q = curr_q + q_dot;
        end

        function jacob = jacob3001(self, joint_values)
            q1 = joint_values(1);
            q2 = joint_values(2);
            q3 = joint_values(3);

            jacob = [-100 * sin(q1) * (sin(q2) + cos(q2 + q3)), 100 * cos(q1) * (cos(q2) - sin(q2 + q3)), 100 * cos(q1) * (-sin(q2 + q3));
                100 * cos(q1) * (sin(q2) + cos(q2 + q3)), 100 * sin(q1) * (cos(q2) - sin(q2 + q3)), 100 * sin(q1) * (-sin(q2 + q3));
                0, -100 * (sin(q2) + cos(q2 + q3)), -100 * cos(q2 + q3);
                0, -sin(q1), -sin(q1);
                0, cos(q1), cos(q1);
                1, 0, 0];

        end

        function p_dot = fdk3001(self, q, q_dot)
            J = self.jacob3001(q);

            p_dot = J * q_dot;
        end

        function p_dot = getFDK(self)
            arm_data = self.measured_js(true, true);
            q = arm_data(1, :);
            q_dot = arm_data(2, :);

            p_dot = self.fdk3001(self, q, q_dot);
        end

        %% THIS TAKES IN DEGREES
        function safe = check_safety(self, qdeg)
            threshold = 300;
            qrad = deg2rad(qdeg);
            J = self.jacob3001(qrad);
            J_top = J(1:3, :);

            J_det = det(J_top);

            safe = J_det < threshold;
        end

        function cubic_cp(self, position, time)

            planner = self.trajPlanner;

            vel = [0 0 0];
            interpolation_time = time;

            resolution = time / 100;

            % disp("Moving from")
            start_point = self.measured_tip();
            % plot_arm(start_point, [0 0 0]);
            % disp('to')
            % disp(position)

            t = 0:resolution:interpolation_time;

            goal_point = position;
            path_points = planner.quintic_traj(t, vel, vel, start_point, goal_point);

            % figure(point)
            % plot(t,path_points,'LineWidth',6)

            t0 = clock;

            i = 0;

            while etime(clock, t0) < interpolation_time
                curr_index = round(etime(clock, t0) / resolution) + 1;
                curr_pos_n_time = path_points(:, curr_index);
                curr_pos = curr_pos_n_time(2:4, :);
                % disp(currpos)
                ik_curpos = ik(curr_pos);
                self.servojp(ik_curpos);

                % i = i + 1;

                % if (mod(i, 10))
                %     % values = ik_curpos;
                %     allValues = self.measured_js(true, false);
                %     values = allValues(1, :);
                %     %     disp(pp.fk3001(values))
                %     % disp(pp.measured_cp())
                %     % plot_arm(values, [0 0 0])
                % end

            end

            curr_pos_n_time = path_points(:, end);
            curr_pos = curr_pos_n_time(2:4, :);
            ik_curpos = ik(curr_pos);
            self.servojp(ik_curpos);
            pause(0.5);

        end

    end

end
