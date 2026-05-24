clc;
clear;
close all;

portName = "COM3"; 
baudRate = 9600;

s = serialport(portName, baudRate);
configureTerminator(s, "LF"); 
flush(s);

dt = 0.1; 
x = [0; 0; 0]; 
A = [1 dt -0.5*dt^2; 0 1 -dt; 0 0 1];
B = [0.5*dt^2; dt; 0];
H = [1 0 0; 0 1 0];
Q = diag([1e-2, 5e-2, 1e-5]);
R = diag([9, 0.25]); 
P = eye(3) * 50;
I3 = eye(3);

fig = figure('Name', 'Live Car Navigation Dashboard', 'Units', 'normalized', 'OuterPosition', [0 0 1 1], 'Color', 'w');

sp1 = subplot(4,2,1); h1_raw = animatedline('Color','r','Style','none','Marker','.'); h1_kf = animatedline('Color','g','LineWidth',2); title('Altitude / Distance: GPS vs KF (m)'); grid on;
sp2 = subplot(4,2,2); h2_raw = animatedline('Color','b','Style','none','Marker','.'); h2_kf = animatedline('Color','m','LineWidth',2); title('Velocity: GPS vs KF (m/s)'); grid on;
sp3 = subplot(4,2,3); h3 = animatedline('Color','k','LineWidth',1.5); title('Raw Accelerometer (m/s²)'); grid on;
sp4 = subplot(4,2,4); h4 = animatedline('Color',[0.5 0 0.5],'LineWidth',2); title('Estimated Accel Bias (m/s²)'); grid on;
sp5 = subplot(4,2,5); h5 = animatedline('Color','b','LineWidth',1.5); title('Altitude Uncertainty (\sigma_{alt})'); grid on;
sp6 = subplot(4,2,6); h6 = animatedline('Color','g','LineWidth',1.5); title('Velocity Uncertainty (\sigma_{vel})'); grid on;
sp7 = subplot(4,2,7); h7 = animatedline('Color','r','LineWidth',1.5); title('Innovation (GPS vs KF Expectation)'); grid on;
sp8 = subplot(4,2,8); h8 = animatedline('Color','k','LineWidth',2); title('Trajectory Phase Portrait'); xlabel('Distance (m)'); ylabel('Velocity (m/s)'); grid on;

startTime = datetime('now');

while ishandle(fig)
    try
        line = readline(s);
        if isempty(line), continue; end
        
        data = str2double(split(line, ','));
        
        if length(data) == 3 && ~any(isnan(data))
            accel = data(1);
            gps_alt = data(2);
            gps_vel = data(3);
            
            t_now = seconds(datetime('now') - startTime);
            
            x = A*x + B*accel;
            P = A*P*A' + Q;
            
            innov = [gps_alt; gps_vel] - H*x;
            K = P*H' / (H*P*H' + R);
            x = x + K*innov;
            P = (I3 - K*H)*P*(I3 - K*H)' + K*R*K';
            
            addpoints(h1_raw, t_now, gps_alt); addpoints(h1_kf, t_now, x(1));
            addpoints(h2_raw, t_now, gps_vel); addpoints(h2_kf, t_now, x(2));
            addpoints(h3, t_now, accel);
            addpoints(h4, t_now, x(3));
            addpoints(h5, t_now, sqrt(P(1,1)));
            addpoints(h6, t_now, sqrt(P(2,2)));
            addpoints(h7, t_now, innov(1));
            addpoints(h8, x(1), x(2));
            
            drawnow limitrate;
        end
    catch
        continue;
    end
end