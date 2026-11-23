function [u, pid] = pid_diskrit(e, de, dt, Kp, Ki, Kd, pid, k)
%PID_DISKRIT  Salinan persis hitungPID() firmware.
%   pid.integral : I(k). Di-reset saat tanda e berganti, dibatasi +/-INTEGRAL_MAX.
%   u = Kp*e + Ki*I + Kd*de, disaturasi +/-OUTPUT_MAX.
if (e > 0 && pid.integral < 0) || (e < 0 && pid.integral > 0)
    pid.integral = 0;
end
pid.integral = pid.integral + e * dt;
pid.integral = min(max(pid.integral, -k.INTEGRAL_MAX), k.INTEGRAL_MAX);
pid.P = Kp * e;
pid.I = Ki * pid.integral;
pid.D = Kd * de;
u = min(max(pid.P + pid.I + pid.D, -k.OUTPUT_MAX), k.OUTPUT_MAX);
pid.u = u;
end
