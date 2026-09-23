%% PID_CALC.M - 严格按照下位机 pid.c 实现的离散 PID 计算函数
% 对应源码: Crane-down/IMU/components/controller/pid.c 中的 PID_calc()

function [out, pid_state, out_unsat] = pid_calc(pid_state, ref, set)
    % ref: 实际测量反馈 (例如当前编码器读数或当前转速 rpm)
    % set: 设定目标值 (例如目标编码器读数或目标转速 rpm)
    
    % 更新历史误差缓存
    pid_state.error(3) = pid_state.error(2);
    pid_state.error(2) = pid_state.error(1);
    pid_state.set = set;
    pid_state.fdb = ref;
    pid_state.error(1) = set - ref;
    
    % 比例项
    Pout = pid_state.Kp * pid_state.error(1);
    
    % 积分项与积分抗饱和限幅
    pid_state.Iout = pid_state.Iout + pid_state.Ki * pid_state.error(1);
    if pid_state.Iout > pid_state.max_iout
        pid_state.Iout = pid_state.max_iout;
    elseif pid_state.Iout < -pid_state.max_iout
        pid_state.Iout = -pid_state.max_iout;
    end
    
    % 微分项 (基于前后两次误差差分)
    pid_state.Dbuf(3) = pid_state.Dbuf(2);
    pid_state.Dbuf(2) = pid_state.Dbuf(1);
    pid_state.Dbuf(1) = (pid_state.error(1) - pid_state.error(2));
    Dout = pid_state.Kd * pid_state.Dbuf(1);
    
    % 输出综合与主输出限幅
    out_unsat = Pout + pid_state.Iout + Dout;
    out = out_unsat;
    if out > pid_state.max_out
        out = pid_state.max_out;
    elseif out < -pid_state.max_out
        out = -pid_state.max_out;
    end
    
    pid_state.out = out;
    pid_state.out_unsat = out_unsat;
end
