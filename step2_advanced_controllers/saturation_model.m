%% SATURATION_MODEL.M - 执行器电流硬限幅、平滑逼近与残差计算模块
% =========================================================================
% 本模块处理 M3508 电流通道的物理约束：
% 1. 真实执行器硬限幅: sat(i, Imax)
% 2. 饱和残差向量计算: Delta_i = i_sat - i_ideal (用于抗饱和补偿)
% 3. 平滑逼近函数 rho(i) 与中值定理因子 Xi (为 C2b 预留标准接口)
% =========================================================================

classdef saturation_model
    methods (Static)
        %% 1. 执行器电流硬限幅与残差
        function [iL_sat, iR_sat, Delta_i, is_sat] = hard_sat(iL, iR, Imax)
            % iL, iR: 理想电流指令 (counts)
            % Imax: CAN 电流控制指令上限 (counts, 标称 16000)
            
            iL_sat = max(-Imax, min(Imax, iL));
            iR_sat = max(-Imax, min(Imax, iR));
            
            % 饱和残差: 实际执行量 - 理想请求量
            Delta_i = [iL_sat - iL; iR_sat - iR];
            
            % 饱和标志
            is_sat = (abs(iL) >= Imax) || (abs(iR) >= Imax);
        end
        
        %% 2. 平滑饱和逼近 rho(i) 与中值因子 Xi (工程近似 zeta = 1)
        function [rho_i, Xi] = smooth_sat(i_cmd, Imax)
            % rho(i) = 2*Imax/pi * atan(pi*i / (2*Imax))
            rho_i = (2.0 * Imax / pi) * atan((pi * i_cmd) / (2.0 * Imax));
            
            % 中值定理工程可计算近似 (zeta = 1)
            % Xi = 1 - 1 / [1 + (pi*i / (2*Imax))^2]
            ratio = (pi * i_cmd) / (2.0 * Imax);
            Xi = 1.0 - 1.0 / (1.0 + ratio^2);
        end
    end
end
