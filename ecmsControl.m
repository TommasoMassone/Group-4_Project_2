
function [engSpd, engTrq] = ecmsControl(SOC, vehSpd, vehAcc, eqFactor, veh)
    
w_idle = veh.eng.idleSpd; % Engine idle speed
w_max = veh.eng.maxSpd; % Maximum engine speed

w_speed = [0 linspace(w_idle, w_max, 10)]; % Speed range of 1000 values for calculations from w_min to w_max  

T_max = max(veh.eng.maxTrq(w_speed));

T_eng = linspace(0, T_max, 11);

const = ((veh.batt.nomEnergy)/veh.eng.fuelLHV) * 1000 * 3600;

dt = 1; 

fuelFlwRate_eq = zeros(length(w_speed), length(T_eng));


for i = 1: length(w_speed)

    for n = 1: length(T_eng)

         [SOC_next, fuelFlwRate, unfeas] = hev_model(SOC, [w_speed(i), T_eng(n)], [vehSpd, vehAcc], veh);

         if unfeas == 0

            unfeas =  ( SOC_next > 0.8 ) | ( SOC_next < 0.4 );
         
         end

         if unfeas == 1

             fuelFlwRate_eq(i, n) = inf;

         else

            fuelFlwRate_eq(i, n) = fuelFlwRate - eqFactor * const * (SOC_next-SOC)/dt;

         end
    end
end

[min_val, idx] = min(fuelFlwRate_eq, [], 'all');

min_fuelFlwRate_eq = min_val;

[a,b] = ind2sub(size(fuelFlwRate_eq), idx);

engSpd = w_speed(a);
engTrq = T_eng(b);
end