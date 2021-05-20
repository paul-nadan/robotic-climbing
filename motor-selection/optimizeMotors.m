function motors = optimizeMotors(torque, bodyMass, mname, mtorque, mmass, mcost)
    motors = ones(size(torque));
    for i = 1:length(mname)
        m = bodyMass + sum(mmass(motors));
        t = torque*m;
        stall = t>mtorque(motors);
        motors(stall) = motors(stall) + 1;
        if ~any(stall)
            break
        end
    end
end