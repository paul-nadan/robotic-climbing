function [name, torque, mass, cost] = loadMotors
    T = readtable('DynamixelServos.xlsx', 'VariableNamingRule', 'preserve');
    T = sortrows(T, 'Var8', 'descend');
    T = sortrows(T, 'Price');
    T = sortrows(T, 'Weight [g]');
    name = T{:,'Product'};
    torque = T{:,'Var8'};
    cost = T{:,'Price'};
    mass = T{:,'Weight [g]'}/1000;
end