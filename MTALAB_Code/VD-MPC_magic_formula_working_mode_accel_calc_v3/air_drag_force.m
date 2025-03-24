function f_drag = air_drag_force(params, z)
    Vx = z(1);
    f_drag = 0.5 * params.rho * params.C_d * params.Af * (Vx + params.V_wind)^2;

end