function f_sum = longitudinal_forces(params, z, tire_forces, delta)
    F_traction = long_traction_forces( z, tire_forces, delta);
    F_drag =air_drag_force(params, z);
    F_rolling = rolling_resistance(params, z);
    F_gradient = 0;
    f_sum = F_traction - F_drag - F_rolling - F_gradient;

end