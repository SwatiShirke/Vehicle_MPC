function dz = vehicle_control(t ,z, u, nlobj, yref)
    [u1, info] = nlmpcmove(nlobj, z, u, yref, []);
    dz = vehicle_dynamics(z, u1);  % Use Ts as dt

end