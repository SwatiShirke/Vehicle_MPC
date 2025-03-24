function z = pendulumDT0(z, u, Ts)
    Ts;
    Nd = 10;
    delta = Ts/Nd;
    for ct=1:Nd
        z = z + delta* pendulumCT0(z, u);
    end

end