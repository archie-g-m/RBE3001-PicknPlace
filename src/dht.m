function M = dht(d_, t_, a_, alp_)
    M = [
    cos(t_), -sin(t_)*cos(alp_), sin(t_)*sin(alp_), a_*cos(t_);
    sin(t_), cos(t_)*cos(alp_), -cos(t_)*sin(alp_), a_*sin(t_);
    0, sin(alp_), cos(alp_), d_;
    0,0,0,1
    ];
end