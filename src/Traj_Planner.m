classdef Traj_Planner

    properties
    end

    methods

        function obj = Traj_Planner(obj)
        end

        function traj = cubic_traj(obj, dur, v0, vf, p0, pf)
            traj = obj.poly_traj(true, dur, v0, vf, p0, pf);
        end

        function traj = quintic_traj(obj, dur, v0, vf, p0, pf)
            traj = obj.poly_traj(false, dur, v0, vf, p0, pf);
        end

        function traj = poly_traj(obj, cubic, dur, v0, vf, p0, pf)
            % traj = zeros(3,size(dur,1));
            traj = dur;
            the_size = max(size(v0));

            for i = 1:the_size

                if (cubic)
                    coefs = obj.cubic_coefs(dur(1), dur(end), v0(i), vf(i), p0(i), pf(i));
                    fn = obj.cubic_fn(coefs, dur);
                else
                    coefs = obj.quintic_coefs(dur(1), dur(end), v0(i), vf(i), p0(i), pf(i));
                    fn = obj.quintic_fn(coefs, dur);
                end

                % row = i+1;
                traj = [traj; fn];
            end

        end

        function coefficients = cubic_coefs(obj, t0, tf, v0, vf, p0, pf)

            A = [1, t0, t0^2, t0^3;
                0, 1, 2 * t0, 3 * (t0^2);
                1, tf, tf^2, tf^3;
                0, 1, 2 * tf, 3 * (tf^2)];

            b = [p0; v0; pf; vf]; % p vector of a given stage

            x = A \ b; % Solve for a coefficients

            coefficients = x.';
        end

        function fn = cubic_fn(obj, a, values)
            fn = a(1) + a(2) * values + a(3) * values.^2 + a(4) * values.^3;
        end

        function coefficients = quintic_coefs(obj, t0, tf, v0, vf, p0, pf)

            A = [   1,  t0, t0^2,   t0^3,       t0^4,       t0^5;
                    0,  1,  2*t0,   3*(t0^2),   4*(t0^3),   5*(t0^4);
                    0,  0,  2,      6*t0,       12*(t0^2),  20*(t0^3);
                    1,  tf, tf^2,   tf^3,       tf^4,       tf^5;
                    0,  1,  2*tf,   3*(tf^2),   4*(tf^3),   5*(tf^4);
                    0,  0,  2,      6*tf,       12*(tf^2),  20*(tf^3)];

            b = [p0; v0; 0; pf; vf; 0]; % p vector of a given stage

            x = A \ b; % Solve for a coefficients

            coefficients = x.';
        end

        function fn = quintic_fn(obj, a, values)
            fn = a(1) + a(2) * values + a(3) * values.^2 + a(4) * values.^3 + a(5) * values.^4 + a(6) * values.^5;
        end

    end

end
