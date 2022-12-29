
P_cri = ParameterList()
add_parameter!(P_cri, Parameter("overlap", 100, 0, Int, "Size of the trajectory overlap. 0 disables this.", ""))

add_parameter!(P_cri, Parameter("v_0", 0, 0, float, "Initial speed [m.s^-1]", "init"))
add_parameter!(P_cri, Parameter("v_lim", 4.5, 4.5, float, "Maximum forward speed [m.s^-1]", "init"))
add_parameter!(P_cri, Parameter("a_acc_max", 0.8, 0.8, float, "Maximum longitudal acceleration [m.s^-2]", "init"))
add_parameter!(P_cri, Parameter("a_break_max", 4.5, 4.5, float, "Maximum longitudal decceleration [m.s^-2]", "init"))

add_parameter!(P_cri, Parameter("g", 9.81, 9.81, float, "Gravity acceleration coeficient [m.s^-2]", "vehicle"))
add_parameter!(P_cri, Parameter("m", 3.68, 3.68, float, "Vehicle mass [kg]", "vehicle"))
add_parameter!(P_cri, Parameter("l_f", 0.16, 0.16, float, "Distance between center of mass and front axle [m]", "vehicle"))
add_parameter!(P_cri, Parameter("l_r", 0.16, 0.16, float, "Distance between center of mass and rear axle [m]", "vehicle"))
add_parameter!(P_cri, Parameter("h", 0.08, 0.08, float, "Height of the center of mass [m]", "vehicle"))
add_parameter!(P_cri, Parameter("I_z", 0.051558333, 0.051558333, float, "Moment of interia of the vehicle around z-axis [kg.m^2]", "vehicle"))

#add_parameter!(P_cri, Parameter("C_sf", 7.5, 7.5, float, "Longitudinal slip coefficient of the front tire [-]", "tire"))
add_parameter!(P_cri, Parameter("C_sf", 2.0, 2.0, float, "Longitudinal slip coefficient of the front tire [-]", "tire"))
add_parameter!(P_cri, Parameter("C_sr", 7.5, 7.5, float, "Longitudinal slip coefficient of the rear tire [-]", "tire"))
add_parameter!(P_cri, Parameter("C_sa", 0.5, 0.5, float, "Longitudinal drop factor [-]", "tire"))
add_parameter!(P_cri, Parameter("C_af", 8.5, 8.5, float, "Cornering stiffness of the front tire [-]", "tire"))
add_parameter!(P_cri, Parameter("C_ar", 8.5, 8.5, float, "Cornering stiffness of the rear tire [-]", "tire"))
add_parameter!(P_cri, Parameter("C_as", 0.5, 0.5, float, "Lateral drop factor [-]", "tire"))

add_parameter!(P_cri, Parameter("s_s", 0.1, 0.1, float, "Saturation slip ratio [-]", "tire"))
add_parameter!(P_cri, Parameter("alpha_s", 0.0873, 0.0873, float, "Saturation sideslip angle [rad]", "tire"))

#add_parameter!(P_cri, Parameter("cl", 0.3, 0.3, float, "Air drag coefficient [-]", "aerodynamic"))
add_parameter!(P_cri, Parameter("cl", 1.0, 1.0, float, "Air drag coefficient [-]", "aerodynamic"))
#add_parameter!(P_cri, Parameter("ro", 1.249512, 1.249512, float, "Air density [N.m^-2]", "aerodynamic"))
add_parameter!(P_cri, Parameter("ro", 1.2, 1.2, float, "Air density [N.m^-2]", "aerodynamic"))
add_parameter!(P_cri, Parameter("A", 0.3, 0.3, float, "Effective flow surface [m^2]", "aerodynamic"))



#New profiler:
#=
_mu = 0.75          # Friction coeficient
_g = 9.81           # Gravity acceleration coeficient
_m = 3.68           # Vehicle mass
_ro = 1.249512           # Air density
_A = 0.3            # Frontal reference aerodynamic area
_cl = 0.3            # Drag coeficient
v_0 = 0             # Initial speed [m.s^-1]
v_lim = 4.5         # Maximum forward speed [m.s^-1]
a_acc_max = 0.8     # Maximum longitudal acceleration [m.s^-2]
a_break_max = 4.5   # Maximum longitudal decceleration [m.s^-2]
=#

_mu = 0.2          # Friction coeficient
_g = 9.81           # Gravity acceleration coeficient
_m = 3.68           # Vehicle mass
_ro = 1.2           # Air density
_A = 0.3            # Frontal reference aerodynamic area
_cl = 1            # Drag coeficient
v_0 = 0             # Initial speed [m.s^-1]
v_lim = 4.5         # Maximum forward speed [m.s^-1]
a_acc_max = 0.8     # Maximum longitudal acceleration [m.s^-2]
a_break_max = 4.5   # Maximum longitudal decceleration [m.s^-2]

#=
consts for IROS Torino tests?
#_mu = 0.2   # Friction coeficient
#_g = 9.81   # Gravity acceleration coeficient
#_m = 3.68   # Vehicle mass
#_ro = 1.2   # Air density
#_A = 0.3    # Frontal reference aeoradynamic area
#_cl = 1     # drag coeficient
#v_lim = 4.5, a_break_max = 4.5
#v_0 = 0, a_acc_max = 0.8
=#

function criterion_init()
    # TODO: update globals
end

function compute(points, overlap::Int=100; overflown...)
    #_, _, _t = jazar_profile_compute(points, overlap)
    _, _, _t = profile_compute(points, overlap)
    return Float64(_t[end])
end

######################
# Length
######################

function path_length(points; overflown...)
    sum(sqrt.(sum((circshift(points[:, 1:2], 1) .- points[:, 1:2]) .^ 2, dims=2)), dims=1)[1]
end

######################
# Profile\profiler
######################

function overlap_create(points, overlap)
    _overlap = overlap > size(points, 1) ? size(points, 1) : overlap
    vcat(points[end-_overlap+1:end, :], points, points[1:_overlap, :])
end

function overlap_remove(points, overlap)
    _overlap = overlap > size(points, 1) ? size(points, 1) : overlap
    points[_overlap+1:end-_overlap, :]
end

function fz(v::Float64)
    _m * _g + 0.5 * (_ro * _A * _cl * v * v)
end

function fy(v::Float64, k::Float64)
    _m * v * v * k
end

function h(k::Float64, v::Float64, d::Int, i::Int=-1)
    fz_res = fz(v)
    fy_res = fy(v, k)

    if (_mu * _mu * fz_res * fz_res - fy_res * fy_res) <= 0
        return 0
    end

    if d == -1
        return -sqrt(_mu * _mu * fz_res * fz_res - fy_res * fy_res) / _m
    elseif d == 1
        return sqrt(_mu * _mu * fz_res * fz_res - fy_res * fy_res) / _m
    end

end

function backward_pass(points)
    k = size(points, 1)
    len = size(points, 1)

    cur = zeros(k)
    for (i, p) in enumerate(eachrow(points))
        cur[i] = p[3] != 0 ? abs(p[3]) : 0.001
    end

    v_bwd = zeros(k)

    a = zeros(k)
    v_max = zeros(k)
    v_max_cr = zeros(k)
    alim = zeros(k)
    v_bwd[(k%len)+1] = v_lim
    v_max[(k%len)+1] = v_lim

    while k > 0
        v_max_cr[k] = sqrt(_mu * _g / cur[k])
        v_max[k] = min(v_max_cr[k], v_lim)
        ds = sqrt((points[(k%len)+1, 1] - points[k, 1])^2 + (points[(k%len)+1, 2] - points[k, 2])^2)
        alim[(k%len)+1] = (v_max[k]^2 - v_bwd[(k%len)+1]^2) / (2 * ds)
        a[k] = -h(cur[(k%len)+1], v_bwd[(k%len)+1], -1, k + 1)
        a[k] = min(min(a[k], a_break_max), alim[(k%len)+1])
        v_bwd[k] = sqrt((v_bwd[(k%len)+1] * v_bwd[(k%len)+1]) + (2 * a[k] * ds))
        k -= 1
    end

    return v_bwd, v_max, cur
end

function forward_pass(points, v_bwd, v_max, cur)
    k = 1
    len = size(points, 1)
    t = zeros(len)
    v_fwd = zeros(len)
    a = zeros(len)
    t[k] = 0
    v_fwd[k] = v_0

    while k <= len
        ds = sqrt((points[k, 1] - points[(k%len)+1, 1])^2 + (points[k, 2] - points[((k%len)+1), 2])^2)
        a_lim = (v_bwd[(k%len)+1] * v_bwd[(k%len)+1] - v_fwd[k] * v_fwd[k]) / (2 * ds)
        a[k] = h(cur[k], v_fwd[k], 1)
        a[k] = min(min(a[k], a_acc_max), a_lim)
        v_fwd[(k%len)+1] = sqrt(v_fwd[k] * v_fwd[k] + 2 * a[k] * ds)
        t[(k%len)+1] = t[k] + 2 * ds / (v_fwd[(k%len)+1] + v_fwd[k])
        k = k + 1
    end

    return v_fwd, a, t
end

function profile_compute(points, overlap::Int=0)
    #println(_mu, " ", _g, " ", _m, " ", _ro, " ", _A, " ", _cl, " ", v_0, " ", v_lim, " ", a_acc_max, " ", a_break_max)
    # Overlap points
    if overlap > 0
        _points = overlap_create(points, overlap)
    else
        _points = points
    end

    # Compute speed profile
    bwd, mx, cur = backward_pass(_points)
    v, a, t = forward_pass(_points, bwd, mx, cur)

    # Convert to numpy.ndarray
    bwd = bwd[:, :]
    mx = mx[:, :]
    cur = cur[:, :]
    _v = v[:, :]
    _a = a[:, :]
    _t = t[:, :]

    # Remove overlap and convert to numpy.ndarray
    if overlap > 0
        _v = overlap_remove(_v, overlap)
        _a = overlap_remove(_a, overlap)
        _t = overlap_remove(_t, overlap)


        # Fix time array
        _t = _t .- _t[1]

        bwd = overlap_remove(bwd, overlap)
        mx = overlap_remove(mx, overlap)
        cur = overlap_remove(cur, overlap)

        # if fd !== nothing
        #     write(fd, "Backward speed: \n $bwd")
        #     write(fd, "Maximum speed: \n $mx")
        #     write(fd, "Curvature: \n $cur")
        #     write(fd, "Final speed: \n $_v")
        #     flush(fd)
        # end

    end

    return _v, _a, _t
end

######################
# Profile\Jazar
######################

function jazar_forward_pass(points)
    v_fwd = zeros(size(points, 1))
    v_max = zeros(size(points, 1))

    beta = 0

    # Parameters
    m = get_value(P_cri, "m")
    g = get_value(P_cri, "g")

    _ca = 0.5 * get_value(P_cri, "ro") * get_value(P_cri, "cl") * get_value(P_cri, "A")
    mu = min(get_value(P_cri, "C_sf"), get_value(P_cri, "C_sr")) * get_value(P_cri, "s_s")
    

    v_fwd[1] = min(
        get_value(P_cri, "v_lim"),
        sqrt(
            sqrt(
                (mu^2 * m^2 * g^2) / (_ca^2 - 2 * _ca * m * abs(points[1, 3]) * tan(beta) + m^2 * points[1, 3]^2 * tan(beta)^2 + m^2 * points[1, 3]^2)
            )
        )
    )

    for i in range(1, size(points, 1) - 1)
        _x, _y, _k = points[i, :]
        _x1, _y1, _k1 = points[i + 1, :]


        # Distance between points
        ds = hypot(_x1 - _x, _y1 - _y)


        if _k1 != 0
            # Compute maximum permissible steady-state vehicle speed
            # With aerodynamics
            v_fwd[i + 1] = sqrt(
                sqrt(
                    (mu^2 * m^2 * g^2) / (_ca^2 - 2 * _ca * m * _k1 * tan(beta) + m^2 * _k1^2 * tan(beta)^2 + m^2 * _k1^2)
                )
            )


            # Constrain it with maximum allowed speed
            v_fwd[i + 1] = min(v_fwd[i + 1], get_value(P_cri, "v_lim"))
            v_max[i + 1] = v_fwd[i + 1]


            # Compute acceleration target, i.e., how much we have to accelerate
            # in order to match maximum speed on the next point.
            a_tar = (v_fwd[i + 1]^2 - v_fwd[i]^2) / (2 * ds)


            # Compute maximum acceleration with respect to Kamm's circle.
            # As we are calculating v_x, I assume that we want mu_x;
            # that should be maximum friction C_s * s_s
            _a_lim = mu^2 * g^2 - (v_fwd[i]^4 * _k^2)

            if _a_lim < 0
                a_lim = sqrt(-_a_lim)
            else
                a_lim = sqrt(_a_lim)
            end

        else # when _k1 == 0
            # Acceleration target is to match maximum speed
            a_tar = (get_value(P_cri, "v_lim")^2 - v_fwd[i]^2) / (2 * ds)


            # Kamm's circle with zero turning.(?)
            a_lim = sqrt(
                mu^2 * g^2
            )

        end # if _k1 != 0


        # Compute acceleration (use all constraints together)
        a = min(
            min(
                a_tar,
                a_lim
            ),
            get_value(P_cri, "a_acc_max")
        )


        # Modify velocity of the next point
        v_fwd[i + 1] = sqrt(
            v_fwd[i]^2 + 2 * a * ds
        )
    end # for i in range

    return v_fwd, v_max
end


function jazar_backward_pass(points, v_fwd)
    v = zeros(size(points, 1))
    a = zeros(size(points, 1))
    dt = zeros(size(points, 1))

    v[end] = v_fwd[end]

    # Parameters
    m = get_value(P_cri, "m")
    g = get_value(P_cri, "g")

    mu = min(get_value(P_cri, "C_sf"), get_value(P_cri, "C_sr")) * get_value(P_cri, "s_s") 

    for i in reverse(range(2, size(points, 1)))
        _x, _y, _k = points[i, :]
        _x1, _y1, _k1 = points[i - 1, :]


        # Distance between points
        ds = hypot(_x1 - _x, _y1 - _y)


        if _k1 != 0
            # Compute acceleration target, i.e., how much we have to accelerate
            # in order to match maximum speed on the next point.
            a_tar = (v_fwd[i - 1]^2 - v[i]^2) / (2 * ds)


            # Compute maximum acceleration with respect to Kamm's circle.
            # As we are calculating v_x, I assume that we want mu_x;
            # that should be maximum friction C_s * s_s
            _a_lim = mu^2 * g^2 - (v[i]^4 * _k^2)

            if _a_lim < 0
                a_lim = sqrt(-_a_lim)
            else
                a_lim = sqrt(_a_lim)
            end

        else # when _k1 == 0
            # Acceleration target is to match maximum speed
            a_tar = (get_value(P_cri, "v_lim")^2 - v[i]^2) / (2 * ds)


            # Kamm's circle with zero turning.(?)
            a_lim = sqrt(
                mu^2 * g^2
            )

        end # if _k1 != 0


        # Compute acceleration (use all constraints together)
        a[i - 1] = min(
            min(
                a_tar,
                a_lim
            ),
            get_value(P_cri, "a_break_max")
        )


        # Modify velocity of the next point
        v[i - 1] = sqrt(
            v[i]^2 + 2 * a[i - 1] * ds
        )


        # Compute time required to drive over ds
        dt[i] = ds / ( (v[i] + v[i - 1]) / 2 )

    end # for i in range


    return v, a, cumsum(dt)
end


function jazar_profile_compute(points, overflown...)

    overlap = get_value(P_cri, "overlap")

    # Overlap points
    if overlap > 0
        _points = overlap_create(points, overlap)
    else
        _points = points
    end


    # Run passes
    v_fwd, v_max = jazar_forward_pass(_points)
    v, a, t = jazar_backward_pass(_points, v_fwd)


    # Remove overlap
    if overlap > 0
        v = overlap_remove(v, overlap)
        a = overlap_remove(a, overlap)
        t = overlap_remove(t, overlap)

        # Fix time array
        t = t .- t[1]
    end

    return v, a, t

end


if (abspath(PROGRAM_FILE) == @__FILE__)
    a = [15.948168943187245 5.184533766170699; 13.344948522985717 7.352767046797679; 9.842467467003205 7.2210443303164995; 5.94249715695647 7.221049885241514; 1.9486657857305651 6.829040548798835; 1.1579217265457844 3.317023590873; 3.10027006023066 1.5448352214980736; 5.99608251621141 2.180063409932212; 5.1267522596244195 5.163292925161625; 7.989068288582386 5.057849184482343; 12.325639560801093 4.464779551662567; 14.650113631105448 1.74629088388137]
    println(profile_compute(a, 100))
end
