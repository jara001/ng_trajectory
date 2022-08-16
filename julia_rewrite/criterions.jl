
_mu = 0.2           # Friction coeficient
_g = 9.81           # Gravity acceleration coeficient
_m = 3.68           # Vehicle mass
_ro = 1.2           # Air density
_A = 0.3            # Frontal reference aerodynamic area
_cl = 1             # Drag coeficient
v_0 = 0             # Initial speed [m.s^-1]
v_lim = 4.5         # Maximum forward speed [m.s^-1]
a_acc_max = 0.8     # Maximum longitudal acceleration [m.s^-2]
a_break_max = 4.5   # Maximum longitudal decceleration [m.s^-2]

######################
# Length
######################

function path_length(points, overflown...)
    sum(sqrt.(sum((circshift(points[:, 1:2], 1) .- points[:, 1:2]).^2, dims = 2)), dims = 1)[1]
end

######################
# Profile\profiler
######################

function overlap_create(points, overlap)
    vcat(points[end - overlap + 1:end - overlap + 1,:], points, points[1:overlap])
end

function overlap_remove(points, overlap)
    points[overlap:end + 1 - overlap, :]
end

function fz(v::Float64)
    _m*_g + 0.5*(_ro*_A*_cl*v*v)
end

function fy(v::Float64, k::Float64)
    _m*v*v*k
end

function h(k::Float64, v::Float64, d::Int, i::Int = -1)
    fz_res = fz(v)
    fy_res = fy(v,k)

    if (_mu * _mu * fz_res * fz_res - fy_res * fy_res) <= 0:
        return 0
    end

    if d == -1:
        return -sqrt(_mu * _mu * fz_res * fz_res - fy_res * fy_res) / _m
    elseif d == 1:
        return sqrt(_mu * _mu * fz_res * fz_res - fy_res * fy_res) / _m
    end

end

function backward_pass(points)
    k = length(points)

    cur = zeros(k)
    for i, p in enumerate(points)
        cur[i] = p[2] != 0 ? abs(p[2]) : 0.001
    end

    v_bwd = zeros(k)

    a = zeros(k)
    v_max = zeros(k)
    v_max_cr = zeros(k)
    alim = zeros(k)
    v_bwd[(k + 1) % length(points)] = v_lim
    v_max[(k + 1) % length(points)] = v_lim

    while k > 0
        v_max_cr[k] = sqrt(_mu*_g/cur[k])
        v_max[k] = min(v_max_cr[k], v_lim)
        ds = sqrt((points[(k + 1) % len(points), 0] - points[k, 0])^2 + (points[k%len(points), 1] - points[k-1, 1])^2)
        alim[(k + 1) % length(points)] = (v_max[k]^2 - v_bwd[(k + 1) % len(points)]^2) / (2 * ds)
        a[k] = -h(cur[(k + 1) % length(points)], v_bwd[(k + 1) % length(points)], -1 , k + 1)
        a[k] = min(min(a[k], a_break_max), alim[(k + 1) % length(points)])
        v_bwd[k] = sqrt((v_bwd[(k + 1) % len(points)] * v_bwd[(k + 1) % length(points)]) + (2 * a[k] * ds))
        k -= 1
    end

    return v_bwd, v_max, cur
end

function forward_pass(points, v_bwd, v_max, cur)
    k = 1
    t = zeros((length(points)))
    v_fwd = zeros((length(points)))
    a = zeros((length(points)))
    t[k] = 0
    v_fwd[k] = v_0

    while k <= len(points):
        ds = sqrt((points[k, 1] - points[(k + 1) % length(points), 1])^2 + (points[k, 2] - points[(k + 1)%len(points), 2])^2)
        a_lim = (v_bwd[(k + 1) % length(points)] * v_bwd[(k + 1) % length(points)] - v_fwd[k] * v_fwd[k])/(2*ds)
        a[k] = h(cur[k], v_fwd[k], 1)
        a[k] = min(min(a[k], a_acc_max),a_lim)
        v_fwd[(k + 1) % length(points)] = sqrt(v_fwd[k]*v_fwd[k] + 2*a[k]*ds)
        t[(k + 1) % length(points)] = t[k] + 2*ds/(v_fwd[(k + 1) % length(points)] + v_fwd[k])
        k = k + 1
    end

    return v_fwd, a, t
end

function profile_compute(points, overlap::Int = 0, fd::IOStream = nothing)
    if overlap > 0
        _points = overlap_create(points, overlap)
    else
        _points = points
    end

    bwd, mx, cur = backward_pass(_points)
    v, a, t = forward_pass(_points, bwd, mx, cur)

    bwd = bwd[:, :]
    mx = mx[:, :]
    cur = cur[:, :]
    _v = v[:, :]
    _a = a[:, :]
    _t = t[:, :]

    if overlap > 0
        _v = overlap_remove(_v, overlap)
        _a = overlap_remove(_a, overlap)
        _t = overlap_remove(_t, overlap)

        _t = _t .- _t[1]

        bwd = overlapRemove(bwd, overlap)
        mx = overlapRemove(mx, overlap)
        cur = overlapRemove(cur, overlap)

        if fd != nothing
            write(fd, "Backward speed: \n $bwd")
            write(fd, "Maximum speed: \n $mx")
            write(fd, "Curvature: \n $cur")
            write(fd, "Final speed: \n $_v")
            flush(fd)
        end

    end

    return _v, _a, _t
end

if (abspath(PROGRAM_FILE) == @__FILE__)
    a = [ 0.16433    0.524746   0.524746;
        0.730177   0.787651   0.787651;
        0.646905   0.0135035   0.0135035;
        0.796598   0.0387711   0.0387711;
        0.442782   0.753235   0.753235;
        0.832315   0.483352   0.483352;
        0.442524   0.912381   0.912381;
        0.336651   0.236891   0.236891;
        0.0954936  0.303086   0.303086;
        0.459189   0.374318   0.374318]
    println(path_length(a))
end
