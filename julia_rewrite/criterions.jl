
const _mu = 0.2           # Friction coeficient
const _g = 9.81           # Gravity acceleration coeficient
const _m = 3.68           # Vehicle mass
const _ro = 1.2           # Air density
const _A = 0.3            # Frontal reference aerodynamic area
const _cl = 1             # Drag coeficient
const v_0 = 0             # Initial speed [m.s^-1]
const v_lim = 4.5         # Maximum forward speed [m.s^-1]
const a_acc_max = 0.8     # Maximum longitudal acceleration [m.s^-2]
const a_break_max = 4.5   # Maximum longitudal decceleration [m.s^-2]

function criterion_init()
    # TODO: update globals
end

function compute(points, overlap::Int=100; overflown...)
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

if (abspath(PROGRAM_FILE) == @__FILE__)
    a = [15.948168943187245 5.184533766170699; 13.344948522985717 7.352767046797679; 9.842467467003205 7.2210443303164995; 5.94249715695647 7.221049885241514; 1.9486657857305651 6.829040548798835; 1.1579217265457844 3.317023590873; 3.10027006023066 1.5448352214980736; 5.99608251621141 2.180063409932212; 5.1267522596244195 5.163292925161625; 7.989068288582386 5.057849184482343; 12.325639560801093 4.464779551662567; 14.650113631105448 1.74629088388137]
    println(profile_compute(a, 100))
end
