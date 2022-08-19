include("utils.jl")
include("interpolator.jl")
include("parameter.jl")
include("criterions.jl")

using .ParameterListClass.ParameterClass
using .ParameterListClass

P = ParameterList()
add_parameter!(P, Parameter("sampling_distance", 1.0, 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "init"))
add_parameter!(P, Parameter("distance", 0, 0, float, "[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.", "init"))
add_parameter!(P, Parameter("rotate", 0, 0, float, "Parameter for rotating the input path. 0 is not rotated. <0, 1)", "init"))
add_parameter!(P, Parameter("fixed_points", [], [], Array, "Points to be used in the selection upon calling 'select'.", "init"))

#curvature2

function path_point_distance_avg(points)
    path_length(points) / length(points)
end

function factor_compute(points, resolution)
    path_point_distance_avg(points) / resolution
end

function resolution_estimate(points, resolution)
    trunc(Int, length(points) * factor_compute(points, resolution))
end

function trajectory_resample(points, remain)
    if points[1, :1] == points[end, :1]
        points = points[1:end-1, :]
    end

    raw_fixed_points = copy(get_value(P, "fixed_points"))
    rpoints = []
    fixed_points = []
    upoints = []

    rotate = typeof(get_value(P, "rotate")) != Vector ? [get_value(P, "rotate") for _ in range(1, stop=max(1, length(get_value(P, "fixed_points"))))] : copy(get_value(P, "rotate"))

    while true
        _points = circshift(points, length(raw_fixed_points) > 0 ? -trajectory_closest_index(points, popfirst!(raw_fixed_points)) : 0)

        if get_value(P, "sampling_distance") != 0.0
            _points = interpolate(_points[:, 1:2], resolution_estimate(_points, get_value(P, "sampling_distance")))
        end

        if remain < 0
            _rpoints = interpolate(_points[:, 1:2], resolution_estimate(_points, get_value(P, "distance")))
        else
            _rpoints = interpolate(_points[:, 1:2], remain)
        end

        if rotate[1] > 0.0
            #TODO
        else
            popfirst!(rotate)
            _fpoints = interpolate(_points[:, 1:2], 10 * length(_rpoints))
            push!(fixed_points, _fpoints[1])
            push!(upoints, _fpoints)
        end

        push!(rpoints, _rpoints)

        if length(raw_fixed_points) <= 0
            break
        end
    end

    if length(rpoints) == 1
        return rpoints[1]
    else
        result = nothing

        for _i in range(1, stop=length(rpoints))
            _p = fixed_points[(_i+1)%length(rpoints)]
            _cpi = trajectory_closest_index(upoints[_i], _p; from_left=true)
            _max_i = 0

            while _max_i + 1 < length(rpoints[_i]) && trajectory_closest_index(upoints[_i], rpoints[_i][_max_i+1, :]', from_left=true) < _cpi
                _max_i += 1
            end

            if _max_i >= 1
                if result == Nothing
                    print("adding: ")
                    println(rpoints[_i][1:_max_i+1, :])
                    result = rpoints[_i][1:_max_i+1, :]
                else
                    print("adding: ")
                    println(rpoints[_i][1:_max_i+1, :])
                    result = vcat(result, rpoints[_i][1:_max_i+1, :])
                end
            end
        end
        return result
    end
end

if (abspath(PROGRAM_FILE) == @__FILE__)
    a = [0.16433 0.524746
        0.730177 0.787651
        0.646905 0.0135035
        0.796598 0.0387711
        0.442782 0.753235
        0.832315 0.483352
        0.442524 0.912381
        0.336651 0.236891
        0.0954936 0.303086
        0.459189 0.374318]
    println(result)
end
