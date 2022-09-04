include("utils.jl")
include("interpolator.jl")
include("parameter.jl")
include("criterions.jl")

# using .ParameterListClass.ParameterClass
# using .ParameterListClass

P_sel = ParameterList()
add_parameter!(P_sel, Parameter("sampling_distance", 1.0, 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "init"))
add_parameter!(P_sel, Parameter("distance", 0, 0, float, "[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.", "init"))
add_parameter!(P_sel, Parameter("rotate", 0, 0, float, "Parameter for rotating the input path. 0 is not rotated. <0, 1)", "init"))
add_parameter!(P_sel, Parameter("fixed_points", [], [], Array, "Points to be used in the selection upon calling 'select'.", "init"))

#curvature2

function selector_init(; kwargs...)
    #TODO: rotate
    update_all!(P_sel, kwargs)
end

function select(points, remain::Int; overflown...)
    if remain < 0 && getValue(P, "distance") <= 0
        # Raise an exception, as we cannot proceed without further information.
        throw(ArgumentError("Negative selection requires set 'distance' parameter for 'uniform_distance' selector."))
    end

    rpoints = trajectory_resample(points, remain)

    # !Force number of points
    if remain > 0 && size(rpoints, 1) != remain
        return trajectory_resample(points, remain - 1)
    end

    return rpoints
end

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

    # Throw away repeated point
    if points[1, :1] == points[end, :1]
        points = points[1:end-1, :]
    end

    # Keep fixed points local
    raw_fixed_points = copy(get_value(P_sel, "fixed_points"))

    # Result
    rpoints = []
    # Intermediate results
    fixed_points = []
    upoints = []

    # Other values
    rotate = typeof(get_value(P_sel, "rotate")) != Vector ? [get_value(P_sel, "rotate") for _ in range(1, stop=max(1, length(get_value(P_sel, "fixed_points"))))] : copy(get_value(P_sel, "rotate"))

    while true
        # Rotate to get to the first fixed point
        _points = circshift(points, length(raw_fixed_points) > 0 ? -trajectory_closest_index(points, popfirst!(raw_fixed_points)) : 0)

        # Resample if requested
        if get_value(P_sel, "sampling_distance") != 0.0
            _points = interpolate(_points[:, 1:2], int_size=resolution_estimate(_points, get_value(P_sel, "sampling_distance")))
        end

        # Select points equidistantly
        if remain < 0
            _rpoints = interpolate(_points[:, 1:2], int_size=resolution_estimate(_points, get_value(P_sel, "distance")))
        # Select 'remain' points
        else
            _rpoints = interpolate(_points[:, 1:2], int_size=remain)
        end

        # Rotate when required
        if rotate[1] > 0.0
            #TODO
        else
            popfirst!(rotate)

            # Create fpoints with a set factor to allow concatenating
            _fpoints = interpolate(_points[:, 1:2], int_size=10 * length(_rpoints))

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
        # Build up the new path waypoints
        result = nothing

        for _i in range(1, stop=length(rpoints))

            # 1) Take the fixed point from the next segment
            _p = fixed_points[(_i+1)%length(rpoints)]

            # 2) Find it in current path (rotated, full)
            _cpi = trajectory_closest_index(upoints[_i], _p; from_left=true)

            # 3) Loop through the selection to find all points that are more to the left
            _max_i = 0

            while _max_i + 1 < length(rpoints[_i]) && trajectory_closest_index(upoints[_i], rpoints[_i][_max_i+1, :]', from_left=true) < _cpi
                _max_i += 1
            end

            # 4) Append them to the result
            if _max_i >= 1
                if result == Nothing
                    result = rpoints[_i][1:_max_i+1, :]
                else
                    result = vcat(result, rpoints[_i][1:_max_i+1, :])
                end
            end
        end
        return result
    end
end

function xxx()
    println("bye")
end

if (abspath(PROGRAM_FILE) == @__FILE__)
    using NPZ
    START_POINTS = npzread("configuration/ng_start_points_torino2.npy")
    res = trajectory_resample(vcat(START_POINTS), 12)
    println(res)
    println(size(res, 1))
end
