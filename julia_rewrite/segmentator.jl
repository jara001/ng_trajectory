using Printf
using LinearAlgebra

include("utils.jl")
include("parameter.jl")

# using .ParameterListClass.ParameterClass
# using .ParameterListClass

MAP = nothing
MAP_ORIGIN = nothing
MAP_GRID = nothing

P_seg = ParameterList()
add_parameter!(P_seg, Parameter("hold_map", false, false, Bool, "When true, the map is created only once.", "init"))
add_parameter!(P_seg, Parameter("range_limit", 0, 0, float, "Maximum distance from the center of the segment. 0 disables this.", ""))
add_parameter!(P_seg, Parameter("reserve_width", false, false, Bool, "When true, the segments are reserved a path towards both walls.", ""))
add_parameter!(P_seg, Parameter("reserve_selected", [], [], Array, "IDs of segments that should use the reservation method, when empty, use all.", ""))
add_parameter!(P_seg, Parameter("reserve_distance", 2.0, 2.0, float, "Distance from the line segment that is reserved to the segment.", ""))
add_parameter!(P_seg, Parameter("plot_flood", false, false, Bool, "Whether the flooded areas should be plotted.", ""))

function segmentator_init(track; kwargs...)
    global MAP, MAP_ORIGIN, MAP_GRID

    update_all!(P_seg, kwargs)

    if MAP === nothing || get_value(P_seg, "hold_map") == false
        MAP, MAP_ORIGIN, MAP_GRID = map_create(track)
    end
end

function segment_distance(p, a, b)
    distance_to_seg = abs((b[1] - a[1]) * (a[2] - p[2]) - (a[1] - p[1]) * (b[2] - a[2])) / sqrt((b[1] - a[1])^2 + (b[2] - a[2])^2)

    unit_ap = [p[1] - a[1], p[2] - a[2]]
    unit_ab = [b[1] - a[1], b[2] - a[2]]
    unit_ap = unit_ap / norm(unit_ap)
    unit_ab = unit_ab / norm(unit_ab)
    dot_ap = dot(unit_ap, unit_ab)
    dot_ap = min(1.0, max(-1.0, dot_ap))
    angle_ap = acosd(dot_ap)

    unit_bp = [p[1] - b[1], p[2] - b[2]]
    unit_ba = [a[1] - b[1], a[2] - b[2]]
    unit_bp = unit_bp / norm(unit_bp)
    unit_ba = unit_ba / norm(unit_ba)
    dot_bp = dot(unit_bp, unit_ba)
    dot_bp = min(1.0, max(-1.0, dot_bp))
    angle_bp = acosd(dot_bp)

    if 0.0 <= angle_ap <= 90.0 && 0.0 <= angle_bp <= 90.0
        return distance_to_seg
    elseif angle_ap > 90.0
        return point_distance(a, p)
    elseif angle_bp > 90.0
        return point_distance(b, p)
    end

    println("segmentDistance: Unexpected situation.")
    @printf("segmentDistance: point = %s, a = %s, b = %s", p, a, b)
    @printf("segmentDistance: angle_ap = %f, angle_bp = %f", angle_ap, angle_bp)
    @printf("segmentDistance: d_to_seg = %f, d_ap = %f, d_bp = %f", distance_to_seg, point_distance(a, p), point_distance(b, p))
    @printf("segmentDistance: V_ap = %s, V_ab = %s, V_ap*V_ab = %s", unit_ap, unit_ab, dot(unit_ap, unit_ab))
    throw(ArgumentError("Unexpected situation at 'segmentDistance'. Read the output for values of the variables."))
end

function segmentate(points, group_centers; overflown...)
    global MAP, MAP_ORIGIN, MAP_GRID, MAP_LAST

    update_all!(P_seg, overflown, reset=false)

    _groups = [[] for _ in 1:size(group_centers, 1)]

    if get_value(P_seg, "reserve_width") == false
        _map = copy(MAP)
        _map[_map.==100] .= 255

        for (_i, _c) in zip(Iterators.countfrom(0), eachrow(points_to_map(group_centers)))
            _map[_c[1], _c[2]] = (_i) & 0xff
        end
    else
        println("Computing reserved zones...")

        _map = zeros(UInt8, size(MAP)[1] + 2, size(MAP)[2] + 2)
        _map[2:end-1, 2:end-1] = copy(MAP)
        _map[_map.==100] .= 255

        color = 200
        walls = findall(_map .== 0)

        while length(walls) > 0
            queue = [(walls[1][1], walls[1][2])]

            while length(queue) > 0
                cell = popfirst!(walls)

                for _a in -1:1
                    for _b in -1:1
                        if _a == 0 && _b == 0
                            continue
                        end

                        if cell[1] + _a < 0 || cell[2] + _b < 0
                            continue
                        end

                        try
                            _cell = _map[cell[1]+_a, cell[2]+_b]
                        catch
                            continue
                        end

                        if _cell == 0
                            _map[cell[1]+_a, cell[2]+_b] = color
                            push!(queue, (cell[1] + _a, cell[2] + _b))
                        end
                    end
                end
            end
            color = color + 1
            walls = findall(_map .== 0)
        end

        println("\tDetected walls: ", color - 200)

        for (_i, _c) in enumerate(eachrow(points_to_map(group_centers)))
            println("\tSegment ", _i, "/", len(group_centers))
            _map[_c[1], _c[2]] = (_i) & 0xff

            if length(get_value(P_seg, "reserve_selected")) > 0 && occursin(get_value(P_seg, "reserve_selected"), _i)
                continue
            end

            for _wall_index in 0:1
                distance = 100000
                closest = nothing

                walls = findall(_map .== (200 + _wall_index))

                for (_wx, _wy) in zip(getindex.(walls, 1), getindex.(walls, 2))
                    _distance = sqrt.((_wx - _c[1])^2 + (_wy - _c[2])^2)

                    if _distance < distance
                        distance = _distance
                        closest = (_wx, _wy)
                    end
                end
            end

            @printf("\t\tWall %i... %03.2f%%", _wall_index, 0.0)

            valids = findall(_map .== 255)
            valids_length = length(valids)

            for (_vi, (_vx, _vy)) in enumerate(zip(getindex.(valids, 1), getindex.(valids, 2)))
                _distance = segment_distance((_vx, _vy), _c, closest)

                if _distance < get_value(P_seg, "reserve_distance")
                    _map[_vx, _vy] = 100 + _i
                end
                if _vi % 1000 == 0
                    @printf("\r\t\tWall %i... %03.2f%%", _wall_index, 100.0 * _vi / valids_length)
                end
            end
            @printf("\r\t\tWall %i... %03.2f%%\n", _wall_index, 100.0)
        end
    end

    queue = points_to_map(group_centers)
    while size(queue, 1) > 0
        cell = queue[1:1, :]
        queue = queue[1:end.!=1, :]

        for _a in -1:1
            for _b in -1:1
                if _a == 0 && _b == 0
                    continue
                end

                if cell[1] + _a < 0 || cell[2] + _b < 0
                    continue
                end

                _cell = nothing

                try
                    _cell = _map[cell[1]+_a, cell[2]+_b]
                catch
                    continue
                end

                if _cell == 255 || _cell == 100 + _map[cell[1], cell[2]]
                    _map[cell[1]+_a, cell[2]+_b] = _map[cell[1], cell[2]]
                    queue = vcat(queue, [cell[1] + _a cell[2] + _b])
                    # push!(queue, [cell[1] + _a, cell[2] + _b])
                end
            end
        end
    end

    # @show(_map)
    MAP_LAST = _map
    for p in eachrow(points)
        index = point_to_map(p)
        _i = _map[index[1], index[2]]

        # Group only taken points
        if _i != 255 && _i < 100
            push!(_groups[_i + 1], p)
        end
    end

    groups = [g for g in _groups]

    # TODO: plot results

    if get_value(P_seg, "range_limit") <= 0
        return [mapreduce(permutedims, vcat, g) for g in groups if size(g, 1) > 0]
    else
        return [x[sqrt.(sum.((x[:, 1:2] .- group_centers[ix][:2])^2, dims=2))<get_value(P_seg, "range_limit")] for (ix, x) in enumerate(groups) if length(x) > 0]
    end
end


if (abspath(PROGRAM_FILE) == @__FILE__)
    using NPZ
    START_POINTS = npzread("configuration/ng_start_points_torino2.npy")
    VALID_POINTS = npzread("configuration/ng_valid_points_torino2.npy")

    segmentator_init(VALID_POINTS)

    a = START_POINTS .+ 0.1
    println(segmentate(START_POINTS, a))

end
