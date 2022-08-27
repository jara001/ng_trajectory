using Statistics
using Dierckx
using Printf
using Evolutionary

include("utils.jl")
include("interpolator.jl")
include("segmentator.jl")
include("selector.jl")
include("criterions.jl")

using .ParameterListClass.ParameterClass
using .ParameterListClass

# Global variables
OPTIMIZER = nothing
MATRYOSHKA = nothing
VALID_POINTS = nothing
CRITERION_ARGS = nothing
INTERPOLATOR_ARGS = nothing
SEGMENTATOR_ARGS = nothing
SELECTOR_ARGS = nothing
PENALIZER_INIT = nothing
PENALIZER_ARGS = nothing
LOGFILE = nothing
VERBOSITY = 3
FILELOCK = ReentrantLock()
HOLDMAP = nothing
GRID = nothing
PENALTY = nothing
FIGURE = nothing
PLOT = nothing

P = ParameterList()

add_parameter!(P, Parameter("budget", 100, 100, Int, "Budget parameter for the genetic algorithm.", "init (general)"))
add_parameter!(P, Parameter("groups", 8, 8, Int, "Number of groups to segmentate the track into.", "init (general)"))
add_parameter!(P, Parameter("workers", "Sys.CPU_CORES", "Sys.CPU_CORES", Int, "Number threads for the genetic algorithm.", "init (general)"))
add_parameter!(P, Parameter("penalty", 100, 100, Float64, "Constant used for increasing the penalty criterion.", "init (general)"))
add_parameter!(P, Parameter("criterion_args", Dict(), Dict(), Dict, "Arguments for the criterion function.", "init (general)"))
add_parameter!(P, Parameter("interpolator_args", Dict(), Dict(), Dict, "Arguments for the interpolator function.", "init (general)"))
add_parameter!(P, Parameter("segmentator_args", Dict(), Dict(), Dict, "Arguments for the segmentator function.", "init (general)"))
add_parameter!(P, Parameter("selector_args", Dict(), Dict(), Dict, "Arguments for the selector function.", "init (general)"))
add_parameter!(P, Parameter("penalizer_init", Dict(), Dict(), Dict, "Arguments for the init part of the penalizer function.", "init (general)"))
add_parameter!(P, Parameter("penalizer_args", Dict(), Dict(), Dict, "Arguments for the penalizer function.", "init (general)"))
add_parameter!(P, Parameter("logging_verbosity", 2, 2, Int, "Index for verbosity of the logger.", "init (general)"))
add_parameter!(P, Parameter("hold_matryoshka", false, false, Bool, "Whether the transformation should be created only once.", "init (Matryoshka)"))
add_parameter!(P, Parameter("plot", false, false, Bool, "Whether a graphical representation should be created.", "init (viz.)"))
add_parameter!(P, Parameter("grid", "computed by default", "computed by default", Vector, "X-size and y-size of the grid used for points discretization.", "init (Matryoshka)"))
add_parameter!(P, Parameter("plot_mapping", false, false, Bool, "Whether a grid should be mapped onto the track (to show the mapping).", "init (viz.)"))

######################
# Utils
######################

function optimizer_init(; points,
    group_centers,
    group_centerline,
    budget::Int=100,
    layers::Int=5,
    groups::Int=8,
    workers::Int=Sys.CPU_THREADS,
    penalty=100,
    criterion_args::Dict=Dict(),
    interpolator_args::Dict=Dict(),
    segmentator_args::Dict=Dict(),
    selector_args::Dict=Dict(),
    penalizer_init::Dict=Dict(),
    penalizer_args::Dict=Dict(),
    logfile::IO=stdout,
    logging_verbosity::Int=2,
    hold_matryoshka::Bool=false,
    plot::Bool=false,
    grid::Vector=[],
    figure=nothing,
    kwargs...)

    global OPTIMIZER, MATRYOSHKA, VALID_POINTS, LOGFILE, VERBOSITY, HOLDMAP, GRID, PENALTY, FIGURE, PLOT
    global CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, SEGMENTATOR, SEGMENTATOR_ARGS, SELECTOR, SELECTOR_ARGS, PENALIZER, PENALIZER_INIT, PENALIZER_ARGS

    CRITERION_ARGS = criterion_args
    INTERPOLATOR_ARGS = interpolator_args
    SEGMENTATOR_ARGS = segmentator_args
    SELECTOR_ARGS = selector_args
    PENALIZER_INIT = penalizer_init
    PENALIZER_ARGS = penalizer_args
    LOGFILE = logfile
    VERBOSITY = logging_verbosity
    _holdmatryoshka = hold_matryoshka
    PENALTY = penalty
    FIGURE = figure
    PLOT = plot


    VALID_POINTS = points

    if MATRYOSHKA === nothing || _holdmatryoshka == false
        group_centers = select(group_centerline, groups; SELECTOR_ARGS...)

        if plot == true
            # TODO: plot
        end

        _groups = segmentate(points, group_centers; SEGMENTATOR_ARGS...)

        penalizer_init()

        grouplayers = groups_border_obtain(_groups)
        grouplayers = groups_border_beautify(grouplayers, 400)

        if plot == true
            # TODO: plot
        end

        layers_center = group_center_compute(_groups)
        layers_count = [layers for _ in 1:length(grouplayers)]

        MATRYOSHKA = [matryoshka_create(grouplayers[_i], layers_center[_i], layers_count[_i]) for _i in 1:length(_groups)]

        # TODO: plot

        println("Matryoshka mapping constructed.")

        if GRID === nothing
            if length(grid) == 2
                GRID = grid
            else
                _GRID = grid_compute(points)
                GRID = [_GRID, _GRID]
            end
        end
    end

    # TODO: set bounds
    OPTIMIZER = GA(populationSize=budget, Evolutionary.Options(parallelization=:thread))
end

function optimize()
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE, PLOT, PENALIZER, PENALIZER_ARGS

    res = Evolutionary.optimize(_opt, zeros(length(MATRYOSHKA), 2), OPTIMIZER)
    # TODO: nevergrad
    points = [matryoshka_map(MATRYOSHKA[i], [p])[1] for (i, p) in enumerate(eachrow(Evolutionary.minimizer(res)))]

    PENALIZER_ARGS["optimization"] = false
    final = _opt(Evolutionary.minimizer(res))

    _points = interpolate(points)

    # TODO: plot

    lock(FILELOCK) do
        if VERBOSITY > 0
            @printf(LOGFILE, "solution:%s", string(points))
            @printf(LOGFILE, "final:%f", final)
        end
    end

    return (final, points, Evolutionary.minimizer(res), _points)
end

function _opt(points)
    global VALID_POINTS, CRITERION_ARGS, INTERPOLATOR_ARGS, PENALIZER_ARGS
    global MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, GRID, PENALTY

    points = [matryoshka_map(MATRYOSHKA[i], [p])[1] for (i, p) in enumerate(eachrow(points))]

    _points = interpolate(points; INTERPOLATOR_ARGS...)

    penalty = penalize(_points, VALID_POINTS, GRID, PENALTY; PENALIZER_ARGS...)

    if penalty != 0
        lock(FILELOCK) do
            if VERBOSITY > 2
                @printf(LOGFILE, "pointsA:%s", string(points))
                @printf(LOGFILE, "pointsT:%s", string(_points))
            end
            if VERBOSITY > 1
                @printf(LOGFILE, "penalty:%f", penalty)
            end
            flush(LOGFILE)

        end
        return penalty
    end

    _c = compute(_points; CRITERION_ARGS...)
    lock(FILELOCK) do
        if VERBOSITY > 2
            @printf(LOGFILE, "pointsA:%s", string(points))
            @printf(LOGFILE, "pointsT:%s", string(_points.tolist()))
        end
        if VERBOSITY > 1
            @printf(LOGFILE, "correct:%f", _c)
        end
        LOGFILE.flush()
    end
    return _c
end

function matryoshka_create(layer0, layer0_center, layer_count::Int)
    layers = group_layers_compute(layer0, layer0_center, layer_count)
    layer0_size = length(layer0)

    _tc = Array{Float64,2}
    _rc = Array{Float64,2}

    for _layer_index in 1:layer_count
        layer_params = layer_index_to_parameters(_layer_index, layer0_size, layer_count)
        indices = 1:layer_params[1]
        tc = indices_to_transformed_coordinates(indices, layer_params...)
        rc = indices_to_real_coordinates(indices, layers[Int(_layer_index)])

        _tc = vcat(_tc, tc)
        _rc = vcat(_rc, rc)
    end

    _ip2d = []
    for _d in 1:size(_rc)[2]
        push!(_ip2d, Spline2D(_tc[:, 0], _tc[:, 1], _rc[:, _d]))
    end

    return _ip2d
end

function matryoshka_map(matryoshka, coords)

    _rcoords = []
    for _c in coords
        _dims = []
        for _interpolator in matryoshka
            push!(_dims, _interpolator(_c[0], _c[1]))
        end
        push!(_rcoords, _dims)
    end
    return _rcoords
end

function layer_index_to_parameters(layer_index::Int, layer0_size::Int, layer_count::Int)
    (trunc(Int, layer0_size - ((layer0_size / layer_count) * layer_index)), 1 - (1.0 / layer_count) * layer_index)
end

function indices_to_transformed_coordinates(indices, layer_size::Int, scale::Float64)
    _coords = []
    l = Float64(layer_size)
    for _i in indices
        _point = []
        for _d in [1, -1]
            push!(_point, (
                max(
                    min(
                        (abs(-((((l / 2.0) + _i + _d * (l / 8.0)) % l) - (l / 2.0))) / (l / 4.0)) - 0.5, 1),
                    0
                ) * scale + ((1 - scale) / 2.0)
            ))
        end
        push!(_coords, _point)
    end
    return _coords
end

function indices_to_real_coordinates(indices, points)
    _rcoords = []
    for _i in indices
        if trunc(Int, _i) == _i
            push!(_rcoords, points[trunc(Int, _i), :])
        else
            push!(_rcoords,
                points[trunc(Int, _i), :] .+ (_i - trunc(Int, _i)) .* (points[(trunc(Int, _i)+1)%size(points)[1], :] .- points[trunc(Int, _i), :])
            )
        end
    end

    return _rcoords
end

function points_filter(points, grid=nothing)
    _points = []
    _grid = grid !== nothing ? grid : minimum(minimum(u[2:length(u)] - u[1:length(u)-1] for u in [unique(c[:]) for c in eachcol(points)]))
    _cells = [[trunc(Int, (round.(_p[_d] / _grid))) for _d = 1:ndims(points)] for _p in eachrow(points)]
    _cells_copy = [[trunc(Int, (round.(_p[_d] / _grid))) for _d = 1:ndims(points)] for _p in eachrow(points)]
    points_l = points

    for _p in eachrow(points_l)
        _cell = [trunc(Int, (round.(_p[_d] / _grid))) for _d = 1:ndims(points)]

        _xr = -1:1
        _yr = -1:1

        x = sum([any([[_cell[1] + _x, _cell[2] + _y] ∈ _cells for _x in _xr]) for _y in _yr])
        y = sum([any([[_cell[1] + _x, _cell[2] + _y] ∈ _cells for _y in _yr]) for _x in _xr])

        if (x < 3) && (y < 3)
            # Return nearby points back to the loop
            for _xr in -1:1
                for _yr in -1:1
                    if _xr == _yr == 0
                        continue
                    end

                    _nearbyc = [_cell[1] + _xr, _cell[2] + _yr]

                    if _nearbyc in _cells_copy && points_l[findfirst(item -> item == _nearbyc, _cells_copy)] in _points
                        _nearbyp = points_l[findfirst(item -> item == _nearbyc, _cells_copy)]
                        filter!(e -> e ≠ _nearbyp, _points)
                        points_l = [points_l; _nearbyp]
                    end
                end
            end
            filter!(e -> e ≠ _cell, _cells)
        else
            _points = [_points; _p]
        end
    end

    return _points
end

function groups_border_obtain(groups, grid=nothing)
    _borders = []

    for (_i, _g) in enumerate(groups)
        _border = []
        _grid = grid !== nothing ? grid : grid_compute(_g)

        for _d in 1:ndims(_g)

            for _u in unique(sort(_g[:, _d]))
                temp = _g[findall(_g[:, _d] .== _u), :]
                push!(_border, minimum(temp, dims=1)[1:1, :])
                push!(_border, maximum(temp, dims=1)[1:1, :])

                _v = temp[1:1, :]
                _v = _v[:, 1:end.!=_d]

                sort!(_v, dims=1)

                # TODO: numpy.roll ?
                _dists = _v[2:end] .- _v[1:end-1]

                _bords = []

                if isempty(_dists) == false
                    _bords = first(findall(_dists .> (_grid * 1.5)))
                end

                for _b in _bords
                    push!(_border, _d == 1 ? [_u] .+ _v[_b] : _v[_b] .+ [_u])
                    push!(_border, _d == 1 ? [_u] .+ _v[_b+1] : _v[_b+1] .+ [_u])
                end
            end

        end
        push!(_borders, _border)
    end
    [unique.(b)[1] for b in eachrow(_borders)]
end

function groups_border_beautify(borders, border_length)
    bborders = []

    for (group_i, border) in enumerate(borders)
        # FIXME: Temporarily hidden as we are working with 0.02 map in Stage.
        border_filtered = points_filter(border)#0.05

        border_sorted = trajectory_sort(border_filtered, verify_sort=true)

        border_interpolated = interpolate(border_sorted, int_size=border_length)

        push!(bborders, border_interpolated)
    end

    return bborders
end

function groups_center_compute(_groups)
    mean(_groups, dims=1)
end

function group_layers_compute(layer0::Matrix{Float64}, layer0_center, layer_count::Int)

    layers_size = length(eachrow(layer0))
    a = [1 - (1 / 5) * layer_index for layer_index in 0:layer_count-1]
    points = [(layer0[:, 1:2] .- layer0_center) .* x .+ layer0_center for x in a]
    remains = (trunc.(Int, layers_size - (layers_size / layer_count) * layer_index for layer_index in 0:layer_count-1))

    [trajectory_reduce(points[i:i, :], remains[i]) for i in eachindex(remains)]
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
    # _groups = a
    # layers_center = groups_center_compute(a)
    # grouplayers = group_layers_compute(_groups, layers_center, 5)

    # grouplayers = groups_border_obtain(_groups)
    # grouplayers = interpolate(a, 400)
    x = [2 * i for i in 0:19]
    y = [i for i in 0:19]
    z = (x .- 1) .^ 2 .+ y .^ 2
    spline = Spline2D(x, y, z, s=1e-4)
    # println("result: ", spline(a[:, 1], a[:, 2]))
    for _c in eachrow(a)
        println(spline(_c[1], _c[2]))
    end
end
