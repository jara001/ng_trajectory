using Statistics
using Dierckx
using Printf
using Evolutionary
using Metaheuristics
using Gnuplot
using PyCall

include("utils.jl")
include("interpolator.jl")
include("segmentator.jl")
include("selector.jl")
include("criterions.jl")
include("parameter.jl")
include("penalizer.jl")

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
BUDGET = nothing
NUM_WORKERS = Sys.CPU_THREADS

mutable struct TrackBitmap
    const p_min::Vector{Float64} # const here needs Julia 1.8
    const p_max::Vector{Float64}
    const p_step::Vector{Float64}
    bitmap::BitArray{2}
    TrackBitmap(p_min, p_max, p_step) = new(p_min, p_max, p_step,
                                            falses((cld.(p_max .- p_min, p_step) .|> Int) + [1, 1] |> Tuple))
end

function index(tb::TrackBitmap, point)
    idx = ((point .- tb.p_min) .÷ tb.p_step .|> Int) + [1, 1]
end

Base.setindex!(tb::TrackBitmap, x, pt) = tb.bitmap[index(tb, pt)...] = x
function Base.getindex(tb::TrackBitmap, pt)
    idx = index(tb, pt)
    if any(idx .< (1,1)) || any(idx .> size(tb.bitmap))
        return false
    end
    return tb.bitmap[idx...]
end

TRACK_BITMAP = nothing

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
    budget::Int=10,
    layers::Int=5,
    groups::Int=12,
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

    global OPTIMIZER, MATRYOSHKA, VALID_POINTS, LOGFILE, VERBOSITY, HOLDMAP, GRID, PENALTY, FIGURE, PLOT, BUDGET, NUM_WORKERS
    global CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, SEGMENTATOR, SEGMENTATOR_ARGS, SELECTOR, SELECTOR_ARGS, PENALIZER, PENALIZER_INIT, PENALIZER_ARGS

    # Local to global variables
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
    BUDGET = budget
    NUM_WORKERS = workers


    VALID_POINTS = points
    if MATRYOSHKA === nothing || _holdmatryoshka == false
        # Note: In version <=1.3.0 the group_centerline passed to the SELECTOR was sorted using
        #       ng_trajectory.interpolators.utils.trajectorySort, but it sometimes rotated the
        #       already sorted centerline; interestingly, the result was counterclockwise at all
        #       times (or at least very very often).

        group_centers = select(group_centerline, groups; SELECTOR_ARGS...)

        if plot == true
            # TODO: plot
        end

        # Matryoshka construction
        _groups = segmentate(points, group_centers; SEGMENTATOR_ARGS...)

        grouplayers = groups_border_obtain(_groups)
        grouplayers = groups_border_beautify(grouplayers, 400)

        if plot == true
            # TODO: plot
        end

        layers_center = groups_center_compute(_groups)
        layers_count = [layers for _ in 1:length(grouplayers)]

        MATRYOSHKA = [matryoshka_create(grouplayers[_i], layers_center[_i], layers_count[_i]) for _i in 1:length(_groups)]

        # TODO: plot

        println("Matryoshka mapping constructed.")

        if GRID === nothing
            GRID = grid_compute(points)
        end
    end
end

using VideoIO
import FileIO
using FixedPointNumbers

WRITER = nothing

function plot_population(population, value; video=false)
    n = length(MATRYOSHKA)
    @gp tit="Best value: $(value)" "set size ratio -1" :-
    @gp :- VALID_POINTS[:, 1] VALID_POINTS[:, 2] "w p pt 1 lc rgbcolor '0xeeeeee' notitle" :-
    foreach(population) do p
        points01 = reshape(p, (n, 2))
        points = [matryoshka_map(MATRYOSHKA[i], [p])[1] for (i, p) in enumerate(eachrow(points01))]
        _points = interpolate(mapreduce(permutedims, vcat, points))
        @gp :- _points[:, 1] _points[:, 2] "w l notitle" :-
    end
    @gp :- ""
    save(term="pngcairo size 1280, 720 fontscale 0.8", output="frame.png")
    write(WRITER, FileIO.load("frame.png"))
end

function Evolutionary.trace!(record::Dict{String,Any}, objfun, state, population, method, options)
    #plot_population(population, value(state), video=true)
end

function Evolutionary.after_while!(objfun, state, method, options)
    global STATE = state # allow investigating the internal state
                         # after the optimization ends
end

function optimize_evolutionary()
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE, PLOT, PENALIZER, PENALIZER_ARGS
    n = length(MATRYOSHKA)
    constr = BoxConstraints(zeros(2n), ones(2n))
    x0 = fill(0.5, 2n)
    #method = Evolutionary.GA(populationSize=30, selection=uniformranking(10), mutation=gaussian(0.1), crossover=TPX)
    #method = Evolutionary.CMAES(sigma0=0.1, c_1=0.01, c_mu=0.001, c_sigma=0.02)
    method = Evolutionary.CMAES(sigma0=0.1)
    res = Evolutionary.optimize(opt, constr, x0, method,
                                Evolutionary.Options(iterations=1000,
                                                     #parallelization=:thread,
                                                     show_trace=true,
                                                     store_trace=true,
                                                     reltol=1e-6,
                                                     ))
    println(res)
    global RESULT = res
    trace = Evolutionary.trace(res)
    @gp :trace value.(trace)[10:end] "w l" "set grid"

    points01 = reshape(Evolutionary.minimizer(res), (n, 2))
end

function optimize_metaheuristics()
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE, PLOT, PENALIZER, PENALIZER_ARGS
    x0 = [0.5 for _ in 1:length(MATRYOSHKA)*2]
    bounds = repeat([0.0, 1.0], 1, length(MATRYOSHKA) * 2)
    ga = Metaheuristics.GA(;
                           crossover=Metaheuristics.OrderCrossover(),
                           mutation=Metaheuristics.SlightMutation())
    points01 = Metaheuristics.optimize(_opt, bounds, ga)
end

function optimize_nevergrad()
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE, PLOT, PENALIZER, PENALIZER_ARGS

    num_rows = length(MATRYOSHKA)

    nevergrad = pyimport("nevergrad")
    # concurrent = pyimport("concurrent")
    # multiprocessing = pyimport("multiprocessing")

    # Optimizer definition
    instrum = nevergrad.Instrumentation(nevergrad.var.Array(num_rows, 2).bounded(0, 1))
    OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation=instrum, budget=BUDGET)
    # OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation=instrum, budget=BUDGET, num_workers=NUM_WORKERS)

    # @pywith concurrent.futures.ProcessPoolExecutor(max_workers=OPTIMIZER.num_workers, mp_context=multiprocessing.get_context("fork")) as executor begin
    #     recommendation = OPTIMIZER.minimize(_opt, executor=executor, batch_mode=false)
    # end
    recommendation = OPTIMIZER.minimize(_opt, batch_mode=false)
    points01 = convert(Array{Float64,2}, recommendation.args[1])
end

function optimize()
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE, PLOT, PENALIZER, PENALIZER_ARGS

    encoder_options = (crf=23, preset="medium")

    global WRITER
    WRITER = open_video_out("video.mp4", RGB{N0f8}, (720, 1280),
                            framerate=4, encoder_options=encoder_options)

    points01 = optimize_evolutionary()

    points = [matryoshka_map(MATRYOSHKA[i], [p])[1] for (i, p) in enumerate(eachrow(points01))]

    PENALIZER_ARGS[:optimization] = false
    final = _opt(points01)
    plot_population([points01], final)

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = interpolate(mapreduce(permutedims, vcat, points))


    close_video_out!(WRITER)

    lock(FILELOCK) do
        if VERBOSITY > 0
            @printf(LOGFILE, "solution:%s\n", string(points))
            @printf(LOGFILE, "final:%f\n", final)
        end
    end

    return (final, points, _points)
end

prepare_points(points::Array{Float64, 2}, matr_len) = convert(Array{Float64,2}, points) # Nevergrad
prepare_points(points::Vector{Float64}, matr_len) = reshape(points, (matr_len, 2)) # Evolutionary

function opt(points)
    #@time _opt(points)
    _opt(points)
end

function _opt(points)
    global VALID_POINTS, CRITERION_ARGS, INTERPOLATOR_ARGS, PENALIZER_ARGS
    global MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, GRID, PENALTY

    points = prepare_points(points, length(MATRYOSHKA))

    # Transform points
    points = [matryoshka_map(MATRYOSHKA[i], [p])[1] for (i, p) in enumerate(eachrow(points))]
    _points = interpolate(mapreduce(permutedims, vcat, points); INTERPOLATOR_ARGS...)

    # Check the correctness of the points and compute penalty
    penalty = penalize(_points, VALID_POINTS, GRID, PENALTY; PENALIZER_ARGS...)

    if penalty != 0
        if VERBOSITY > 0
            lock(FILELOCK) do
                if VERBOSITY > 2
                    @printf(LOGFILE, "pointsA:%s\n", string(points))
                    @printf(LOGFILE, "pointsT:%s\n", string(_points))
                end
                if VERBOSITY > 1
                    @printf(LOGFILE, "penalty:%f\n", penalty)
                end
                flush(LOGFILE)
            end
        end
        return Float64(penalty)
    end

    _c = compute(_points; CRITERION_ARGS...)
    lock(FILELOCK) do
        if VERBOSITY > 2
            @printf(LOGFILE, "pointsA:%s\n", string(points))
            @printf(LOGFILE, "pointsT:%s\n", string(_points))
        end
        if VERBOSITY > 1
            @printf(LOGFILE, "correct:%f\n", _c)
        end
        flush(LOGFILE)
    end
    return _c
end

function matryoshka_create(layer0, layer0_center, layer_count::Int)
    layers = group_layers_compute(layer0, layer0_center, layer_count)
    layer0_size = size(layer0, 1)

    # Method, where we learn the interpolator all layers at once and use that information
    # for computing of the transformation
    # Note: This is much more precise.

    _tc = Array{Float64}(undef, 0, 2)
    _rc = Array{Float64}(undef, 0, 2)

    for _layer_index in 0:layer_count-1
        layer_params = layer_index_to_parameters(_layer_index, layer0_size, layer_count)
        indices = 1:layer_params[1]
        tc = indices_to_transformed_coordinates(indices, layer_params...)
        rc = indices_to_real_coordinates(indices, layers[Int(_layer_index)+1])

        _tc = vcat(_tc, mapreduce(permutedims, vcat, tc))
        _rc = vcat(_rc, mapreduce(permutedims, vcat, rc))
    end

    # transformedCoordinatesToRealCoordinates
    _ip2d = []

    for _d in axes(_rc, 2)
        push!(_ip2d, Spline2D(_tc[:, 1], _tc[:, 2], _rc[:, _d]; s=length(_tc[:, 1]) - sqrt(2 * length(_tc[:, 1]))))
    end

    return _ip2d
end

function matryoshka_map(matryoshka, coords)

    _rcoords = []
    for _c in coords
        _dims = []
        for _interpolator in matryoshka
            push!(_dims, _interpolator(_c[1], _c[2]))
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
    # _points = Array{Float64}(undef, 0, 2)
    _points = []
    _grid = grid !== nothing ? grid : minimum(minimum(u[2:length(u)] - u[1:length(u)-1] for u in [unique(sort(c[:])) for c in eachcol(points)]))
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
                        push!(points_l, _nearbyp)
                    end
                end
            end
            filter!(e -> e ≠ _cell, _cells)
        else
            push!(_points, _p)
        end
    end

    return _points
end

function groups_border_obtain(groups, grid=nothing)

    _borders = []

    for (_i, _g) in enumerate(groups)
        _border = Array{Float64}(undef, 0, 2)

        # Obtain grid size if not set
        _grid = grid !== nothing ? grid : minimum(minimum(u[2:end] - u[1:end-1] for u in [unique(sort(_g[:, d])) for d in 1:size(_g)[2]]))

        # Go through all dimensions
        for _d in 1:ndims(_g)

            # Find unique values in that dimension
            for _u in unique(sort(_g[:, _d]))
                temp = _g[findall(_g[:, _d] .== _u), :]
                # Append points with max / min values in another dimension
                _border = vcat(_border, minimum(temp, dims=1))
                _border = vcat(_border, maximum(temp, dims=1))

                # Append inner borders
                # Obtain values in the dimension
                _v = _g[findall(_g[:, _d] .== _u), :]
                _v = _v[:, 1:end.!=_d]

                # Sort them
                # Enforce the axis as otherwise it is not sorted in ascending order everytime.
                _v = _v[sortperm(_v[:, 1]), :]

                # Find distances between concurrent points
                _dists = circshift(_v, (1, 0))[2:end, :] .- _v[1:end-1, :]

                # Find points in the distance larger than 1.5x _grid
                _bords = findall(_dists .> (_grid * 1.5))

                for _b in _bords
                    _border = vcat(_border, _d == 1 ? [_u] .+ _v[_b] : _v[_b] .+ [_u])
                    _border = vcat(_border, _d == 1 ? [_u] .+ _v[_b+1] : _v[_b+1] .+ [_u])
                end
            end

        end
        push!(_borders, _border)
    end
    [unique(sortslices(b, dims=1, by=x -> (x[1], x[2])), dims=1) for b in _borders]
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
    _centers = []

    for _g in _groups
        push!(_centers, mean(_g, dims=1))
    end

    _centers
end

function group_layers_compute(layer0::Matrix{Float64}, layer0_center, layer_count::Int)

    layers_size = size(layer0, 1)
    points = [(layer0[:, 1:2] .- layer0_center) .* (1 - (1 / layer_count) * layer_index) .+ layer0_center for layer_index in 0:layer_count-1]
    remains = (trunc.(Int, layers_size - (layers_size / layer_count) * layer_index for layer_index in 0:layer_count-1))

    [trajectory_reduce(points[i], remains[i]) for i in eachindex(remains)]
end


if (abspath(PROGRAM_FILE) == @__FILE__)
    # grouplayers = interpolate(a, 400)
    # x = [2 * i for i in 0:19]
    # y = [i for i in 0:19]
    # z = (x .- 1) .^ 2 .+ y .^ 2
    # spline = Spline2D(x, y, z, s=1e-4)
    # # println("result: ", spline(a[:, 1], a[:, 2]))
    # for _c in eachrow(a)
    #     println(spline(_c[1], _c[2]))
    # end
end
