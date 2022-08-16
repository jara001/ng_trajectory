using JSON
using PyCall

#

function grid_compute(points)
    minimum(minimum(u[2:length(u)] - u[1:length(u)-1] for u in [unique!(c[:]) for c in eachcol(points)]))
end

function map_create(points::Array{Float64, 2}, origin = Nothing, size = Nothing, grid = Nothing)
    println("Creating map...")

    _grid = grid != Nothing ? grid : gridCompute(points)
    println("\tGrid:", _grid)

    _origin = origin != Nothing ? origin : reshape(minimum(points, dims = 1), (2, 1))

    println("\tOrigin:", _origin)

    _size = size != Nothing ? size : reshape(map(abs, maximum(points, dims = 1) - minimum(points, dims = 1)), (2, 1))

    println("\tMin:", reshape(minimum(points, dims = 1), (2, 1)))
    println("\tMax:", reshape(maximum(points, dims = 1), (2, 1)))
    println("\tDist:", reshape(maximum(points, dims = 1), (2, 1)) - _origin)

    println("\tSize:", _size)
    println("\tCell size:", (_size ./ _grid) .+ 1, convert.(UInt64, (_size ./ _grid) .+ 1))

    _m = zeros(UInt8, Tuple(convert.(UInt64, (_size ./ _grid) .+ 1)));

    for _p in eachrow(points)
        index = convert.(UInt64, map(round, ((_p[1:2] - _origin) ./ _grid) .+ 1))
        _m[index[1], index[2]] = 100
        break;
    end

    println("Map created.")

end

######################
# Utilities (Point)
######################

function point_distance(a, b)
    sqrt(sum([(b[i] - a[i])^2 for i in range(1, stop = min(length(a), length(b)))]))
end

######################
# Utilities (Trajectory)
######################

function trajectory_closest_index(points, reference; from_left::Bool = false)
    _distances = points[:, 1:2] .- reference[1:2]'
    index = argmin(hypot.(_distances[:,1], _distances[:2]), dims = 1)[1]

    if from_left == false
        return index
    else
        d1 = hypot(_distances[index, 1], distances[index, 2])
        d2 = hypot(_distances[index + 1, 1], distances[index + 1, 2])
        ds = point_distance(points[index, 1:2], points[index + 1, 1:2])
        return (d1^2 - d2^2 + ds^2) / (2 * d1 * ds) > 0 ? index : index - 1
    end
end


if (abspath(PROGRAM_FILE) == @__FILE__)
    # CONFIGURATION = JSON.parsefile(ARGS[1])

    # # START_POINTS = read("start_points.bin")
    # # VALID_POINTS = read("valid_points.bin")

    # START_POINTS = Array{Float64}(undef, 440, 2)
    # VALID_POINTS = Array{Float64}(undef, 63959, 2)
    # read!("start_points.bin", START_POINTS)
    # read!("valid_points.bin", VALID_POINTS)

    # # py"""
    # #    import numpy
    # #    start_points = numpy.load("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/ng_start_points_torino2.npy")
    # #    valid_points = numpy.load("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/ng_valid_points_torino2.npy")
    # #    """

    # # START_POINTS = convert(Array{Float64, 2}, PyArray(py"start_points"o))
    # # write("start_points.bin", START_POINTS)

    # # VALID_POINTS = convert(Array{Float64, 2}, PyArray(py"valid_points"o))
    # # write("valid_points.bin", VALID_POINTS)

    # mapCreate(VALID_POINTS)

    # # VALID_POINTS = PyArray(py"b"o)

    a = [ 0.16433    0.524746;
        0.730177   0.787651;
        0.646905   0.0135035;
        0.796598   0.0387711;
        0.442782   0.753235;
        0.832315   0.483352;
        0.442524   0.912381;
        0.336651   0.236891;
        0.0954936  0.303086;
        0.459189   0.374318]
    b =  [0.7589091211161472,
        0.8091539348190575,
        0.5256478329286531,
        0.41357337873861466]

    println(typeof(trajectory_closest_index(a, b)))
end
