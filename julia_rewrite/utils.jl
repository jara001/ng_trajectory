using JSON
using PyCall

function gridCompute(points)
    minimum(minimum(u[2:length(u)] - u[1:length(u)-1] for u in [unique!(c[:]) for c in eachcol(points)]))
end

function mapCreate(points::Array{Float64, 2}, origin = Nothing, size = Nothing, grid = Nothing)
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

function trajectory_closest_index(points, reference; from_left::bool = false)
    _distances = points[:, 1:2] .- reference[1:2]

end


if (abspath(PROGRAM_FILE) == @__FILE__)
    CONFIGURATION = JSON.parsefile(ARGS[1])

    # START_POINTS = read("start_points.bin")
    # VALID_POINTS = read("valid_points.bin")

    START_POINTS = Array{Float64}(undef, 440, 2)
    VALID_POINTS = Array{Float64}(undef, 63959, 2)
    read!("start_points.bin", START_POINTS)
    read!("valid_points.bin", VALID_POINTS)

    # py"""
    #    import numpy
    #    start_points = numpy.load("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/ng_start_points_torino2.npy")
    #    valid_points = numpy.load("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/ng_valid_points_torino2.npy")
    #    """

    # START_POINTS = convert(Array{Float64, 2}, PyArray(py"start_points"o))
    # write("start_points.bin", START_POINTS)

    # VALID_POINTS = convert(Array{Float64, 2}, PyArray(py"valid_points"o))
    # write("valid_points.bin", VALID_POINTS)

    mapCreate(VALID_POINTS)
    # VALID_POINTS = PyArray(py"b"o)

end
