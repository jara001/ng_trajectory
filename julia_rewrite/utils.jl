using JSON
using PyCall

function gridCompute(points)
    minimum(minimum(u[2:length(u)] - u[1:length(u)-1] for u in [unique!(c[:]) for c in eachcol(points)]))
end

function mapCreate(points::Array{Float64, 2}, origin = Nothing, size = Nothing, grid = Nothing)
    println("Creating map...")

    _grid = grid != Nothing ? grid : gridCompute(points)
    println("\tGrid:", _grid)

    _origin = origin != Nothing ? origin : minimum(points, dims = 1)

    println("\tOrigin:", _origin)

    _size = size != Nothing ? size : map(abs, maximum(points, dims = 1) - minimum(points, dims = 1))

    println("\tMin:", minimum(points, dims = 1))
    println("\tMax:", maximum(points, dims = 1))
    println("\tDist:", maximum(points, dims = 1) - _origin)

    println("\tSize:", _size)
    println("\tCell size:", (_size ./ _grid) .+ 1, convert.(UInt64, (_size ./ _grid) .+ 1))

    _m = Array{UInt8, 2}(undef, Tuple(convert.(UInt64, (_size ./ _grid) .+ 1)));

    for _p in eachrow(points)
        println("origin:", typeof(_origin), _origin)
        println("_p:", typeof(reshape(_p[1:2], (1, 2))), _p)
        _m[Tuple(convert.(UInt64, map(round, (reshape(_p[1:2], (1, 2)) - _origin) ./ _grid)))] = 100
        break;
    end

    println("Map created.")

end


if (abspath(PROGRAM_FILE) == @__FILE__)
    println("Hello computer. My name is Dias. I'm your human. I am writing julia code. Thank you for running it.")
    CONFIGURATION = JSON.parsefile("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/matryoshka_ex_torino.json")

    # START_POINTS = Array{Float64}(undef, 440, 2)
    # VALID_POINTS = Array{Float64}(undef, 63959, 2)

    py"""
       import numpy
       start_points = numpy.load("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/ng_start_points_torino2.npy")
       valid_points = numpy.load("/mnt/c/Users/User/OneDrive/Рабочий стол/ng/configuration/ng_valid_points_torino2.npy")
       """

    # read!(CONFIGURATION["start_points"], START_POINTS)
    # read!(CONFIGURATION["valid_points"], START_POINTS)

    START_POINTS = convert(Array{Float64, 2}, PyArray(py"start_points"o))
    VALID_POINTS = convert(Array{Float64, 2}, PyArray(py"valid_points"o))

    mapCreate(VALID_POINTS)
    # VALID_POINTS = PyArray(py"b"o)

end
