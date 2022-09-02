using JSON
using NPZ
using Printf
using Plots

include("selector.jl")
include("interpolator.jl")
include("segmentator.jl")
include("criterions.jl")
include("optimizer.jl")

f_helper(x) = x
f_helper(x::Vector) = [f_helper(e) for e in x]
f_helper(d::Dict) = Dict(Symbol(k) => f_helper(v) for (k, v) in d)
symbol_dict(d::Dict) = f_helper(d)

function cascade_run(; track, fileformat, notification, loop_i, loop_output, conf...)
    # TODO: time

    _alg = merge(conf, loop_i[2])

    fitness, rcandidate, tcandidate, result = loop_output
    if fileformat !== nothing
        # TODO: fileformat
    end

    LOGFILE = stdout

    # update params
    selector_init(; merge(_alg, get(_alg, :selector_init, Dict()), Dict(:logfile => ""))...)

    # create map
    segmentator_init(track; merge(_alg, get(_alg, :segmentator_init, Dict()), Dict(:logfile => ""))...)

    # update params
    criterion_init()

    # construct matryoshka mapping
    optimizer_init(points=track, group_centers=rcandidate, group_centerline=result, logfile=LOGFILE; _alg...)

    # run GA
    _fitness, _rcandidate, _tcandidate, _result = optimize()

    # TODO: plot

    # TODO: fileformat

    if _fitness < fitness
        return (_fitness, _rcandidate, _tcandidate, _result)
    else
        return loop_output
    end
end

# TODO: loop
function loop_cascade_run(; track, initline, fileformat, notification, loop_i, loop_output=nothing, conf...)
    # TODO: time

    fitness = 10000000
    result = initline
    rcandidate = initline
    tcandidate = [0.5 for _ in 1:size(initline, 1)*2]

    _fileformat = nothing
    if fileformat !== nothing
        # _fileformat = @sprintf(fileformat, (loop_i + 1)) * @sprintf("-%%0%dd", length(string(length(conf["cascade"]))))
    end

    # notification = @sprintf(notification, (loop_i + 1)) * @sprintf(" Running step %%d/%d", length(conf["cascade"]))
    notification = @sprintf(" Running step %%d/%d", length(conf[:cascade]))

    # cascade_output = cascade_run(
    #     ;track=track,
    #     fileformat=_fileformat,
    #     notification=notification,
    #     merge(conf, "loop_i" => 1, "loop_output" => (fitness, rcandidate, tcandidate, result))...
    # )

    cascade_output = nothing
    for i in enumerate(conf[:cascade])
        cascade_output = cascade_run(
            track=track,
            fileformat=_fileformat,
            notification=notification,
            loop_i=i,
            loop_output=cascade_output === nothing ? (fitness, rcandidate, tcandidate, result) : cascade_output;
            conf...
        )
    end

    if loop_output === nothing || cascade_output[1] < loop_output[1]
        return cascade_output
    else
        return loop_output
    end
end

function execute(START_POINTS=nothing, VALID_POINTS=nothing)
    # TODO: time

    CONFIGURATION = JSON.parsefile("configuration/matryoshka_ex_ruudskogen.json")
    CONFIGURATION = symbol_dict(CONFIGURATION)

    if START_POINTS === nothing
        START_POINTS = npzread(CONFIGURATION[:start_points])
    end
    if VALID_POINTS === nothing
        VALID_POINTS = npzread(CONFIGURATION[:valid_points])
    end

    fileformat = nothing
    if haskey(CONFIGURATION, :prefix)
        fileformat = @sprintf "%s" CONFIGURATION[:prefix]
    end

    notification = ""

    solution = nothing
    if haskey(CONFIGURATION, :variate) && CONFIGURATION[:variate] in CONFIGURATION
        #TODO: variate
    else
        if fileformat !== nothing
            fileformat = fileformat * @sprintf("%%0%dd", length(string(CONFIGURATION[:loops])))
        end

        notification = notification * @sprintf("[%%d / %d]", CONFIGURATION[:loops])

        solution = loop_cascade_run(
            track=VALID_POINTS,
            initline=START_POINTS,
            fileformat=fileformat,
            notification=notification,
            loop_i=1;
            CONFIGURATION...
        )

        for i in 2:CONFIGURATION[:loops]
            solution = loop_cascade_run(
                track=VALID_POINTS,
                initline=START_POINTS,
                fileformat=fileformat,
                notification=notification,
                loop_i=i,
                loop_output=solution;
                CONFIGURATION...
            )
        end
    end

    scatter(VALID_POINTS[:, 1], VALID_POINTS[:, 2], markershape=:+, markersize=1)
    plot!(solution[4][:, 1], solution[4][:, 2])
    savefig("myplot.png")
    # TODO: time
    @printf("Optimization finished in .")

    return solution
end

if (abspath(PROGRAM_FILE) == @__FILE__)
    println(execute())
end
