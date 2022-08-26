using JSON
using NPZ
using Printf

include("selector.jl")
include("interpolator.jl")
include("segmentator.jl")
include("criterions.jl")
include("optimizer.jl")

function cascade_run(; track, fileformat, notification, loop_i, loop_output, conf...)
    # TODO: time

    _alg = merge(conf, loop_i[1])

    fitness, rcandidate, tcandidate, result = loop_output
    if fileformat !== nothing
        # TODO: fileformat
    end

    selector_init(merge(_alg, get(alg, "selector_init", {}), "logfile" => "")...)
    # interpolator init
    segmentator_init(track, merge(_alg, get(alg, "segmentator_init", {}), "logfile" => "")...)
    criterion_init()
    optimizer_init(points = track, group_centers = rcandidate, group_centerline = result, logfile = LOGFILE)

    _fitne

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
    tcandidate = [[0.5, 0.5] for _ in 1:size(initline[1])]

    _fileformat = nothing
    if fileformat !== nothing
        _fileformat = @sprintf fileformat % (loop_i+1) * @sprintf "-%%0%dd" length(string(length(conf["cascade"])))
    end

    notification =  @sprintf notification % (loop_i+1) * @sprintf " Running step %%d/%d" length(conf["cascade"])

    cascade_output = cascade_run(
        ;elements = conf["cascade"],
        track=track,
        fileformat=_fileformat,
        notification=notification,
        merge(conf, "loop_output" => (fitness, rcandidate, tcandidate, result))...
    )

    if loop_output === nothing || cascade_output[1] < loop_output[1]
        return cascade_output
    else
        return loop_output
    end
end

function execute(START_POINTS=nothing, VALID_POINTS=nothing)
    # TODO: time

    CONFIGURATION = JSON.parsefile("configuration/matryoshka_ex_torino.json")

    if START_POINTS === nothing
        START_POINTS = npzread(CONFIGURATION["start_points"])
    end
    if VALID_POINTS === nothing
        VALID_POINTS = npzread(CONFIGURATION["valid_points"])
    end

    fileformat = nothing
    if haskey(CONFIGURATION, "prefix")
        fileformat = @sprintf "%s" CONFIGURATION["prefix"]
    end

    notification = ""

    solution = nothing
    if haskey(CONFIGURATION, "variate") && CONFIGURATION["variate"] in CONFIGURATION
        #TODO: variate
    else
        if fileformat !== nothing
            fileformat = fileformat * @sprintf "%%0%dd" length(string(CONFIGURATION["loops"]))
        end

        notification = notification * @sprintf "[%%d / %d]" CONFIGURATION["loops"]

        solution = loop_cascade_run(;
            elements=CONFIGURATION["loops"],
            track=VALID_POINTS,
            initline=START_POINTS,
            fileformat=fileformat,
            notification=notification,
            CONFIGURATION...
        )
    end

    # TODO: time
    @printf("Optimization finished in .")

    return solution
end
