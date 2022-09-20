using JSON
using NPZ
using Printf
using Plots
using Gnuplot

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

    # Get configuration for current step
    _alg = merge(conf, loop_i[2])

    # Rename output from previous stages
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
    _fitness, _rcandidate, _result = optimize()

    # TODO: plot

    # TODO: fileformat

    # Store only better solution for next steps of the cascade
    if _fitness < fitness
        return (_fitness, _rcandidate, _result)
    else
        return loop_output
    end
end

function loop_cascade_run(; track, initline, fileformat, notification, loop_i, loop_output=nothing, conf...)

    # Initial solution
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

    # Run cascade
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

    # TODO: print to file

    if loop_output === nothing || cascade_output[1] < loop_output[1]
        return cascade_output
    else
        return loop_output
    end
end

function variate_run(; fileformat, notification, loop_i, loop_output, conf...)

    # Local variables
    _i = loop_i[1]
    _param = loop_i[2][1]
    _value = loop_i[2][2]

    # Update logging file
    if fileformat !== nothing
        # Fill group count, and add format for number of loops

        # fileformat = @sprintf(fileformat, _value) * @sprintf("-%%0%dd", length(string(CONFIGURATION[:loops])))
    end

    # Update notification
    # Fill loop index, group count and prepare loops progress

    # notification = @sprintf(notification, _i + 1, _value, _param) * @sprintf(" [%%d / %d]", CONFIGURATION[:loops])

    ## Loop cascade
    for i in enumerate(conf[:cascade])
        cascade_output = cascade_run(
            elements=CONFIGURATION[:loops],
            fileformat=fileformat,
            notification=notification,
            loop_i=i,
            loop_output=cascade_output === nothing ? (fitness, rcandidate, tcandidate, result) : cascade_output;
            merge(Dict(_param => _value), conf)...
        )
    end

    @printf("Variating %s %s finished.", _param, _value)

    if loop_output === nothing || cascade_output[1] < loop_output[1]
        return cascade_output
    else
        return loop_output
    end
end

function execute(START_POINTS=nothing, VALID_POINTS=nothing)

    CONFIGURATION = JSON.parsefile(length(ARGS) >= 1 ? ARGS[1] : "configuration/matryoshka_ex_torino.json")

    CONFIGURATION = symbol_dict(CONFIGURATION)

    # Load data about the track
    if START_POINTS === nothing
        START_POINTS = npzread(CONFIGURATION[:start_points])
    end
    if VALID_POINTS === nothing
        VALID_POINTS = npzread(CONFIGURATION[:valid_points])
    end

    # Logging file format
    fileformat = nothing
    if haskey(CONFIGURATION, :prefix)
        fileformat = @sprintf("%s", CONFIGURATION[:prefix])
    end

    # Notification about progress
    notification = ""

    solution = nothing

    # Identify and prepare variating variable
    if haskey(CONFIGURATION, :variate) && CONFIGURATION[:variate] in CONFIGURATION
        param = CONFIGURATION[:variate]
        values = CONFIGURATION[Symbol(param)]

        # Force list
        if typeof(values) != Vector
            values = [values]
        end

        # Convert to tuples
        tvalues = [(param, value) for value in values]

        # Add variate to the file format
        if fileformat !== nothing
            if all(typeof(_value) == Vector for _value in values)
                fileformat = fileformat * @sprintf("%%0%dd", length(string(max(values))))
            else
                fileformat = fileformat * "-%s"
            end
        end

        # ... and also to the notification
        notification = notification * @sprintf("{%%d / %d (%%s %%s)}", length(values))

        ## And variate the parameter
        solution = nothing
        for i in enumerate(tvalues)
            solution = variate_run(
                track=VALID_POINTS,
                initline=START_POINTS,
                fileformat=fileformat,
                notification=notification,
                loop_i=i,
                loop_output=solution;
                CONFIGURATION...
            )
        end

    else
        # Skip to the loop
        # Update logging file
        if fileformat !== nothing
            fileformat = fileformat * @sprintf("%%0%dd", length(string(CONFIGURATION[:loops])))
        end

        # Update notification
        notification = notification * @sprintf("[%%d / %d]", CONFIGURATION[:loops])

        ## Loop cascade
        solution = nothing
        for i in 1:CONFIGURATION[:loops]
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
    plot!(solution[3][:, 1], solution[3][:, 2])
    savefig("myplot.png")
    @printf("Optimization finished.\n")

    return solution
end

if (abspath(PROGRAM_FILE) == @__FILE__)
    println(execute())
end
