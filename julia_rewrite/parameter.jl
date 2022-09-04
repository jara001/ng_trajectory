# module ParameterListClass

# export ParameterList, add_parameter!, to_string, get_value, reset!, reset_all!, iterate, update!, update_all!

# module ParameterClass

# export Parameter, reset_p!, to_string_par

mutable struct Parameter
    name::String
    default
    value
    type
    description::String
    group::String
end

function reset_p!(this::Parameter)
    this.value = this.default
    return Nothing
end

function to_string_par(this::Parameter)
    string(this.name, " (", this.type, ") ", this.value, " [", this.description, "]")
end

# end #MODULE

# using .ParameterClass

mutable struct ParameterList
    parameters::Dict
    ParameterList() = new(Dict())
end

function add_parameter!(this::ParameterList, p::Parameter)
    this.parameters[p.name] = p
    return nothing
end

function get_value(this::ParameterList, name::String)
    get(this.parameters, name, nothing).value
end

function reset!(this::ParameterList, name::String)
    reset_p!(get(this.parameters, name, nothing))
    return nothing
end

function reset_all!(this::ParameterList)
    for _p in this.parameters
        reset!(this, _p.first)
    end
    return nothing
end

function iterate(this::ParameterList)
    pairs(this)
end

function update!(this::ParameterList, name, value)
    get(this.parameters, string(name), nothing).value = value
    return nothing
end

function update_all!(this::ParameterList, kwargs; reset::Bool=true)
    if reset == true
        reset_all!(this)
    end

    for (_p, _v) in kwargs
        if haskey(this.parameters, _p)
            update!(this, _p, _v)
        end
    end
    return nothing
end

function to_string(this::ParameterList)
    string("\t", join([to_string_par(parameter) for parameter in values(this.parameters)]))
end

# end #MODULE


if (abspath(PROGRAM_FILE) == @__FILE__)
    # using .ParameterListClass.ParameterClass
    # using .ParameterListClass
    a = Parameter("sampling_distance", 1.0, 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "init")
    println(to_string_par(a))
    P = ParameterList()
    add_parameter!(P, a)
    println(get_value(P, "sampling_distance"))
    println(to_string(P))
end
