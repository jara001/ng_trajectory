using .ParameterListClass.ParameterClass
using .ParameterListClass

P = ParameterList()
add_parameter!(P, Parameter("sampling_distance", 1.0, 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "init"))
add_parameter!(P, Parameter("distance", 0, 0, float, "[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.", "init"))
add_parameter!(P, Parameter("rotate", 0, 0, float, "Parameter for rotating the input path. 0 is not rotated. <0, 1)", "init"))
add_parameter!(P, Parameter("fixed_points", [], [], list, "Points to be used in the selection upon calling 'select'.", "init"))

function trajectoryResample(points, remain)
    if points[1, :1] == points[end, :1]
        points = points[1:end-1, :]
    end

    raw_fixed_points = copy(get_value(P, "fixed_points"))
    rpoints = []
    fixed_points = []
    upoints = []

    rotate = typeof(get_value(P, "rotate")) != Vector ? [getValue(P, "rotate") for _ in range(1, maximum(1, length(get_value(P, "fixed_points"))))] : copy(get_value(P, "rotate"))

    while true
        
    end
end
