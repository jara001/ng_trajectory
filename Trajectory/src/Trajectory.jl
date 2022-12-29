module Trajectory

using Revise

include("parameter.jl")
include("utils.jl")
include("selector.jl")
include("interpolator.jl")
include("segmentator.jl")
include("criterions.jl")
include("optimizer.jl")
include("penalizer.jl")
include("run.jl")

export trajectory_resample
export P_sel
export profile_compute
export trajectory_closest_index
export trajectory_time_resample
export jazar_profile_compute
export P_cri
export execute
export optimizer_init
export set_maps

end # module Trajectory
