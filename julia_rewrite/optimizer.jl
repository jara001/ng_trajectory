using Statistics
include("utils.jl")
include("interpolator.jl")

######################
# Utils
######################


function points_filter(points, grid=nothing)
    _points = []
    _grid = grid !== nothing ? grid : minimum(minimum(u[2:length(u)] - u[1:length(u)-1] for u in [unique!(c[:]) for c in eachcol(points)]))

    _cells = [Int(round(_p[_d] / _grid)) for _d = 1:ndims(points), _p in eachrow(points)]
    _cells_copy = [Int(round(_p[_d] / _grid)) for _d = 1:ndims(points), _p in eachrow(points)]

    points_l = eachrow(points)

    for _p in points_l
        _cell = [Int(round(_p[_d] / _grid)) for _d = 1:ndims(points)]

        _xr = -1:1
        _yr = -1:1

        x = sum([ any([ [ _cell[1] + _x, _cell[2] + _y ] ∈ _cells for _x in _xr ]) for _y in _yr ])
        y = sum([ any([ [ _cell[1] + _x, _cell[2] + _y ] ∈ _cells for _y in _yr ]) for _x in _xr ])

        if ( x < 3 ) && ( y < 3 )
            # Return nearby points back to the loop
            for _xr in -1:1
                for _yr in -1:1
                    if _xr == _yr == 0
                        continue
                    end

                    _nearbyc = [ _cell[1] + _xr, _cell[2] + _yr ]

                    if _nearbyc in _cells_copy && points_l[findfirst(_cells_copy, _nearbyc)] in _points
                        _nearbyp = points_l[findfirst(_cells_copy, _nearbyc)]
                        filter!(e->e≠_nearbyp, _points)
                        points_l = [points_l; _nearbyp]
                    end
                end
            end
            filter!(e->e≠_cell, _cells)
        else
            _points = [_points; _p]
        end
    end

    return _points
end

function groups_border_obtain(groups, grid=nothing)
    borders = []

    for (_i, _g) in enumerate(groups)
        _border = []
        _grid = grid !== nothing ? grid : grid_compute(_g)

        for _d in 1:ndims(_g)

            for _u in unique(_g[:, _d])
                temp = _g[findall(_g[:, _d] == _u), :]

                push!(border, minimum(temp, dims=2)[1])
                push!(border, maximum(temp, dims=2)[1])

                _v = temp[1:1, :]
                _v = _v[: , 1:end .!= _g]

                sort!(_v, dims = 1)

                # TODO: numpy.roll ?
                _dists = _v[2:end] .- _v[1:end-1]

                _bords = findall(_dists > (_grid * 1.5))[1]

                for _b in _bords
                    push!(border, _d == 0 ? [_u] .+ _v[_b] : _v[_b] .+ [_u])
                    push!(border, _d == 0 ? [_u] .+ _v[_b+1] : _v[_b+1] .+ [_u])
                end
            end

        end
        push!(borders, _border)
    end

    [unique!.(b) for b in _borders]
end

function groups_border_beautify(borders, border_length)
    bborders = []

    for (group_i, border) in enumerate(borders)
        # FIXME: Temporarily hidden as we are working with 0.02 map in Stage.
        border_filtered = points_filter(border)#0.05

        border_sorted = trajectory_sort(border_filtered, verify_sort = true)

        border_interpolated = interpolate(border_sorted, border_length)

        push!(bborders, border_interpolated)
    end

    return bborders
end

function groups_center_compute(groups)
    centers = []
    for _g in groups
        push!(centers, mean(_g, dims = 1))
    end
    _centers
end

function group_layers_compute()
    
end
