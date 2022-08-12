using Dierckx

function mapCreate(points::Array{Float64, 2}, int_size = 400, overflown...)

    _points = vcat(points, points[1:1, :])
    # x, y = eachcol(_points)
    distance = vec(cumsum(sqrt.(sum(diff(_points, dims=1).^2, dims=2 )), dims = 1))
    distance = insert!(distance, 1, 0) ./ last(distance)

    alpha = range(0, stop=1, length=int_size + 1)[2:end-1]

    spline = Spline1D(x, y; periodic = true)

    ipol = spline(alpha)
    ipol2 = derivative(spline, alpha; nu=2)
    ipol1 = derivative(spline, alpha; nu=1)

    x = (ipol1[:, 0] .* ipol1[:, 1]) .- (ipol1[:, 1] .* ipol2[:, 0])
    y = sqrt.((ipol1[:, 0] .^2 .+ ipol1[:, 1] .^2) .^3)
    hcat(ipol, (x./y)[:,:])
end
