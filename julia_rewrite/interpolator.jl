using Dierckx

function mapCreate(points::Array{Float64, 2}, int_size = 400, overflown...)

    _points = vcat(points, points[1:1, :])
    x, y = eachcol(_points)
    distance = vec(cumsum(sqrt.(sum(diff(_points, dims=1).^2, dims=2 )), dims = 1))
    distance = insert!(distance, 1, 0) ./ last(distance)

    alpha = range(0, stop=1, length=int_size + 1)[2:end-1]

    splinex = Spline1D(distance, x; periodic = true)
    spliney = Spline1D(distance, y; periodic = true)

    ipolx = splinex(alpha)
    ipol2x = derivative(splinex, alpha; nu=2)
    ipol1x = derivative(splinex, alpha; nu=1)

    ipoly = spliney(alpha)
    ipol2y = derivative(spliney, alpha; nu=2)
    ipol1y = derivative(spliney, alpha; nu=1)

    xx = (ipol1x .* ipol1y) .- (ipol1y .* ipol2x)
    yy = sqrt.((ipol1x .^2 .+ ipol1y .^2) .^3)

    hcat(ipolx, ipoly, xx./yy)
end
