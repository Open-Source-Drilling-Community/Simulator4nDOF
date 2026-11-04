using GLMakie

function plot_lumped_elements_shape()
    fig = Figure()
    L = 10
    ξ(x) = x / L 
    M1(x) = sin(π * ξ(x)) 
    M2(x) = 1 - sin(π * ξ(x))

    N1(x) = 1 - ξ(x)
    N2(x) = ξ(x)

    ax = Axis(fig[1, 1], xlabel="x", ylabel="Interpolation function", title="Lateral element")
    lines!(ax, 0..L, M1, color = :black, label="first element")
    lines!(ax, 0..L, M2, color = :blue, label="second element")
    ax2 = Axis(fig[2, 1], xlabel="x", ylabel="Interpolation function", title="Axial/torsional element")
    lines!(ax2, 0..L, N1, color = :black, label="first element")
    lines!(ax2, 0..L, N2, color = :blue, label="second second element")

    axislegend(ax)
    display(fig)
end
plot_lumped_elements_shape()