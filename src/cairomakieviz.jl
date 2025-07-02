struct MakieLaserTagElements
    fig::Figure
    ax::Axis
    obstacle_scatter
    belief_hm
    agent_scatter
    opponent_scatter
    laser_segs::Vector{Observable{Vector{Point2f}}}
    
    # observables backing dynamic data:
    agent_pos::Observable{Point2f}
    opponent_pos::Observable{Point2f}
    belief_data::Observable{Matrix{Float64}}
end

function setup_lasertag_plot(pomdp::LaserTagPOMDP)
    F = pomdp.floor
    nR, nC = F.n_rows, F.n_cols

    fig = Figure(size = (600,600))
    ax  = Axis(fig[1,1],
               title       = "LaserTag Sim",
               xlabel      = "Col",
               ylabel      = "Row",
               aspect      = DataAspect(),
               yreversed   = true)
    xlims!(ax, 0.5, nC + 0.5)
    ylims!(ax, 0.5, nR + 0.5)

    # grid
    for i in 0:nR
        lines!(ax, [0.5, nC+0.5], [i+0.5, i+0.5], linewidth = 0.5, color = :gray)
    end
    for j in 0:nC
        lines!(ax, [j+0.5, j+0.5], [0.5, nR+0.5], linewidth = 0.5, color = :gray)
    end

    # obstacles
    obs_pts = Point2f[]
    for (x,y) in pomdp.obstacles
        push!(obs_pts, Point2f(x, y))
    end
    obs_scatter = scatter!(ax, obs_pts;
                           marker    = :rect,
                           markersize= 40,
                           color     = :black,
                           label     = "Obstacle")

    # belief heatmap (initially zeros / hidden)
    belief_data = Observable(zeros(Float64, nC, nR))
    belief_vis  = Observable(false)
    hm = heatmap!(ax, 1:nC, 1:nR, belief_data;
                  colormap = :Oranges,
                  visible  = belief_vis,
                  alpha    = 0.6)
    Colorbar(fig[1,2], hm, label = "Belief")

    # robot & opponent
    agent_pos    = Observable(Point2f(1,1))
    opponent_pos = Observable(Point2f(1,1))
    agent_s      = scatter!(ax, agent_pos; marker=:circle,   markersize=40, color=:green, label="Robot")
    opp_s        = scatter!(ax, opponent_pos; marker=:star5, markersize=40, color=:orange, label="Opponent")

    # laser rays (8 segments), initially empty
    laser_segs = [ Observable(Point2f[]) for _ = 1:8 ]
    for seg in laser_segs
        lines!(ax, seg; linestyle=:dash, linewidth=2, color=:red)
    end

    Legend(fig[2,1:2], [agent_s, opp_s, obs_scatter], 
           ["Robot","Opponent","Obstacle"], 
           orientation = :horizontal, tellheight=true)

    return MakieLaserTagElements(fig, ax, obs_scatter, hm, agent_s, opp_s,
                                 laser_segs, agent_pos, opponent_pos, belief_data)
end

function update_lasertag_plot!(
      E::MakieLaserTagElements,
      pomdp::LaserTagPOMDP,
      state::LTState,
      obs::Union{CMeas,DMeas},
      belief::Union{AbstractParticleBelief,Nothing};
      show_belief::Bool = true
    )
    # 1) update positions
    E.agent_pos[]    = Point2f(state.robot...)
    E.opponent_pos[] = Point2f(state.opponent...)

    # 2) update belief heatmap
    if show_belief && belief !== nothing
        # build a nC×nR matrix of normalized counts over opponent coords
        F = pomdp.floor  # or pass in floor dims separately
        mat = zeros(Float64, F.n_cols, F.n_rows)
        counts = Dict{Coord,Int}()
        total = 0
        for p in particles(belief)
            if !p.terminal
                counts[p.opponent] = get(counts,p.opponent,0) + 1
                total += 1
            end
        end
        for ((x,y), c) in counts
            mat[x,y] = c/total
        end
        E.belief_data[] = mat
        E.belief_hm.visible[] = true
    else
        E.belief_hm.visible[] = false
    end

    # 3) update lasers
    # first clear them
    for seg in E.laser_segs
        seg[] = Point2f[]
    end
    middle = (state.robot)
    # cardinal
    for i in 1:4
        dir = CARDINALS[i]
        start  = middle .+ 0.5 .* dir
        finish = start   .+ obs[i] .* dir
        E.laser_segs[i][] = [Point2f(start...), Point2f(finish...)]
    end
    # diagonal
    for i in 1:4
        dir = DIAGONALS[i] .* (√2/2)
        start  = middle .+ 0.5*√2 .* dir
        finish = start   .+ obs[i+4] .* dir
        E.laser_segs[i+4][] = [Point2f(start...), Point2f(finish...)]
    end

    return nothing
end
