using LaserTag
using Random
using POMDPTools
using ParticleFilters
using POMDPs

p = gen_lasertag()

pol = RandomPolicy(p, rng=StableRNG(1))

sim = HistoryRecorder(max_steps=10, rng=StableRNG(2))

filter = BootstrapFilter(p, 10000)

hist = simulate(sim, p, pol, filter)

tikz_pic(LaserTagVis(p))

E = setup_lasertag_plot(my_laser_tag_pomdp)
display(E.fig)

# — for each step: —
state, obs, belief = …  # however you extract them
update_lasertag_plot!(E, state, obs, belief; show_belief=true)