"""
    estimate_value(estimator, mdp, state, depth)
Return an estimate of the value.
"""
function estimate_value end
estimate_value(f::Function, mdp::Union{POMDP,MDP}, state, depth::Int) = f(mdp, state, depth)
estimate_value(estimator::Number, mdp::Union{POMDP,MDP}, state, depth::Int) = convert(Float64, estimator)

"""
RolloutEstimator
If this is passed to the estimate_value field of the solver, rollouts will be used to estimate the value at the leaf nodes
Fields:
    solver::Union{Solver,Policy,Function}
        If this is a Solver, solve(solver, mdp) will be called to find the rollout policy
        If this is a Policy, the policy will be used for rollouts
        If this is a Function, a POMDPToolbox.FunctionPolicy with this function will be used for rollouts
"""
mutable struct RolloutEstimator
    solver::Union{Solver,Policy,Function} # rollout policy or solver
end

"""
SolvedRolloutEstimator
This is within the policy when a RolloutEstimator is passed to an AbstractDecMCTSSolver
"""
mutable struct SolvedRolloutEstimator{P<:Policy, RNG<:AbstractRNG}
    policy::P
    rng::RNG
end

convert_estimator(ev, solver, mdp) = ev
function convert_estimator(ev::RolloutEstimator, solver::AbstractDecMCTSSolver, mdp::Union{POMDP,MDP})
    return SolvedRolloutEstimator(convert_to_policy(ev.solver, mdp), solver.rng)
end
convert_to_policy(p::Policy, mdp::Union{POMDP,MDP}) = p
convert_to_policy(s::Solver, mdp::Union{POMDP,MDP}) = solve(s, mdp)
convert_to_policy(f::Function, mdp::Union{POMDP,MDP}) = FunctionPolicy(f)


@POMDP_require estimate_value(estimator::SolvedRolloutEstimator, mdp::MDP, state, depth::Int) begin
    @subreq rollout(estimator, mdp, state, depth)
end

estimate_value(estimator::SolvedRolloutEstimator, mdp::MDP, state, depth::Int) = rollout(estimator, mdp, state, depth)

estimate_value(estimator::SolvedRolloutEstimator, mdp::MDP, dec_agents_sampled::Vector{SampledFlow}, state, depth::Int, t::Float64) = rollout(estimator, mdp, dec_agents_sampled::Vector{SampledFlow}, state, depth, t)

function rollout(estimator::SolvedRolloutEstimator, mdp::MDP, dec_agents_sampled::Vector{SampledFlow}, s, d::Int, t::Float64)
    sim = RolloutSimulator(estimator.rng, d)
    dec_simulate(sim, mdp, estimator.policy, dec_agents_sampled, s, t)
end

function dec_simulate(sim::RolloutSimulator, mdp::MDP, policy::Policy, dec_agents_sampled::Vector{SampledFlow}, initialstate, t::Float64) # this function only works for co-routing
    if sim.eps == nothing
        eps = 0.0
    else
        eps = sim.eps
    end
    if sim.max_steps == nothing
        max_steps = typemax(Int)
    else
        max_steps = sim.max_steps
    end

    s = initialstate

    disc = 1.0
    r_total = 0.0
    step = 1

    a = action(policy, s)

    sp, r = dec_gen(mdp, s, a, dec_agents_sampled, t, sim.rng)
    return r
end


# this rollout function is really just here in case people search for rollout
function rollout(estimator::SolvedRolloutEstimator, mdp::MDP, s, d::Int)
    sim = RolloutSimulator(estimator.rng, d)
    POMDPs.simulate(sim, mdp, estimator.policy, s)
end

@POMDP_require rollout(estimator::SolvedRolloutEstimator, mdp::MDP, s, d::Int) begin
    sim = RolloutSimulator(rng=estimator.rng, max_steps=d)
    @subreq POMDPs.simulate(sim, mdp, estimator.policy, s)
end

"""
    init_Q(initializer, mdp, s, a)
Return a value to initialize Q(s,a) to based on domain knowledge.
"""
function init_Q end
init_Q(f::Function, mdp::Union{MDP,POMDP}, s, a) = f(mdp, s, a)
init_Q(n::Number, mdp::Union{MDP,POMDP}, s, a) = convert(Float64, n)

"""
    init_N(initializer, mdp, s, a)
Return a value to initialize N(s,a) to based on domain knowledge.
"""
function init_N end
init_N(f::Function, mdp::Union{MDP,POMDP}, s, a) = f(mdp, s, a)
init_N(n::Number, mdp::Union{MDP,POMDP}, s, a) = convert(Int, n)
