import Random: rand
import POMDPs: actions

struct CheckPoint
    id :: Int
    name :: Int
    congestion :: Int
    future_route :: Vector{Int}
end

struct Route
    id :: Int
    route :: Vector{Int}
end


mutable struct CoTAP <: MDP{CheckPoint, Route}
    num_vehs :: Int
    flow :: Float64
    discount :: Float64
    future_weight :: Float64
    origin :: Vector{Int}
    destination :: Vector{Int}
    ordered_states :: Vector{CheckPoint}
    ordered_actions :: Vector{Route}
    initial_states :: Vector{CheckPoint}
    checkpoint_route_dict :: Dict{Int, Vector{Route}}
    transitions:: Dict{Tuple{Int, Int, Int}, Int}
    road_infos :: SimpleUrbanNetwork
    #ncflows :: Vector{FlowDist}
    ncflows_sampled :: Vector{SampledFlow}
    current_time :: Float64
    step_length :: Float64
end

function createSAspaces(routes::Vector{Vector{Int}})
    origins = unique([routes[i][1] for i=1:length(routes)])
    destinations = unique([routes[i][end] for i=1:length(routes)])
    initial_states = CheckPoint[]
    states = CheckPoint[]
    actions = Route[]
    partial_routes = Dict{Int, Vector{Vector{Int}}}()
    # collect all the starting edge
    all_edges = Int[]
    neighbors = Dict{Int, Set{Int}}()
    for rou in routes
        for (idx, r) in enumerate(rou)
            push!(all_edges, r)
            if r in keys(neighbors)
                if idx != length(rou)
                    push!(neighbors[r], rou[idx+1])
                else
                    push!(neighbors[r], -1)
                end
               
            else
                if idx != length(rou)
                    neighbors[r] = Set{Int}(rou[idx+1])
                else
                    neighbors[r] = Set{Int}(-1)
                end
            end
        end
    end
    all_edges = unique(all_edges)
    all_checkpoints = Vector{Int}[]
    all_checkpoints_ids = Vector{Int}[]
    all_future_routes = Vector{Int}[]
    all_future_routes_ids = Int[]
    transitions = Dict{Tuple{Int, Int, Int}, Int}()
    for ed in all_edges
        for rou in routes
            if ed in rou
                ridx = findfirst(x->x==ed, rou) 
                current_rou = rou[ridx:end]
                future_rou = rou[ridx+1:end]
                if !(current_rou in all_checkpoints)
                    push!(states, CheckPoint(length(states)+1, ed, 1, future_rou))
                    push!(states, CheckPoint(length(states)+1, ed, 2, future_rou))
                    push!(states, CheckPoint(length(states)+1, ed, 3, future_rou))
                    if ed in origins
                        push!(initial_states, CheckPoint(length(states)-2, ed, 1, future_rou))
                        push!(initial_states, CheckPoint(length(states)-1, ed, 2, future_rou))
                        push!(initial_states, CheckPoint(length(states), ed, 3, future_rou))
                    end
                    push!(all_checkpoints, current_rou)
                    push!(all_checkpoints_ids, [length(states)-2, length(states)-1, length(states)])
                end
                if !(future_rou in all_future_routes)
                    push!(actions, Route(length(actions)+1, future_rou))
                    push!(all_future_routes, future_rou)
                    push!(all_future_routes_ids, length(actions))
                end
            end
        end
    end
    checkpoint_route_dict = Dict{Int, Vector{Route}}()
    for s in states 
        start_ed = s.name
        checkpoint_route_dict[s.id] = Route[]
        for (frou,frou_ind) in zip(all_future_routes, all_future_routes_ids)
            next_ed = 0
            if length(frou) == 0
                next_ed = -1 
            else
                next_ed = frou[1]
            end
            if next_ed in neighbors[start_ed]
                a = actions[frou_ind]
                push!(checkpoint_route_dict[s.id], a)
                next_route  = a.route
                if length(next_route) == 0
                    next_ind = findfirst(x->x==[s.name;s.future_route], all_checkpoints)
                    transitions[(s.id, a.id, 1)] = all_checkpoints_ids[next_ind][1]
                    transitions[(s.id, a.id, 2)] = all_checkpoints_ids[next_ind][2]
                    transitions[(s.id, a.id, 3)] = all_checkpoints_ids[next_ind][3]
                else
                    next_ind = findfirst(x->x==next_route, all_checkpoints)
                    transitions[(s.id, a.id, 1)] = all_checkpoints_ids[next_ind][1]
                    transitions[(s.id, a.id, 2)] = all_checkpoints_ids[next_ind][2]
                    transitions[(s.id, a.id, 3)] = all_checkpoints_ids[next_ind][3]
                end
            end
        end 
    end
    
    return origins, destinations, initial_states, states, actions, transitions, checkpoint_route_dict
end

function CoTAP(num_vehs::Int, flow::Float64, routes::Vector{Vector{Int}}, road_infos::SimpleUrbanNetwork, ncflows::Vector{FlowDist};
                   discount::Float64=0.8, future_weight::Float64=0.01, step_length::Float64=1.0)

        origins, 
        destinations, 
        initial_states, 
        ordered_states,
        ordered_actions, 
        transitions,
        checkpoint_route_dict = createSAspaces(routes)
        ncflows_sampled = SampledFlow[]
        for flow in ncflows
            for (prob, rou) in zip(flow.probs, flow.routes)
                push!(ncflows_sampled, SampledFlow(Int(ceil(flow.flow*prob)), flow.flow*prob, 0, rou, false))
            end
        end
        return CoTAP(num_vehs, flow, discount, future_weight, origins, destinations, ordered_states, ordered_actions, initial_states, checkpoint_route_dict, transitions, road_infos, ncflows_sampled, 0, step_length)
end

function get_initial_state(mdp::CoTAP, rou::Vector{Int}, congestion::Int)
    for s in mdp.initial_states
        s_rou = [s.name;s.future_route]
        if s_rou == rou && s.congestion == congestion
            return s
        end
    end
    error("initial route $(rou) not found")
end


discount(p::CoTAP) = p.discount
isterminal(p::CoTAP, s::CheckPoint) = s.name in p.destination

n_actions(p::CoTAP) = length(p.ordered_actions)
n_states(p::CoTAP) = length(p.ordered_states)
actions(p::CoTAP, s::CheckPoint) = p.checkpoint_route_dict[s.id]



function rand(rng::AbstractRNG, d::FlowDist)
    r = sum(d.probs)*rand(rng)
    tot = zero(eltype(d.probs))
    for (v, p) in zip(d.routes, d.probs)
        tot += p
        if r < tot
            return v
        end
    end
    error("sampling error $(d.probs) with $(d.num_vehs)")
end

function rollout(dec_agents_dists::Vector{FlowDist}, rng::AbstractRNG)
    all_flows = SampledFlow[]
    for flow_dist in dec_agents_dists
    
        if flow_dist.num_vehs != 0
            sampled_flow = SampledFlow(flow_dist.num_vehs, flow_dist.flow, flow_dist.time_stamp, rand(rng, flow_dist), flow_dist.cv)
            push!(all_flows, sampled_flow)
        end
    end
    return all_flows
end

function reward(mdp::CoTAP, s::CheckPoint, a::Route, all_flows::Vector{SampledFlow}, current_time::Float64)
    if isterminal(mdp, s)
        return 0.0
    end
    s_tt, a_tt, s_t, _ = simulate!(mdp, s, a, all_flows, current_time)
    return s_tt - a_tt# - s_t*mdp.num_vehs
end

function dec_gen(mdp::CoTAP, s::CheckPoint, a::Route, dec_agents::Vector{SampledFlow}, current_time::Float64, rng::AbstractRNG)
    if isterminal(mdp, s)
        return (s, 0.0, 0.0)
    end
    s_tt, a_tt, s_t, sp_queue = simulate!(mdp, s, a, dec_agents, current_time)
    r = s_tt - a_tt - s_t*mdp.num_vehs
    congestion = 1
    if sp_queue > 5 && sp_queue <= 10
        congestion = 2
    elseif sp_queue > 10
        congestion = 3
    end 
    sp = mdp.ordered_states[mdp.transitions[(s.id, a.id, congestion)]]
    return (sp, r, s_t)
end

function simulate!(mdp::CoTAP, s::CheckPoint, a::Route, all_flows::Vector{SampledFlow}, start_time::Float64)
    step_length = mdp.step_length
     # here we restrict name to length = 1
    s_rou = [s.name;s.future_route]
    #println(mdp.num_vehs)
    #println(mdp.flow)
    s_ego_flow = SampledFlow(mdp.num_vehs, mdp.flow, start_time, s_rou, true)
    s_all_flows = [s_ego_flow;all_flows]
    s_links = constructRoute(mdp.road_infos, s_rou, s_all_flows, step_length)
    
    s_tt, _, _ = simulate!(s_links, start_time)
    
    a_rou = [s.name;a.route]
    a_ego_flow = SampledFlow(mdp.num_vehs, mdp.flow, start_time, a_rou, true)
    a_all_flows = [a_ego_flow;all_flows]
    a_links = constructRoute(mdp.road_infos, a_rou, a_all_flows, step_length)
    
    a_tt, s_t, sp_queue = simulate!(a_links, start_time)
    #println("S_T", s_t)
    return s_tt, a_tt, s_t, sp_queue
end

