
mutable struct Vehicle
    id :: String
    steps :: Vector{Int}
    edges :: Vector{Int}
    speeds :: Vector{Float64}
    arrived :: Bool
end
mutable struct Fleet
    start_time :: Int
    fromTAZ :: Int
    toTAZ :: Int
    route :: Vector{Int}
    fleet_size :: Int
    vehicles :: Dict{String, Vehicle}
    current_state :: CheckPoint
    current_action :: Route
    last_update_step :: Int
    planner :: DecMCTSPlanner
end

mutable struct AllFleet
    fleets :: Dict{Tuple{Int, Int}, Vector{Fleet}}
    fleets_capacities :: Dict{Tuple{Int, Int}, Int}
    fleets_flows :: Dict{Tuple{Int, Int}, Float64}
    ncflows :: Dict{Tuple{Int, Int}, FlowDist}
    mdps :: Dict{Tuple{Int, Int}, CoTAP}
    channels :: Dict{Tuple{Int, Int}, Vector{Channel{FlowDist}}}
    coordination_graph :: Dict{Tuple{Int, Int, Int, Int}, Int}
end

function create_default_fleets(traffic::SimpleUrbanNetwork, nc_routes, cv_routes)
    fleets = Dict{Tuple{Int, Int}, Vector{Fleet}}()
    channels = Dict{Tuple{Int, Int}, Vector{Channel{FlowDist}}}()
    fleets_capacities = Dict{Tuple{Int, Int}, Int}()
    fleets_flows = Dict{Tuple{Int, Int}, Float64}()
    ncflows = Dict{Tuple{Int, Int}, FlowDist}()
    mdps = Dict{Tuple{Int, Int}, CoTAP}()
    all_edges = Dict{Tuple{Int, Int}, Vector{Int}}()
    co_graph = Dict{Tuple{Int,Int,Int,Int},Int}()
    vec_ncflows = FlowDist[]
    for id in keys(nc_routes)
        fromTAZ = nc_routes[id].from
        toTAZ = nc_routes[id].to
        demands = nc_routes[id].demands
        ncprobs = Float64[]
        ncroutes = Vector{Int}[]

        for (p,r) in zip(nc_routes[id].probabilities, nc_routes[id].routes)
            push!(ncprobs, p)
            push!(ncroutes, [traffic.l2id[x] for x in r])

        end
        ncprobs /= sum(ncprobs)
        num_vehs = Int(ceil(demands*(1-traffic.penetration)))
        flow_rt = num_vehs/60
        ncflows[(fromTAZ, toTAZ)] = FlowDist(num_vehs, flow_rt, 0, ncprobs, ncroutes, false)
        push!(vec_ncflows, FlowDist(num_vehs, flow_rt, 0, ncprobs, ncroutes, false))
    end

    for id in keys(cv_routes)
        fromTAZ = cv_routes[id].from
        toTAZ = cv_routes[id].to
        demands = nc_routes[id].demands
        fleets[(fromTAZ, toTAZ)] = Fleet[]
        channels[(fromTAZ, toTAZ)] = Channel{FlowDist}[]
        fleets_capacities[(fromTAZ,toTAZ)] = max(5, Int(ceil(demands*traffic.penetration/12)))
        fleets_flows[(fromTAZ,toTAZ)] = demands*traffic.penetration/60
        
        cvprobs = Float64[]
        cvroutes = Vector{Int}[]
        edges = []
        for (p, r) in zip(cv_routes[id].probabilities, cv_routes[id].routes)
            push!(cvprobs, p)
            push!(cvroutes, [traffic.l2id[x] for x in r])
            append!(edges, [traffic.l2id[x] for x in r])
        end
        #println(routes[id])
        cvprobs /= sum(cvprobs)
        mdps[(fromTAZ,toTAZ)] = CoTAP(fleets_capacities[(fromTAZ,toTAZ)], fleets_flows[(fromTAZ,toTAZ)],
                                       cvroutes, traffic, vec_ncflows)
        edges = unique(edges)
        all_edges[(fromTAZ,toTAZ)] = edges
    end
    for pair_i in keys(all_edges)
        for pair_j in keys(all_edges)
            if pair_i != pair_j
                if length(intersect(all_edges[pair_i], all_edges[pair_j])) > 0
                    co_graph[(pair_i[1],pair_i[2],pair_j[1],pair_j[2])] = 1
                else
                    co_graph[(pair_i[1],pair_i[2],pair_j[1],pair_j[2])] = 0
                end
            end
        end
    end
    return AllFleet(fleets, fleets_capacities, fleets_flows, ncflows, mdps, channels, co_graph)
end

function getSourceTAZ(traffic::SimpleUrbanNetwork, o::String)
    return traffic.sourceTAZs[o]
end
function getSinkTAZ(traffic::SimpleUrbanNetwork, d::String)
    return traffic.sinkTAZs[d]
end
function getOpenfleet(all_fleets::AllFleet, oTAZ::Int, dTAZ::Int, time::Int, rng::AbstractRNG)
    fleets = all_fleets.fleets[(oTAZ, dTAZ)]
    ncflow = all_fleets.ncflows[(oTAZ, dTAZ)]
    channels = all_fleets.channels[(oTAZ, dTAZ)]
    if length(fleets) >=1
        if all_fleets.fleets_capacities[(oTAZ,dTAZ)] > fleets[end].fleet_size
            return fleets[end]
        end
    end
    mdp = all_fleets.mdps[(oTAZ, dTAZ)]
    rou = rand(rng, ncflow)
    s_queue = mdp.road_infos.roads[rou[1]].observed_queue
    congestion = 1
    if s_queue > 5 && s_queue <= 10
        congestion = 2
    elseif s_queue > 10
        congestion = 3
    end 
    current_state = get_initial_state(mdp, rou, congestion)
    solver = DecMCTSSolver(n_iterations=10, n_period=10, depth=10, exploration_constant=5.0, reuse_tree=true)
    acs = actions(mdp, current_state)
    new_routes = Vector{Int}[]
    for ac in acs
        push!(new_routes, [current_state.name;ac.route])
    end
    #println(new_routes)
    new_probs = ones(Float64, length(new_routes))/length(new_routes)
    default_flowdist = FlowDist(all_fleets.fleets_capacities[(oTAZ,dTAZ)], all_fleets.fleets_flows[(oTAZ,dTAZ)], time/60, new_probs, new_routes, true)
    newchannel = Channel{FlowDist}(1)
    put!(newchannel, default_flowdist)
    push!(channels, newchannel)
    channel_id = (oTAZ,dTAZ, length(channels))
    joint_ids = Tuple{Int,Int,Int}[]
    for (fromTAZ,toTAZ) in keys(all_fleets.fleets)
        if fromTAZ == oTAZ  && toTAZ == dTAZ
            continue
        end
        if all_fleets.coordination_graph[(oTAZ,dTAZ,fromTAZ,toTAZ)] == 1
            dec_fleets = all_fleets.fleets[(fromTAZ,toTAZ)]
            for (dec_ind, dec_fleet) in enumerate(dec_fleets)
                if dec_fleet.start_time - time <= 5*60
                    push!(joint_ids, (fromTAZ, toTAZ, dec_ind))
                    push!(dec_fleet.planner.joint_ids, (oTAZ,dTAZ,length(fleets)+1))
                end
            end
        end
    end
    planner = DecMCTSPlanner(solver, mdp, all_fleets.channels, channel_id, joint_ids)
    newfleet = Fleet(time, oTAZ, dTAZ,
                     rou, 0,
                     Dict{String, Vehicle}(),
                     current_state, Route(-1, Int[]), 0, planner)
    push!(fleets, newfleet)

    return newfleet
end
function add!(fleet::Fleet, veh::Vehicle, traffic::SimpleUrbanNetwork)
    fleet.vehicles[veh.id] = veh
    fleet.fleet_size += 1
    veh_current_edge = veh.edges[end]
    #println(traffic.roads[veh_current_edge])
    #println([traffic.roads[x].name for x in fleet.route])
    start_ind = findfirst(x->x==veh_current_edge, fleet.route)
    new_route = fleet.route[start_ind:end]
    new_route_string = [traffic.roads[x].name for x in new_route]
    #println(new_route_string)
    traci.vehicle.setRoute(veh.id, new_route_string)
end


function update!(all_fleets::AllFleet, vehid::String, oTAZ::Int, dTAZ::Int, t::Float64)
    fleets = all_fleets.fleets[(oTAZ, dTAZ)]
    channels = all_fleets.channels[(oTAZ, dTAZ)]
    mdp = all_fleets.mdps[(oTAZ, dTAZ)]
    #mdp.current_time = t
    for (fleet, channel) in zip(fleets, channels)
        if vehid in keys(fleet.vehicles)
            current_flowdist = fetch(channel)
            current_edge = fleet.vehicles[vehid].edges[end]
            #sumo_current_edge = traci.vehicle.getRoadID(vehid)
            #if occursin(":", sumo_current_edge)
            #    println("Here")
            #    current_edge = fleet.route[findfirst(x->x==current_edge, fleet.route)+1]
            #end
            #println(vehid)
            #println(all_fleets.fleets_capacities[(oTAZ, dTAZ)])
            #println(current_flowdist.routes)
            #println(current_flowdist.flow)
            if current_edge in fleet.current_action.route
                queue = mdp.road_infos.roads[current_edge].observed_queue
                congestion = 1
                if queue > 5 && queue <= 10
                    congestion = 2
                elseif queue > 10
                    congestion = 3
                end 
                #tempo_state = fleet.current_state
                #tempo_action = fleet.current_action
                fleet.current_state = mdp.ordered_states[mdp.transitions[(fleet.current_state.id, fleet.current_action.id, congestion)]]
                if length(fleet.current_state.future_route) == 0
                    fleet.current_action = Route(0, Int[])
                else
                    fleet.current_action = Route(-1, Int[])
                end
                acs = actions(mdp, fleet.current_state)
                new_routes = Vector{Int}[]
                for ac in acs
                    push!(new_routes, [fleet.current_state.name;ac.route])
                end
                #update flow dist
                new_probs = ones(Float64, length(new_routes))/length(new_routes)
                #new_routes = Vector{Int}[]
                #new_probs = Float64[]
                #for (prob, r) in zip(current_flowdist.probs, current_flowdist.routes)
                #    if current_edge in r
                #        idx = findfirst(x->x==current_edge, r)
                #        push!(new_routes, r[idx:end])
                #        push!(new_probs, prob)
                #    end
                #end
                #new_probs = new_probs/sum(new_probs)
                #if length(new_probs)  == 0
                #    println("CE ", current_edge)
                #    println("flowdist ", current_flowdist)
                #    println("flow_routes ", current_flowdist.routes)
                #    println("action ", tempo_action.route)
                #    println("state ", tempo_state)
                #    println("sp ", fleet.current_state)
                #    println("fleet route ", fleet.route)
                #end
                empty!(current_flowdist.routes)
                empty!(current_flowdist.probs)
                for (prob, r) in zip(new_probs, new_routes)
                    push!(current_flowdist.probs, prob)
                    push!(current_flowdist.routes, r)
                end
                
                current_flowdist.time_stamp = t
                take!(channel)
                put!(channel, current_flowdist)
            #elseif current_edge != current_flowdist.routes[1][1] && current_edge in current_flowdist.routes[1]
                #println("FLOWDIST")
                #println(vehid)
                #println(current_edge)
                #println(fleet.vehicles[vehid].edges)
                #println(fleet.current_state)
                #println(fleet.current_action)
                #println(fleet.route)
                #println(traci.vehicle.getRoadID(vehid))
                #println(current_flowdist.routes)
            #    for (ridx, r) in enumerate(current_flowdist.routes)
            #        idx = findfirst(x->x==current_edge, r)
            #        current_flowdist.routes[ridx] = r[idx:end]
            #    end
            #    current_flowdist.time_stamp = t
            #    take!(channel)
            #    put!(channel, current_flowdist)
            end
            break
        end
    end
end

function get_vehicle(all_fleets::AllFleet, vehid::String, oTAZ::Int, dTAZ::Int)
    fleets = all_fleets.fleets[(oTAZ, dTAZ)]
    for fleet in fleets
        if vehid in keys(fleet.vehicles)
            return fleet.vehicles[vehid]
        end
    end
    error("vehicle $(vehid) not found")
end

function get_vehicle(all_fleets::AllFleet, vehid::String)
    for fleets in values(all_fleets.fleets)
        for fleet in fleets
            if vehid in keys(fleet.vehicles)
                return fleet.vehicles[vehid]
            end
        end
    end
    return nothing
end

function update_arrival_vehicle!(all_fleets::AllFleet, vehid::String)
    for (oTAZ, dTAZ) in keys(all_fleets.fleets)
        fleets = all_fleets.fleets[(oTAZ, dTAZ)]
        channels = all_fleets.channels[(oTAZ, dTAZ)]
        mdp = all_fleets.mdps[(oTAZ, dTAZ)]
        for (fleet, channel) in zip(fleets, channels)
            if vehid in keys(fleet.vehicles)
                fleet.vehicles[vehid].arrived = true
                current_flowdist = fetch(channel)
                if current_flowdist.num_vehs <= 1
                    # remove channel
                    new_flowdist = FlowDist(0, current_flowdist.flow, current_flowdist.time_stamp, Float64[], Vector{Int}[], true)
                    take!(channel)
                    put!(channel, new_flowdist)
                else
                    # reduce channel
                    latest_step = 1
                    for dec_vehid in keys(fleet.vehicles)
                        if !fleet.vehicles[dec_vehid].arrived
                            this_step = findfirst(x->x==fleet.vehicles[dec_vehid].edges[end], fleet.route)
                            #println(this_step)
                            if this_step > latest_step
                                latest_step = this_step
                            end
                        end
                    end
                    new_flowdist = FlowDist(current_flowdist.num_vehs-1, current_flowdist.flow, 
                                            current_flowdist.time_stamp, [1.0], [fleet.route[latest_step:end]], true)
                    take!(channel)
                    put!(channel, new_flowdist)
                end
                return fleet.vehicles[vehid]
            end
        end
    end
    return nothing
end

function get_fleets_to_update(all_fleets::AllFleet, step::Int)
    fleets_to_update = Fleet[]
    for fleets in values(all_fleets.fleets)
        for fleet in fleets
            if step - fleet.last_update_step == 5 || fleet.current_action.id==-1#update every 5 second
                if fleet.current_action.id != 0
                    stop = 0
                    for vehid in keys(fleet.vehicles)
                        cur_edge = traci.vehicle.getRoadID(vehid)
                        if cur_edge == ""
                            stop = 1
                            break
                        end
                    end
                    if stop == 0
                        push!(fleets_to_update, fleet)
                    end
                end
            end
        end
    end
    return fleets_to_update
end
#=
function get_channels(all_fleets::AllFleet, fleet::Fleet)
    fromTAZ = fleet.fromTAZ
    toTAZ = fleet.toTAZ
    get_neighbors(all_fleets, fromTAZ, toTAZ)
end
=#
