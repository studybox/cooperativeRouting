using Parameters

VEHLENGTH = 7.5 # carlength + minigap
MIN_DEPART = 0.2

@with_kw mutable struct FlowDist 
    num_vehs :: Int
    flow :: Float64
    time_stamp :: Float64
    probs :: Vector{Float64}
    routes :: Vector{Vector{Int}}
    cv :: Bool
end

struct SampledFlow
    num_vehs :: Int
    flow :: Float64
    time_stamp :: Float64
    route :: Vector{Int}
    cv :: Bool
end

@with_kw mutable struct Output
    downstream_capacity :: Float64
    downstream_numvehs :: Float64
    downstream_depart :: Float64
    
    saturated :: Float64
    undersaturated :: Float64
    oversaturated :: Float64
    
    free_capacity :: Float64
    lanes :: Vector{Int}
    partial_lanes :: Int
    depart :: Float64
    queue :: Float64
    turn_rate :: Float64
end

@with_kw mutable struct MacroLink
    id :: Int
    name :: String
    travel_time :: Float64 #time cost of traveling link
    num_vehs :: Float64 #number of vehicles
    enter_fr :: Float64 #entering flow rate
    arrival_fr :: Float64 #queue arriving flow rate
    depart_fr :: Float64 #departing flow rate
    incomings_fr :: Dict{Int, Float64}#Vector{Int64}
    outgoings :: Dict{Int, Output}#Vector{Int64}
    queue_vehs :: Float64
    storage_capacity :: Float64 #capacity of the link
    speed_limit :: Float64
    num_lanes :: Int64
end

@with_kw mutable struct MacroRoute
    current_time :: Float64
    step_length :: Float64
    links::Vector{MacroLink}    
    all_flows :: Vector{SampledFlow}
    travel_times :: Dict{Int, Float64}
    cvflow_traveltime :: Float64
end

struct UrbanNetwork
    roads::Vector{MacroLink}
    l2id::Dict{String, Int}
end

mutable struct Road
    id :: Int
    name :: String
    capacity :: Dict{Int, Float64}
    connection :: Dict{Int, Vector{Int}}
    speedlimit :: Float64
    free_traveltime :: Float64
    observed_traveltime :: Float64
    observed_queue :: Float64
    observed_numvehs :: Float64
    storage_capacity :: Float64
    num_lanes :: Float64
    incomings :: Vector{Int}
    outgoings :: Vector{Int}
end
mutable struct SimpleUrbanNetwork
    roads :: Vector{Road}
    l2id :: Dict{String, Int}
    sourceTAZs :: Dict{String, Int}
    sinkTAZs :: Dict{String, Int}
    penetration :: Float64
    current_time :: Float64
end


function SimpleUrbanNetwork(net::UrbanNetwork, route_data::Dict{Int, Demand}, penetration::Float64=0.5)
    roads = Road[]
    for (idx,r) in enumerate(net.roads)
        capacity = Dict{Int, Float64}(-1=>875*r.num_lanes/60)
        connection = Dict{Int, Vector{Int}}(-1=>collect(0:r.num_lanes-1))
        incomings = Int[]
        outgoings = Int[-1]
        for (outid, outgoing) in r.outgoings
            ff = outgoing.free_capacity
            capacity[outid] = ff*60
            connection[outid] = outgoing.lanes
            push!(outgoings, outid)
        end
        for inid in keys(r.incomings_fr)
            push!(incomings, inid)
        end
        push!(roads, Road(idx, r.name, 
                       capacity, connection, r.speed_limit*60, 
                       r.travel_time/60, r.travel_time/60,
                       0.0, 0.0, r.storage_capacity, r.num_lanes, incomings, outgoings))
    end
    sourceTAZs = Dict{String, Int}()
    sinkTAZs = Dict{String, Int}()
    for id in keys(route_data)
        for r in route_data[id].routes
            sourceTAZs[r[1]] = route_data[id].from
            sinkTAZs[r[end]] = route_data[id].to
        end
    end
    SimpleUrbanNetwork(roads, net.l2id, sourceTAZs, sinkTAZs, penetration, 0.0)
end

function getTrafficinfo(ed::String)
    vehs = traci.edge.getLastStepVehicleIDs(ed)
    numveh = length(vehs)
    numqueue = 0
    for veh in vehs
        ret = traci.vehicle.getLeader(veh)
        if ret == nothing
            if traci.vehicle.getSpeed(veh) <= 5.0
                numqueue += 1
            else
                continue
            end
        else
            if ret[2] >= 2.5
                continue
            else
                numqueue += 1
            end
        end
    end
    return numveh, numqueue
end


function update!(net::SimpleUrbanNetwork, traveltime::Int64, fromedge::Int, toedge::Int, t::Float64)
    #roadind = net.l2id[fromedge]
    roadind = fromedge
    road = net.roads[roadind]
    
    num_vehs, queue = getTrafficinfo(road.name)
    road.observed_traveltime = traveltime/60
    road.observed_queue = queue
    road.observed_numvehs = num_vehs
    net.current_time = t
end




function UrbanNetwork(sumonet)
    sumoedges = sumonet.getEdges()
    id2ind = Dict{String, Int}()
    roads = MacroLink[]
    for ed in sumoedges
        lns = ed.getLanes()
        if length(lns) == 1 && !lns[1].allows("passenger")
            continue
        end
        id = ed.getID()
        id2ind[id] = length(id2ind)+1
    end
    for ed in sumoedges
        lns = ed.getLanes()
        if length(lns) == 1 && !lns[1].allows("passenger")
            continue
        end
        num_lanes = 0
        lanes = Int[]
        for ln in lns
            if ln.allows("passenger")
                num_lanes += 1
                push!(lanes, parse(Int, split(ln.getID(), "_")[2]))
            end
        end

        id = ed.getID()
        cycle = 0

        lanes2out = Dict{Int64, Vector{Int64}}()
        green2out = Dict{Int64, Int64}()
        phase2out = Dict{Int64, Vector{Int64}}()
        outgoings = Int[]
        for outed in keys(ed.getOutgoing())
            outid = outed.getID()
            outlns = outed.getLanes()
            if length(outlns) == 1 && !outlns[1].allows("passenger")
                continue
            end
            push!(outgoings, id2ind[outid])
            lanes2out[id2ind[outid]] = Int[]
            phase2out[id2ind[outid]] = Int[]
            green2out[id2ind[outid]] = 0
        end
        
        incomings = Dict{Int, Float64}()
        
        for ined in keys(ed.getIncoming())
            inid = ined.getID()
            inlns = ined.getLanes()
            if length(inlns) == 1 && !inlns[1].allows("passenger")
                continue
            end
            incomings[id2ind[inid]] = 0
        end

        if occursin("traffic", ed.getToNode().getType())
             tl = sumonet.getTLS(ed.getToNode().getID())
             #println(tl.getPrograms())
             tlg = tl.getPrograms()[collect(keys(tl.getPrograms()))[1]]
             phases = tlg.getPhases()
             cons = tl.getConnections()

             for cidx = 1:size(cons)[1]
                 fromln = cons[cidx, 1]
                 toln = cons[cidx, 2]
                 phase_ind = 1+cons[cidx,3]
                 fromedid, fromlnid = split(fromln.getID(), "_")
                 if fromedid == id && fromln.allows("passenger") && toln.allows("passenger")
                     toedid = split(toln.getID(), "_")[1]
                     #lanes2out[id2ind[toedid]] += 1
                     push!(lanes2out[id2ind[toedid]], parse(Int, fromlnid))
                     push!(phase2out[id2ind[toedid]], phase_ind)
                 end
             end
             for p in phases
                 cycle += p.duration
                 for toedind in keys(phase2out)
                     pind = phase2out[toedind]
                     #println(p.state[pind])
                     if occursin('G', p.state[pind])
                         green2out[toedind] += p.duration
                     elseif occursin('g', p.state[pind])
                         green2out[toedind] += Int(floor(p.duration/2))
                     end
                 end
             end
             
        else
            cycle = 60
            for toedind in keys(lanes2out)
                lanes2out[toedind] = lanes
                green2out[toedind] = cycle
            end
        end
        speed_limit = ed.getSpeed()

        free_flow = getFreeCapacity(speed_limit)/3600
        outgoings_output = Dict{Int, Output}()
        for oidx = 1:length(outgoings)
            outind = outgoings[oidx]
            outgoings_output[outind] = Output(0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0, 
                                              length(lanes2out[outind])*free_flow*green2out[outind]/cycle,
                                              lanes2out[outind],
                                              length(lanes2out[outind]),
                                              0.0, 0.0, 0.0)
            
        end
        edge_length = ed.getLength()
        travel_time = edge_length/speed_limit
        num_vehs = 0.0
        queue_vehs = 0.0
        demand_fr = enter_fr=arrival_fr=depart_fr = 0.0
        capacity = Int(floor(num_lanes*edge_length/VEHLENGTH))
        newlink = MacroLink(id2ind[id], id, travel_time, num_vehs,enter_fr,
                            arrival_fr, depart_fr, incomings,
                            outgoings_output,
                            queue_vehs,
                            capacity, speed_limit,
                            num_lanes)
        push!(roads, newlink)

    end

    
    return UrbanNetwork(roads, id2ind)
end
function getFreeCapacity(speed_limit::Float64)
    if speed_limit <= 5.
        return 200
    elseif speed_limit > 5. && speed_limit <= 7.
        return 412.5
    elseif speed_limit > 7. && speed_limit <= 9.
        return 600
    elseif speed_limit > 9. && speed_limit <= 11.
        return 800
    elseif speed_limit > 11. && speed_limit <= 13.
        return 800
    elseif speed_limit > 13. && speed_limit <= 16.
        return 875
    elseif speed_limit > 16. && speed_limit <= 18.
        return 875
    elseif speed_limit > 18. && speed_limit <= 22.
        return 1200
    elseif speed_limit > 22. && speed_limit <= 26.
        return 1300
    else
        return 1400
    end
end

function constructRoute(net::SimpleUrbanNetwork, rou::Vector{Int}, all_flows::Vector{SampledFlow}, step_length::Float64)
    links = MacroLink[]
    for (ind, ed) in enumerate(rou)
        road = net.roads[ed]
        # outgoing queue
        incomings = Dict{Int, Float64}(-1=>0.0)
        for in_ed in road.incomings
            if !(in_ed in rou)
                incomings[in_ed] = 0.0
            end
        end
        outgoings = Dict{Int, Output}()
        
        for out_ed in road.outgoings
            if out_ed != -1                
                caps = collect(values(net.roads[out_ed].capacity))
                min_depart = caps[1]
                for value in caps
                    if value < min_depart
                        min_depart = value
                    end
                end
                partial_lanes = 0
                if road.num_lanes == 1 || ind == length(rou)
                    partial_lanes = road.num_lanes
                else
                    target_laneids = road.connection[rou[ind+1]]
                    for lnid in road.connection[out_ed]
                        if lnid in target_laneids
                            partial_lanes += 1
                        end
                    end  
                end
                
                partial_storage = net.roads[out_ed].storage_capacity
                if out_ed == rou[ind+1] && ind+2<=length(rou)
                    partial_storage = length(net.roads[out_ed].connection[rou[ind+2]])/net.roads[out_ed].num_lanes*net.roads[out_ed].storage_capacity
                end
                outgoings[out_ed] = Output(partial_storage,#net.roads[out_ed].storage_capacity,
                                       net.roads[out_ed].observed_numvehs,
                                       min_depart,
                                       0.0, 0.0, 0.0, road.capacity[out_ed], road.connection[out_ed], partial_lanes,
                                       0.0, 0.0, 0.0)
                
            else
                if ind == length(rou)
                    outgoings[out_ed] = Output(Inf,
                                       0.0,
                                       Inf,
                                       0.0, 0.0, 0.0, road.capacity[out_ed], road.connection[out_ed], length(road.connection[out_ed]),
                                       0.0, 0.0, 0.0)
                else
                    outgoings[out_ed] = Output(Inf,
                                       0.0,
                                       Inf,
                                       0.0, 0.0, 0.0, road.capacity[out_ed], road.connection[out_ed], length(road.connection[out_ed]),
                                       0.0, 0.0, 0.0)
                end
            end
        end
        new_link = MacroLink(road.id, road.name, road.observed_traveltime, 
                             road.observed_numvehs, 0.0, 0.0, 0.0, 
                             incomings, outgoings, 
                             road.observed_queue, road.storage_capacity, road.speedlimit, 
                             road.num_lanes)
        push!(links, new_link)
    end
    initial_traveltime = Dict{Int, Float64}()
    for ind = 1:length(net.roads)
        initial_traveltime[ind] = net.roads[ind].observed_traveltime
    end
    return MacroRoute(net.current_time, step_length, links, all_flows, initial_traveltime, 0.0)
end



function updateBoundary!(mroute::MacroRoute, ind::Int, last_step::Bool, pre_step::Bool)
    link = mroute.links[ind]
    step_length2 = mroute.step_length
    if last_step
        step_length2 = 120-mroute.current_time
    end
    for ed in keys(link.incomings_fr)
        link.incomings_fr[ed] = 0.0
    end
    for ed in keys(link.outgoings)
        link.outgoings[ed].turn_rate = 0.0        
    end
    total_turn = 0.0
    for flow in mroute.all_flows
        idx = findfirst(x->x==link.id, flow.route)
        if idx != nothing
            period = flow.num_vehs/flow.flow
            arrival_time = flow.time_stamp
            for ed in flow.route[1:idx-1]
                arrival_time += mroute.travel_times[ed]
            end
            depart_time = arrival_time# + link.travel_time
            if arrival_time+period >= mroute.current_time && arrival_time <= mroute.current_time+mroute.step_length# && idx != 1
                #input entering fr
                if idx != 1
                    if flow.route[idx-1] in keys(link.incomings_fr)
                        tau = min(mroute.current_time+mroute.step_length, arrival_time+period)-max(mroute.current_time, arrival_time)
                        link.incomings_fr[flow.route[idx-1]] += flow.flow*tau/mroute.step_length
                    end
                else
                    tau = min(mroute.current_time+mroute.step_length, arrival_time+period)-max(mroute.current_time, arrival_time)
                    link.incomings_fr[-1] += flow.flow*tau/mroute.step_length
                end      
                
            end
            if arrival_time+period >= mroute.current_time && arrival_time <= mroute.current_time+step_length2 && flow.cv && !pre_step
                # total travel time record
                tau = min(mroute.current_time+step_length2, arrival_time+period)-max(mroute.current_time, arrival_time)
                mroute.cvflow_traveltime += tau*flow.flow * link.travel_time
            end
            
            if depart_time+period >= mroute.current_time && depart_time <= mroute.current_time+mroute.step_length 
                #output turn rate
                tau = min(mroute.current_time+mroute.step_length, depart_time+period)-max(mroute.current_time, depart_time)    
                if idx != length(flow.route)
                    link.outgoings[flow.route[idx+1]].turn_rate += flow.flow*tau/mroute.step_length
                    
                else
                    link.outgoings[-1].turn_rate += flow.flow*tau/mroute.step_length
                    
                end
                total_turn += flow.flow*tau/mroute.step_length
            end
        end
    end  
    
    if total_turn != 0.0
        for ed in keys(link.outgoings)
            link.outgoings[ed].turn_rate /= total_turn
            
        end
    end
    
    
end
function initializeQueue!(link::MacroLink, step_length::Float64)
    queue_pr = Dict{Int, Float64}()
    has_positive = false
    for ed in keys(link.outgoings)
        min_depart = min(link.outgoings[ed].free_capacity, max(0.0,link.outgoings[ed].downstream_capacity-link.outgoings[ed].downstream_numvehs)/step_length)
        queue_pr[ed] = link.arrival_fr*link.outgoings[ed].turn_rate - min_depart
        if queue_pr[ed] > 0.0
            has_positive = true
        end
    end
    tot = 0.0
    if has_positive
        for ed in keys(link.outgoings)
            if queue_pr[ed] <= 0.0
                queue_pr[ed] = -Inf
            end
            tot += exp(queue_pr[ed])
        end
    else
        for ed in keys(link.outgoings)
            tot += exp(queue_pr[ed])
        end
    end
    for ed in keys(link.outgoings)
        link.outgoings[ed].queue = link.queue_vehs*exp(queue_pr[ed])/tot
    end
end

function updateOutput!(output::Output, step_length::Float64, arrival_fr::Float64)
    output.saturated = output.free_capacity#*output.turn_rate
    output.undersaturated = output.queue/step_length+arrival_fr*output.turn_rate
    output.oversaturated = (output.downstream_capacity-max(0.0,(min(output.downstream_numvehs, output.downstream_capacity)-output.downstream_depart*step_length)))/step_length
    output.depart = min(output.saturated, output.undersaturated, output.oversaturated)
    if output.depart == output.undersaturated
        output.queue = 0.0
    else
        output.queue += (arrival_fr*output.turn_rate-output.depart)*step_length    
    end
end

function step!(mroute::MacroRoute, first_step::Bool, last_step::Bool, pre_step::Bool)
    for ind = 1:length(mroute.links)
        link = mroute.links[ind]            
        updateBoundary!(mroute, ind, last_step, pre_step)
        new_enter_fr = 0.0
        if ind != 1
            new_enter_fr = mroute.links[ind-1].outgoings[link.id].depart
        end
        for in_enter_fr in values(link.incomings_fr)
            new_enter_fr += in_enter_fr
        end    
            
        delay = (link.storage_capacity-link.queue_vehs)*VEHLENGTH/link.num_lanes/link.speed_limit/mroute.step_length
        tau = Int(floor(delay))
        if first_step
            link.arrival_fr = new_enter_fr
        elseif tau == 0  
            gamma = delay
            link.arrival_fr = (1-gamma)*new_enter_fr+gamma*link.enter_fr
        else
            link.arrival_fr = link.enter_fr
        end
        if first_step
            initializeQueue!(link, mroute.step_length)
        end
        link.enter_fr = new_enter_fr
        link.depart_fr = 0.0
        link.queue_vehs = 0.0
        partial_depart = 0.0
        partial_queue = 0.0
        for i in keys(link.outgoings)
            updateOutput!(link.outgoings[i], mroute.step_length, link.arrival_fr)
            link.depart_fr += link.outgoings[i].depart
            link.queue_vehs += link.outgoings[i].queue
            partial_depart += link.outgoings[i].depart*link.outgoings[i].partial_lanes/length(link.outgoings[i].lanes)
            partial_queue += link.outgoings[i].queue*link.outgoings[i].partial_lanes/length(link.outgoings[i].lanes)
        end

        link.num_vehs += (link.enter_fr - link.depart_fr)*mroute.step_length
        link.num_vehs = max(min(link.storage_capacity, link.num_vehs),0.0)
        if link.queue_vehs > link.num_vehs
            for i in keys(link.outgoings)
                link.outgoings[i].queue = link.outgoings[i].queue*link.num_vehs/link.queue_vehs
            end
            link.queue_vehs = link.num_vehs
        end
            
        if ind != 1
            mroute.links[ind-1].outgoings[link.id].downstream_numvehs = link.num_vehs
            if ind != length(mroute.links)
                mroute.links[ind-1].outgoings[link.id].downstream_depart = link.outgoings[mroute.links[ind+1].id].depart
            else
                mroute.links[ind-1].outgoings[link.id].downstream_depart = link.outgoings[-1].depart
            end
        end
            
        if link.queue_vehs == 0.0 
            link.travel_time = (link.storage_capacity-link.queue_vehs)*VEHLENGTH/link.num_lanes/link.speed_limit
                
        else
            if partial_depart <= MIN_DEPART
                link.travel_time = link.travel_time*0.8 + 0.2*((link.storage_capacity-link.queue_vehs)*VEHLENGTH/link.num_lanes/link.speed_limit + partial_queue/MIN_DEPART)
                    
            else
                link.travel_time = link.travel_time*0.8 + 0.2*((link.storage_capacity-link.queue_vehs)*VEHLENGTH/link.num_lanes/link.speed_limit + partial_queue/partial_depart)
                    
            end
        end
        mroute.travel_times[link.id] = link.travel_time    
    end
end

function simulate!(mroute::MacroRoute, initial_time::Float64)
    steps = 30 
    pre_steps = min(30, Int(floor(initial_time - mroute.current_time)/mroute.step_length))
    
    #println("pre_steps ", initial_time-mroute.current_time)
    first_link_traveltime = mroute.links[1].travel_time
    
    finish_first_link = false
    countdown = 0.0
    second_queue = mroute.links[2].queue_vehs
    for step = 1:steps+pre_steps
        if step == pre_steps+1
            first_link_traveltime = mroute.links[1].travel_time
            #countdown = 0.0
        end
        if (step > pre_steps || step==steps+pre_steps) && !finish_first_link 
            countdown += mroute.step_length
            first_link_traveltime = mroute.links[1].travel_time
            if first_link_traveltime <= countdown
                second_queue = mroute.links[2].queue_vehs
                finish_first_link = true
            end
        end
        # update total_travel_time of all connected flows
        step!(mroute, step==1, step==steps+pre_steps, step<=pre_steps)
        mroute.current_time += mroute.step_length
    end
    #println("first link tt ", first_link_traveltime)
    return mroute.cvflow_traveltime, first_link_traveltime, second_queue
end



