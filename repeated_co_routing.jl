using PyCall
using JuMP, Ipopt
using DataFrames
using DataStructures
using CSV
using POMDPs
using POMDPModelTools
using POMDPPolicies
using POMDPSimulators
using Random
using CPUTime
using Printf
include("default_routes.jl")
include("traffic.jl")
include("./DecMCTS/decmdp.jl")
include("./DecMCTS/decmcts.jl")
include("./DecMCTS/domain_knowledge.jl")
include("utils.jl")
SUMO = string(ENV["SUMO_HOME"], "/sumo")
#CONFIG = ["../SUMOSim/AnnArbor/urban/test2.sumocfg",0, 6000]
sumolib = pyimport("sumolib")
traci = pyimport("traci")
tc = pyimport("traci.constants")
#include("macrotrafficmodel.jl")
function rand(rng::AbstractRNG, d::Demand)
    r = sum(d.probabilities)*rand(rng)
    tot = zero(eltype(d.probabilities))
    for (v, p) in zip(d.routes, d.probabilities)
        tot += p
        if r < tot
            return v
        end
    end
    error("sampling error $(d.probs)")
end

function doStep!(traffic::SimpleUrbanNetwork, cv_routes, vehicles::Dict{String, Tuple{Vehicle,Int}}, rng::AbstractRNG)
    traci.simulationStep()
    subscribes = traci.vehicle.getAllSubscriptionResults()
    time = Int(traci.simulation.getCurrentTime()/1000)
    for (vehid, sub) in subscribes
        edge = traci.vehicle.getRoadID(vehid)
        route = traci.vehicle.getRoute(vehid)
        veh = vehicles[vehid][1]
        
        if edge == "" || occursin(":",edge)
            eidx = findfirst(x->x==traffic.roads[veh.edges[end]].name, route)
            edge = route[eidx]
        end
        speed = traci.vehicle.getSpeed(vehid)
        push!(veh.steps, time)
        push!(veh.speeds, speed)
        push!(veh.edges, traffic.l2id[edge])
        traveltime = length(veh.edges)-findfirst(x->x==veh.edges[end-1], veh.edges)
        if veh.edges[end] != veh.edges[end-1] || traveltime/60 >= traffic.roads[veh.edges[end-1]].observed_traveltime
            update!(traffic, traveltime, veh.edges[end-1], veh.edges[end], time/60)
        end
        # rerouting
        edge = traci.vehicle.getRoadID(vehid)
        id = parse(Int, split(vehid, ".")[1])
        last_update_step = vehicles[vehid][2]
        if time - last_update_step == 5 && !(edge == "" || occursin(":",edge))
            best_rou = nothing
            best_traveltime = Inf
            for rou in cv_routes[id].routes
                if edge in rou
                    first_ind = findfirst(x->x==edge, rou)
                    current_travel_time = 0
                    for ed in rou[first_ind:end]
                        current_travel_time += traffic.roads[traffic.l2id[ed]].observed_traveltime
                    end
                    if current_travel_time < best_traveltime
                        best_rou = rou[first_ind:end]
                    end
                end
            end
            
            try
                traci.vehicle.setRoute(vehid, best_rou)
                vehicles[vehid] = (veh,time)
            catch
                println("fail to set route $(best_rou) for veh $(vehid)")
            end
        end
    end
    departed = traci.simulation.getSubscriptionResults()[tc.VAR_DEPARTED_VEHICLES_IDS]
    for vehid in departed
        if rand(rng) >= traffic.penetration
            continue
        end
        traci.vehicle.subscribe(vehid)
        route = traci.vehicle.getRoute(vehid)
        speed = traci.vehicle.getSpeed(vehid)
        origin = route[1]
        destination = route[end]
        newVehicle = Vehicle(vehid, [time], [traffic.l2id[origin]], [speed], false)
        vehicles[vehid] = (newVehicle, time)
        id = parse(Int, split(vehid, ".")[1])
        best_rou = nothing
        best_traveltime = Inf
        for rou in cv_routes[id].routes
            current_travel_time = 0
            for ed in rou
                current_travel_time += traffic.roads[traffic.l2id[ed]].observed_traveltime
            end
            if current_travel_time < best_traveltime
                best_rou = rou
            end
        end
        traci.vehicle.setRoute(vehid, best_rou)
        #
    end

    arrived = traci.simulation.getSubscriptionResults()[tc.VAR_ARRIVED_VEHICLES_IDS]
    for vehid in arrived
        if vehid in keys(vehicles)
            veh = vehicles[vehid][1]
            veh.arrived = true
            traveltime = length(veh.edges)-findfirst(x->x==veh.edges[end], veh.edges)+1
            update!(traffic, traveltime, veh.edges[end], -1, time/60)
        end
    end
    return time
end

function main(pr)
    sumoExe = SUMO
    traffic_config = "../SUMOSim/AnnArbor/urban/test_repeated_$(pr).sumocfg"
    config_info = Dict("summary"=>"summary_repeated$(pr).xml",
                       "tripinfo"=>"tripinfo_repeated$(pr).xml",
                       "vehroute"=>"routeinfo_repeated$(pr).xml")
    writeConfigfile(traffic_config, config_info)
    ST = 0
    ET = 12000
    sumonet = sumolib.net.readNet("../SUMOSim/AnnArbor/urban/u_AnnArbor1.2.net.xml", withPrograms=true)
    route_data_ncflow = readRoutefile("../SUMOSim/AnnArbor/urban/test_ncflow.rou.xml");
    route_data_cvflow = readtripfile("../SUMOSim/AnnArbor/urban/test_cvflow_solution_dua.rou.xml", deepcopy(route_data_ncflow))
    traffic = SimpleUrbanNetwork(UrbanNetwork(sumonet), route_data_ncflow, pr)

    traci.start([sumoExe, "-c", traffic_config])
    traci.simulation.subscribe(varIDs=(tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS))
    step = ST
    rng = MersenneTwister(1)
    df = DataFrame(Vehid=String[], Steps=Int[], EdgeIDs=String[])
    vehicles = Dict{String, Tuple{Vehicle, Int}}()
    while step <= ET
        step = doStep!(traffic, route_data_cvflow, vehicles, rng)
    end
    for vehid in keys(vehicles)
        veh = vehicles[vehid][1]
        for (t, e) in zip(veh.steps, veh.edges)
            push!(df, [vehid, t, traffic.roads[e].name])
        end
    end
    traci.close()
    CSV.write("urban_repeated_$(pr).csv", df)
end
