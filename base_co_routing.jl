using PyCall
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
include("macrotrafficmodel.jl")
include("./DecMCTS/decmdp.jl")
include("./DecMCTS/decmcts.jl")
include("./DecMCTS/domain_knowledge.jl")
include("utils.jl")
SUMO = string(ENV["SUMO_HOME"], "/sumo")
CONFIG = ["../SUMOSim/AnnArbor/urban/test2.sumocfg",0, 6000]
sumolib = pyimport("sumolib")
traci = pyimport("traci")
tc = pyimport("traci.constants")
#include("macrotrafficmodel.jl")


function doStep!(traffic::SimpleUrbanNetwork, all_fleets::AllFleet, rng::AbstractRNG)
    traci.simulationStep()
    subscribes = traci.vehicle.getAllSubscriptionResults()
    time = Int(traci.simulation.getCurrentTime()/1000)
    for (vehid, sub) in subscribes
        edge = traci.vehicle.getRoadID(vehid)
        route = traci.vehicle.getRoute(vehid)
        origin = route[1]
        destination = route[end]
        fromTAZ = getSourceTAZ(traffic, origin)
        toTAZ = getSinkTAZ(traffic, destination)
        veh = get_vehicle(all_fleets, vehid, fromTAZ, toTAZ)
        if edge == "" || occursin(":",edge)
            eidx = findfirst(x->x==traffic.roads[veh.edges[end]].name, route)
            #if eidx == length(route)
            #    continue
            #end
            #edge =  route[eidx+1]
            edge = route[eidx]
        end
        speed = traci.vehicle.getSpeed(vehid)
        push!(veh.steps, time)
        push!(veh.speeds, speed)
        push!(veh.edges, traffic.l2id[edge])
        traveltime = length(veh.edges)-findfirst(x->x==veh.edges[end-1], veh.edges)
        if veh.edges[end] != veh.edges[end-1] || traveltime/60 >= traffic.roads[veh.edges[end-1]].observed_traveltime
            update!(traffic, traveltime, veh.edges[end-1], veh.edges[end])
        end
        update!(all_fleets, vehid, fromTAZ, toTAZ)
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
        fromTAZ = getSourceTAZ(traffic, origin)
        toTAZ = getSinkTAZ(traffic, destination)
        currentFleet = getOpenfleet(all_fleets, fromTAZ, toTAZ, time, rng)
        add!(currentFleet, newVehicle, traffic)
    end

    fleets_to_update = get_fleets_to_update(all_fleets, time)
    for fleet in fleets_to_update
        # update channels
        fleet.last_update_step = time
        #println(keys(all_fleets.mdps))
        mdp = all_fleets.mdps[(fleet.fromTAZ, fleet.toTAZ)]
        ac = action(fleet.planner, fleet.current_state)
        if ac.route != fleet.current_action.route
            fleet.current_action = ac
            # update route
            fleet.route = [fleet.current_state.name;ac.route]
            for vehid in keys(fleet.vehicles)
                veh = fleet.vehicles[vehid]
                if veh.arrived
                    continue
                end
                #println(vehid)
                #println([traffic.roads[x].name for x in veh.edges])
                #println([traffic.roads[x].name for x in fleet.route])
                #println(traci.vehicle.getRoute(vehid))
                #println(traci.vehicle.getRoadID(vehid))

                veh_current_edge = veh.edges[end]
                start_ind = findfirst(x->x==veh_current_edge, fleet.route)
                new_route = fleet.route[start_ind:end]
                new_route_string = [traffic.roads[x].name for x in new_route]
                #println(new_route_string)
                traci.vehicle.setRoute(vehid, new_route_string)
            end
        end
        #acs = actions(mdp, fleet.current_state)
        #for ac in acs
        #    if ac.route == fleet.current_state.future_route
        #        fleet.current_action = ac
        #        break
        #    end
        #end
        #println(fleet.vehicles)
        # update planner

    end

    arrived = traci.simulation.getSubscriptionResults()[tc.VAR_ARRIVED_VEHICLES_IDS]
    for vehid in arrived
        veh = update_arrival_vehicle!(all_fleets, vehid)
        #veh = get_vehicle(all_fleets, vehid)
        if veh != nothing
            traveltime = length(veh.edges)-findfirst(x->x==veh.edges[end], veh.edges)+1
            update!(traffic, traveltime, veh.edges[end], -1)

        end
    end
    return time
end
function main(pr)
    sumoExe = SUMO
    traffic_config, ST, ET = CONFIG
    sumonet = sumolib.net.readNet("../SUMOSim/AnnArbor/urban/u_AnnArbor1.2.net.xml", withPrograms=true)
    route_data_ncflow = readRoutefile("../SUMOSim/AnnArbor/urban/test_ncflow.rou.xml");
    route_data_cvflow = readtripfile("../SUMOSim/AnnArbor/urban/test_cvflow_solution_dua.rou.xml", deepcopy(route_data_ncflow))

    traffic = SimpleUrbanNetwork(UrbanNetwork(sumonet), route_data_ncflow, pr)
    all_fleets = create_default_fleets(traffic, route_data_ncflow, route_data_cvflow) # create non CAV fleets
    traci.start([sumoExe, "-c", traffic_config])
    traci.simulation.subscribe(varIDs=(tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS))
    step = ST
    rng = MersenneTwister(1)
    df = DataFrame(Vehid=String[], Steps=Int[], EdgeIDs=String[])
    while step <= ET
        println(step)
        step = doStep!(traffic, all_fleets, rng)
    end
    for fleets in values(all_fleets.fleets)
        for fleet in fleets
            for vehid in keys(fleet.vehicles)
                veh = fleet.vehicles[vehid]
                for (t, e) in zip(veh.steps, veh.edges)
                    push!(df, [vehid, t, traffic.roads[e].name])
                end
            end
        end
    end
    traci.close()
    CSV.write("urban_dec_$(pr).csv", df)
end
