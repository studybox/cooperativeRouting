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
#include("macrotrafficmodel.jl")
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

function doStep!(traffic::SimpleUrbanNetwork, cv_routes, vehicles::Dict{String, Vehicle}, rng::AbstractRNG)
    traci.simulationStep()
    subscribes = traci.vehicle.getAllSubscriptionResults()
    time = Int(traci.simulation.getCurrentTime()/1000)
    for (vehid, sub) in subscribes
        edge = traci.vehicle.getRoadID(vehid)
        route = traci.vehicle.getRoute(vehid)
        veh = vehicles[vehid]
        if edge == "" || occursin(":",edge)
            eidx = findfirst(x->x==traffic.roads[veh.edges[end]].name, route)
            edge = route[eidx]
        end
        speed = traci.vehicle.getSpeed(vehid)
        push!(veh.steps, time)
        push!(veh.speeds, speed)
        push!(veh.edges, traffic.l2id[edge])
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
        vehicles[vehid] = newVehicle
        id = parse(Int, split(vehid, ".")[1])
        rou = rand(rng, cv_routes[id])
        traci.vehicle.setRoute(vehid, rou)
        #
    end

    #arrived = traci.simulation.getSubscriptionResults()[tc.VAR_ARRIVED_VEHICLES_IDS]

    return time
end

function main(pr)
    sumoExe = SUMO
    traffic_config = "../SUMOSim/AnnArbor/urban/test_3static_$(pr).sumocfg"
    config_info = Dict("summary"=>"summary_3static$(pr).xml",
                       "tripinfo"=>"tripinfo_3static$(pr).xml",
                       "vehroute"=>"routeinfo_3static$(pr).xml",
                       "nointernallinks"=>"true")
    writeConfigfile(traffic_config, config_info)
    ST = 0
    ET = 12000
    sumonet = sumolib.net.readNet("../SUMOSim/AnnArbor/urban/u_AnnArbor1.2.net.xml", withPrograms=true)
    route_data_ncflow = readRoutefile("../SUMOSim/AnnArbor/urban/test_ncflow.rou.xml");
    route_data_cvflow = readtripfile("../SUMOSim/AnnArbor/urban/test_cvflow_solution_dua.rou.xml", deepcopy(route_data_ncflow))
    #route_data_cvflow = deepcopy(route_data_ncflow)
    traffic = SimpleUrbanNetwork(UrbanNetwork(sumonet), route_data_ncflow, pr)
    ### solve the static system optimal problem
    roadids = Dict{String, Int}()
    num_ODs = length(route_data_cvflow)
    num_routes = 0
    for id in keys(route_data_cvflow)
        for r in route_data_cvflow[id].routes
            num_routes += 1
            for ed in r
                if ed in keys(roadids)
                    continue
                else
                    roadids[ed] = length(roadids)+1
                end
            end
        end
    end
    num_roads = length(roadids)
    capacities = zeros(num_roads)
    default_traveltimes = zeros(num_roads)
    for id in keys(roadids)
        ed = sumonet.getEdge(id)
        lns = ed.getLanes()
        num_lanes = 0
        for ln in lns
            if ln.allows("passenger")
                num_lanes += 1
            end
        end
        tt = ed.getLength()/ed.getSpeed()
        capacities[roadids[id]] = 800*num_lanes
        default_traveltimes[roadids[id]] = tt/60
    end
    Ap = zeros(num_ODs, num_routes)
    Bp = ones(num_ODs)
    Ad = zeros(num_roads, num_routes)
    Bd = zeros(num_roads)
    rind = 1
    for id = 1:length(route_data_cvflow)
        for r in route_data_cvflow[id-1].routes
            Ap[id, rind] = 1
            for ed in r
                Ad[roadids[ed], rind] = route_data_ncflow[id-1].demands*pr
            end
            rind += 1
        end
        for (prob,r) in zip(route_data_ncflow[id-1].probabilities, route_data_ncflow[id-1].routes)
            for ed in r
                if ed in keys(roadids)
                    Bd[roadids[ed]] += route_data_ncflow[id-1].demands*(1-pr)*prob/sum(route_data_ncflow[id-1].probabilities)
                end
            end
        end
    end
    model = Model(with_optimizer(Ipopt.Optimizer))
    @variable(model, 0<=x[1:num_routes]<=1)
    @constraint(model, con, Ap * x .== Bp)
    #f(x...) = sum(sum(Ad[i,j]*x[j] for j in 1:num_routes)*((0.15*((sum(Ad[i,j]*x[j] for j in 1:num_routes)+Bd[i])/capacities[i])^4+1)*default_traveltimes[i]) for i in 1:num_roads)
    f(x...) = sum(sum(Ad[i,j]*x[j] for j in 1:num_routes)*((3.4*((sum(Ad[i,j]*x[j] for j in 1:num_routes)+Bd[i])/capacities[i])+1)*default_traveltimes[i]) for i in 1:num_roads)
    register(model, :f, num_routes, f; autodiff = true)
    #@NLobjective(model, Min, f(x...))
    @objective(model, Min, f(x...))
    optimize!(model)
    println(termination_status(model))
    x_solve = JuMP.value.(x)
    rind = 1
    for id = 1:length(route_data_cvflow)
        for idx = 1:length(route_data_cvflow[id-1].routes)
            route_data_cvflow[id-1].probabilities[idx] = x_solve[rind]
            rind += 1
        end
    end
    writeRoutefile("../SUMOSim/AnnArbor/urban/test_3static_solution_$(pr).rou.xml", route_data_cvflow)
    ###
    traci.start([sumoExe, "-c", traffic_config])
    traci.simulation.subscribe(varIDs=(tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS))
    step = ST
    rng = MersenneTwister(2)
    df = DataFrame(Vehid=String[], Steps=Int[], EdgeIDs=String[])
    vehicles = Dict{String, Vehicle}()
    while step <= ET
        step = doStep!(traffic, route_data_cvflow, vehicles, rng)
    end
    for vehid in keys(vehicles)
        veh = vehicles[vehid]
        for (t, e) in zip(veh.steps, veh.edges)
            push!(df, [vehid, t, traffic.roads[e].name])
        end
    end
    traci.close()
    CSV.write("urban_3static_$(pr).csv", df)
end
