mutable struct Demand
    id:: Int
    demands :: Int
    from :: Int
    to :: Int
    probabilities :: Vector{Float64}
    routes :: Vector{Vector{String}}
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
        return 1125
    elseif speed_limit > 13. && speed_limit <= 16.
        return 1583
    elseif speed_limit > 16. && speed_limit <= 18.
        return 1100
    elseif speed_limit > 18. && speed_limit <= 22.
        return 1200
    elseif speed_limit > 22. && speed_limit <= 26.
        return 1300
    else
        return 1400
    end
end

function readRoutefile(filename::String)
    current_id::Int = 0
    current_demand_flow::Int = 0
    data = Dict{Int, Demand}()
    open(filename, "r") do io
        lines = readlines(io)
        for line in lines
            tokens = split(line, "\"")
            #println(line)
            if occursin("<flow", tokens[1])
                current_id = parse(Int, tokens[2])
                current_demand_flow = parse(Int, tokens[8])
                fromTaz = parse(Int, tokens[10])
                toTaz = parse(Int, tokens[12])
                data[current_id] = Demand(current_id,
                                           current_demand_flow,
                                           fromTaz,
                                           toTaz,
                                           Float64[],
                                           Vector{String}[])
            end
            if occursin("<route ", tokens[1])
                push!(data[current_id].routes, split(tokens[end-1]))
                push!(data[current_id].probabilities, parse(Float64, tokens[4]))
            end
        end
    end
    return data
end

function readtripfile(filename::String, data::Dict{Int, Demand}=Dict{Int, Demand}())
    current_id::Int = 0
    current_demand_flow::Int = 0
    od = Dict{Tuple{Int,Int},Int}()
    if length(data) != 0
        for id in keys(data)
            fromTAZ = data[id].from
            toTAZ = data[id].to
            od[(fromTAZ,toTAZ)] = id 
        end
    end
    open(filename, "r") do io 
        lines = readlines(io)
        for line in lines 
            tokens = split(line, "\"")
            if occursin("<vehicle", tokens[1])
                fromTAZ = parse(Int, tokens[10])
                toTAZ = parse(Int, tokens[12])
                if (fromTAZ, toTAZ) in keys(od)
                    current_id = od[(fromTAZ, toTAZ)]
                    #data[current_id].demands += 1
                else
                    current_id = length(od)+1
                    od[(fromTAZ,toTAZ)] = current_id
                    data[current_id] = Demand(current_id, 1, fromTAZ, toTAZ, Float64[], Vector{String}[])
                end
            end
            if occursin("<route ", tokens[1])
                rou = split(tokens[end-1])
                prob = parse(Float64, tokens[4])
                if rou in data[current_id].routes
                    idx = findfirst(x->x == rou, data[current_id].routes)
                    data[current_id].probabilities[idx] += prob
                else
                    push!(data[current_id].routes, rou)
                    push!(data[current_id].probabilities, prob)
                end
            end
        end
       
    end
    return data
end

function writeRoutefile(filename::String, data::Dict{Int, Demand})
    open(filename, "w") do io
        write(io, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        write(io, "<routes xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"http://sumo.dlr.de/xsd/routes_file.xsd\">\n")
        for id = 1:length(data)
            flow_info = string("    <flow id=\"", id-1, "\" begin=\"0.00\" end=\"3600.00\" number=\"",
                   data[id-1].demands, "\" fromTaz=\"", data[id-1].from,
                   "\" toTaz=\"", data[id-1].to,
                   "\" departLane=\"free\" departSpeed=\"max\">\n")

            write(io, flow_info)
            write(io, "        <routeDistribution>\n")
            for idx = 1:length(data[id-1].routes)
                route = ""
                for  r in data[id-1].routes[idx]
                    route = string(route, " ", r)
                end
                route_info = string("            <route probability=\"",
                                    data[id-1].probabilities[idx],
                                    "\" edges=\"", route, "\"/>\n")
                write(io, route_info)
            end
            write(io, "        </routeDistribution>\n")
            write(io, "    </flow>\n")
        end
        write(io, "</routes>\n")
    end
end

function writeConfigfile(filename::String, info::Dict{String, String})
    open(filename, "w") do io 
        write(io, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        write(io, "<configuration>\n")
        write(io, "    <input>\n")
        if "net-file" in keys(info)
            write(io, "        <net-file value=\"$(info["net-file"])\"/>\n")
        else
            write(io, "        <net-file value=\"u_AnnArbor1.2.net.xml\"/>\n")
        end
        if "districts" in keys(info)
            write(io, "        <additional-files value=\"$(info["districts"])\"/>\n")
        else
            write(io, "        <additional-files value=\"districts.xml\"/>\n")
        end
        if "nc-route" in keys(info)
            write(io, "        <route-files value=\"$(info["nc-route"])\"/>\n")
        else
            write(io, "        <route-files value=\"test_ncflow.rou.xml\"/>\n")
        end
        write(io, "    </input>\n")
        write(io, "    <output>\n")
        if "summary" in keys(info)
            write(io, "        <summary-output value=\"$(info["summary"])\"/>\n")
        else
            error("no summary file")
        end
        if "tripinfo" in keys(info)
            write(io, "        <tripinfo-output value=\"$(info["tripinfo"])\"/>\n")
        else
            error("no tripinfo file")
        end
        if "vehroute" in keys(info)
            write(io, "        <vehroute-output value=\"$(info["vehroute"])\"/>\n")
        else
            error("no vehroute file")
        end
        write(io, "        <vehroute-output.sorted value=\"true\"/>\n")
        write(io, "        <vehroute-output.write-unfinished value=\"true\"/>\n")
        write(io, "    </output>\n")
        write(io, "    <time>\n")
        write(io, "        <begin value=\"0\"/>\n")
        write(io, "        <step-length value=\"1\"/>\n")
        write(io, "        <lanechange.duration value=\"2.0\"/>\n")
        write(io, "    </time>\n")
        write(io, "    <processing>\n")
        if "nointernallinks" in keys(info)
            write(io, "        <no-internal-links value=\"$(info["nointernallinks"])\"/>\n")
        else
            write(io, "        <no-internal-links value=\"false\"/>\n")
        end
        if "teleport" in keys(info)
            write(io, "        <time-to-teleport value=\"$(info["teleport"])\"/>\n")
        else
            write(io, "        <time-to-teleport value=\"300\"/>\n")
        end
        write(io, "        <ignore-accidents value=\"true\"/>\n")
        write(io, "        <collision.action value=\"warn\"/>\n")
        write(io, "    </processing>\n")        
        write(io, "</configuration>")
    end
end
#=
FLOWS = Demand[]
@assert length(DEMANDS)==length(ROUTES)

for i = 1:length(DEMANDS)
    routes = Vector{String}[]
    for j = 1:5
        push!(routes, split(ROUTES[i][j], " "))
    end
    push!(FLOWS, Demand(i, DEMANDS[i], routes))
end
=#
