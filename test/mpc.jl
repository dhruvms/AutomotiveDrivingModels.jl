using AutomotiveDrivingModels
using AutoViz
using Distributions
using Interact

roadway = gen_stadium_roadway(4, length=150.0, width=50.0, radius=10.0)
num_vehs = 1
timestep = 0.2

scene = Scene()
push!(scene, Vehicle(VehicleState(VecSE2(0.0,-2*DEFAULT_LANE_WIDTH,0.0),roadway,15.0), VehicleDef(),1))
push!(scene,Vehicle(VehicleState(VecSE2(15.0,-2*DEFAULT_LANE_WIDTH,0.0), roadway, 10.0), VehicleDef(), 2))
push!(scene,Vehicle(VehicleState(VecSE2(10.0,-DEFAULT_LANE_WIDTH,0.0), roadway, 15.0), VehicleDef(), 3))
push!(scene,Vehicle(VehicleState(VecSE2(10.0,-3*DEFAULT_LANE_WIDTH,0.0), roadway, 5.0), VehicleDef(), 4))
push!(scene,Vehicle(VehicleState(VecSE2(20.0,0.0,0.0), roadway, 5.0), VehicleDef(), 5))
push!(scene,Vehicle(VehicleState(VecSE2(30.0,-2*DEFAULT_LANE_WIDTH,0.0), roadway, 5.0), VehicleDef(), 6))
push!(scene,Vehicle(VehicleState(VecSE2(40.0,-DEFAULT_LANE_WIDTH,0.0), roadway, 5.0), VehicleDef(), 7))

car_colors = get_pastel_car_colors(scene)
cam = FitToContentCamera()

models = Dict{Int, DriverModel}()
models[1] = MPCDriver(timestep)
models[2] = Tim2DDriver(timestep,
        mlane = MOBIL(timestep),
    )
models[3] = LatLonSeparableDriver( # produces LatLonAccels
        ProportionalLaneTracker(), # lateral model
        IntelligentDriverModel(), # longitudinal model
)
models[4] = Tim2DDriver(timestep,
        mlane = MOBIL(timestep),
    )
models[5] = Tim2DDriver(timestep,
    mlane = MOBIL(timestep),
)
models[6] = LatLonSeparableDriver( # produces LatLonAccels
    ProportionalLaneTracker(), # lateral model
    IntelligentDriverModel(), # longitudinal model
)
models[7] = Tim2DDriver(timestep,
    mlane = MOBIL(timestep),
)

nticks = 1000
rec = SceneRecord(nticks+1, timestep)
simulate!(rec, scene, roadway, models, nticks)
render(rec[0], roadway, cam=cam, car_colors=car_colors)

@manipulate for frame_index in 1:nframes(rec)
    render(rec[frame_index-nframes(rec)],roadway,cam=cam,car_colors=car_colors)
end
