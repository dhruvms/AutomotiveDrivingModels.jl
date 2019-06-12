"""
	MPCDriver <: LaneFollowingDriver

This driver model picks a target state to go to based on neighbouring cars. It
then solves a simple MPC problem to get a trajectory to the target

# Fields
"""

mutable struct MPCDriver <: DriverModel{LatLonAccel}
    rec::SceneRecord
    σ::Float64
    noisy::Bool
    noise_μ::Float64
    noise_σ::Float64
    noise_θ::Float64
	lookahead::Float64

    # Outputs
    a::Float64
	δ::Float64
    v_des::Float64

    # MPC Hyperparameters
    n::Int64
	timestep::Float64
    # time::Float64
	interp::Int64

    function MPCDriver(
        timestep::Float64;
        rec::SceneRecord=SceneRecord(1, timestep),
        σ::Float64=1.0,
        noisy::Bool=false,
        noise_μ::Float64=0.0,
        noise_σ::Float64=0.2,
        noise_θ::Float64=1.0,
        num_params::Int64=6,
		lookahead::Float64=50.0,
		v_des::Float64=10.0
        )
        retval = new()

        retval.rec = rec
        retval.σ = σ
        retval.noisy = noisy
        retval.noise_μ = noise_μ
        retval.noise_σ = noise_σ
        retval.noise_θ = noise_θ
		retval.lookahead = lookahead
		retval.v_des = v_des

        retval.a = NaN
        retval.δ = NaN

        retval.n = 26
		retval.timestep = timestep
        # retval.time = 5.0
		retval.interp = 2

        retval
    end
end
get_name(::MPCDriver) = "MPCDriver"

function lane_tag_modifier(right::Bool, left::Bool, rΔ::Float64, lΔ::Float64,
							mΔ::Float64)
	if right && rΔ > mΔ
		if left
			if rΔ >= lΔ
				return -1, rΔ
			end
		end
		return -1, rΔ
	end

	if left && lΔ > mΔ
		if right
			if lΔ >= rΔ
				return 1, lΔ
			end
		end
		return 1, lΔ
	end

	return 0, mΔ
end


function observe!(driver::MPCDriver, scene::Scene, roadway::Roadway, egoid::Int)
    update!(driver.rec, scene)

    self_idx = findfirst(egoid, scene)
    ego_state = scene[self_idx].state
	ego_lane = roadway[ego_state.posF.roadind.tag]

    left_exists = convert(Float64, get(N_LANE_LEFT, driver.rec, roadway, self_idx)) > 0
    right_exists = convert(Float64, get(N_LANE_RIGHT, driver.rec, roadway, self_idx)) > 0
    fore_M = get_neighbor_fore_along_lane(scene, self_idx, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront(), max_distance_fore=driver.lookahead)
    fore_L = get_neighbor_fore_along_left_lane(scene, self_idx, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront(), max_distance_fore=driver.lookahead)
    fore_R = get_neighbor_fore_along_right_lane(scene, self_idx, roadway, VehicleTargetPointRear(), VehicleTargetPointRear(), VehicleTargetPointFront(), max_distance_fore=driver.lookahead)


	lane_choice, headway = lane_tag_modifier(right_exists, left_exists, fore_R.Δs, fore_L.Δs, fore_M.Δs)
	target_lane = roadway[LaneTag(ego_lane.tag.segment, ego_lane.tag.lane + lane_choice)]
	ego_target = Frenet(ego_state.posG, target_lane, roadway) # egostate projected onto target lane
	target_roadind = move_along(ego_target.roadind, roadway, driver.lookahead) # RoadIndex after moving along target lane
	target_pos = Frenet(target_roadind, roadway) # Frenet position on target lane after moving

	target = MPCState()
	target.x = headway
	target.y = lane_choice * target_lane.width - ego_state.posF.t
	target.θ = 0.0
	target.v = driver.v_des * (headway/driver.lookahead)
	target.β = 0.0

	self = MPCState(x=0.0, y=0.0, θ=ego_state.posF.ϕ, v=ego_state.v, β=0.0)
    params = zeros(6)
    hyperparams = [driver.n, driver.timestep, driver.interp]
    params, a1, δ1 = optimise_trajectory(target, params, hyperparams, initial=self)
	self = MPCState(x=ego_state.posG.x, y=ego_state.posG.y, θ=ego_state.posG.θ, v=ego_state.v, β=ego_state.posF.ϕ)
	last = generate_last_state!(self, params, hyperparams)

	driver.a = a1
	driver.δ = δ1
end

function set_hyperparams!(model::MPCDriver, hyperparams::Vector{Float64})
    base, ord, n, time, interp, log, knot = hyperparams
    model.base = Int64(base)
    model.ord = Int64(ord)
    model.n = Int64(n)
    model.time = time
    model.interp = Int64(interp)
    model.log = Bool(log)
    model.knot = knot
    model
end
function set_hyperparams!(model::MPCDriver,
                            base::Int64,
                            ord::Int64,
                            n::Int64,
                            time::Float64,
                            interp::Int64,
                            log::Bool,
                            knot::Float64)
    model.base = base
    model.ord = ord
    model.n = n
    model.time = time
    model.interp = interp
    model.log = log
    model.knot = knot
    model
end
function set_σ!(model::MPCDriver, σ::Float64)
    model.σ = σ
    model
end

function Base.rand(model::MPCDriver)
    if isnan(model.a) || isnan(model.δ)
        LatLonAccel(0.0, 0.0)
    else
        LatLonAccel(model.δ, model.a)
    end
end
function Distributions.pdf(model::MPCDriver, a::LatLonAccel)
    if isnan(model.a) || isnan(model.δ)
        Inf
    else
        pdf(Normal(model.a, model.σ), a.a_lon) * pdf(Normal(model.δ, model.σ), a.a_lat)
    end
end
function Distributions.logpdf(model::MPCDriver, a::LatLonAccel)
    if isnan(model.a) || isnan(model.δ)
        Inf
    else
        logpdf(Normal(model.a, model.σ), a.a_lon) * logpdf(Normal(model.δ, model.σ), a.a_lat)
    end
end
