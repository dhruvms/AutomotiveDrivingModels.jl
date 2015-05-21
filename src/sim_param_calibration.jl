export 
        ParamsHistobin,

        calc_trace_deviation,
        calc_traceset_deviations,
        calc_histobin,
        KL_divergence_categorical,
        KL_divergence_dirichlet,
        coordinate_descent_histobin_fit!

type ParamsHistobin
    discx :: LinearDiscretizer
    discy :: LinearDiscretizer
end

function calc_trace_deviation(simlog::Matrix{Float64}, history::Int)
    
    initial_speed = simlog[history,LOG_COL_V]

    Δx = simlog[end,LOG_COL_X] - simlog[history,LOG_COL_X]
    Δy = simlog[end,LOG_COL_Y] - simlog[history,LOG_COL_Y]
    Δt = Δx / initial_speed
    
    (Δt, Δy)
end
function calc_trace_deviation(trace::VehicleTrace)

    calc_trace_deviation(trace.log, trace.history)
end

function calc_traceset_deviations(tracesets::Vector{Vector{VehicleTrace}})

    m = length(tracesets)
    Δt_arr = Array(Float64, m) # x-deviation divided by initial speed
    Δy_arr = Array(Float64, m)

    for (i,traceset) in enumerate(tracesets)

        egotrace = traceset[1]
        Δt, Δy = calc_trace_deviation(egotrace)

        Δt_arr[i] = Δt
        Δy_arr[i] = Δy
    end

    (Δt_arr, Δy_arr)
end
function calc_traceset_deviations(simlogs::Vector{Matrix{Float64}}, history::Int)

    m = length(simlogs)
    Δt_arr = Array(Float64, m) # x-deviation divided by initial speed
    Δy_arr = Array(Float64, m)

    for (i,simlog) in enumerate(simlogs)
        
        Δt, Δy = calc_trace_deviation(simlog, history)

        Δt_arr[i] = Δt
        Δy_arr[i] = Δy
    end

    (Δt_arr, Δy_arr)
end

function calc_histobin(Δt_arr::Vector{Float64}, Δy_arr::Vector{Float64}, histobin_params::ParamsHistobin)

    n_bins_x = nlabels(histobin_params.discx)
    n_bins_y = nlabels(histobin_params.discy)

    histobin = zeros(Float64, n_bins_x, n_bins_y)

    for (Δt, Δy) in zip(Δt_arr, Δy_arr)

        bin_x = encode(histobin_params.discx, Δt)
        bin_y = encode(histobin_params.discy, Δy)

        histobin[bin_x, bin_y] += 1.0
    end

    histobin
end
function calc_histobin(tracesets::Vector{Vector{VehicleTrace}}, histobin_params::ParamsHistobin)

    n_bins_x = nlabels(histobin_params.discx)
    n_bins_y = nlabels(histobin_params.discy)

    histobin = zeros(Float64, n_bins_x, n_bins_y)

    for (i,traceset) in enumerate(tracesets)

        egotrace = traceset[1]
        Δt, Δy = calc_trace_deviation(egotrace)

        bin_x = encode(histobin_params.discx, Δt)
        bin_y = encode(histobin_params.discy, Δy)

        histobin[bin_x, bin_y] += 1.0
    end

    histobin
end
function calc_histobin(simlogs::Vector{Matrix{Float64}}, histobin_params::ParamsHistobin, history::Int)

    n_bins_x = nlabels(histobin_params.discx)
    n_bins_y = nlabels(histobin_params.discy)

    histobin = zeros(Float64, n_bins_x, n_bins_y)

    for (i,simlog) in enumerate(simlogs)

        Δt, Δy = calc_trace_deviation(simlog, history)

        bin_x = encode(histobin_params.discx, Δt)
        bin_y = encode(histobin_params.discy, Δy)

        histobin[bin_x, bin_y] += 1.0
    end

    histobin
end

function KL_divergence_categorical(histobinA::Matrix{Float64}, histobinB::Matrix{Float64})
    
    Ap = histobinA ./ sum(histobinA)
    Bp = histobinB ./ sum(histobinB)

    KL_divergence = 0.0
    for i = 1 : length(Ap)
        KL_divergence += Ap[i]*log(Ap[i]/Bp[i])
    end
    KL_divergence::Float64
end
function KL_divergence_dirichlet(histobinA::Matrix{Float64}, histobinB::Matrix{Float64})
    α0 = sum(histobinA)
    KL_divergence = 0.0
    KL_divergence += lgamma(α0)
    KL_divergence -= lgamma(sum(histobinB))
    KL_divergence -= sum([lgamma(a) for a in histobinA])
    KL_divergence += sum([lgamma(b) for b in histobinB])
    for i = 1 : length(histobinA)
        KL_divergence += (histobinA[i] - histobinB[i])*(digamma(histobinA[i]) - digamma(α0))
    end
    KL_divergence::Float64
end

function coordinate_descent_histobin_fit!{B<:AbstractVehicleBehavior}(
    simlogs         :: Vector{Matrix{Float64}},
    behaviors       :: Vector{B},
    road            :: StraightRoadway,
    history         :: Int,
    target_histobin :: Matrix{Float64},
    histobin_params :: ParamsHistobin,
    simparams       :: SimParams,    
    KLdiv_method    :: Symbol; # ∈ :Dirichlet, :Categorical
    verbosity       :: Int = 0
    )

    if KLdiv_method == :Dirichlet
        KL_div_function = KL_divergence_dirichlet
    elseif KLdiv_method == :Categorical
        KL_div_function = KL_divergence_categorical
    else
        error("unknown KL divergence method $KLdiv_method")
    end

    if verbosity > 0
        println("Coordinte descent $KLdiv_method")
        tic()
    end

    target_histobin_with_prior = target_histobin .+ 1.0

    param_options = (
        [SAMPLE_UNIFORM, SAMPLE_BIN_CENTER, SAMPLE_UNIFORM_ZERO_BIN],
        [(:none, 1), (:SMA, 2), (:SMA, 3), (:SMA, 4), (:SMA, 5),
                     (:WMA, 2), (:WMA, 3), (:WMA, 4), (:WMA, 5)],
        [SAMPLE_UNIFORM, SAMPLE_BIN_CENTER, SAMPLE_UNIFORM_ZERO_BIN],
        [(:none, 1), (:SMA, 2), (:SMA, 3), (:SMA, 4), (:SMA, 5),
                     (:WMA, 2), (:WMA, 3), (:WMA, 4), (:WMA, 5)]
    )

    n_params = length(param_options)
    params_tried = Set{Vector{Int}}() # paraminds
    paraminds = ones(Int, n_params)
    egobehavior = behaviors[1]

    converged = false
    best_KLdiv = Inf
    iter = 0
    while !converged
        iter += 1
        converged = true
        if verbosity > 0
            println("iteration ", iter)
            toc()
            tic()
        end
        
        for coordinate = 1 : n_params
            if verbosity > 1
                println("\tcoordinate ", coordinate)
            end

            for ip in 1 : length(param_options[coordinate])

                newparams = copy(paraminds)
                newparams[coordinate] = ip

                if !in(newparams, params_tried)
                    push!(params_tried, newparams)
                    
                    egobehavior.simparams_lat.sampling_scheme  = param_options[1][newparams[1]]
                    egobehavior.simparams_lon.sampling_scheme  = param_options[3][newparams[3]]
                    egobehavior.simparams_lat.smoothing        = param_options[2][newparams[2]][1]
                    egobehavior.simparams_lat.smoothing_counts = param_options[2][newparams[2]][2]
                    egobehavior.simparams_lat.smoothing        = param_options[4][newparams[4]][1]
                    egobehavior.simparams_lat.smoothing_counts = param_options[4][newparams[4]][2]
                    
                    simulate!(simlogs, behaviors, road, history, simparams)
                    # TODO(tim): compute histobin directly
                    (Δt_arr, Δy_arr) = calc_traceset_deviations(simlogs, history)
                    histobin = calc_histobin(Δt_arr, Δy_arr, histobin_params)
                    histobin .+= 1.0
                    KLdiv = KL_div_function(target_histobin_with_prior,histobin)

                    if KLdiv < best_KLdiv
                        best_KLdiv = KLdiv
                        paraminds[coordinate] = ip
                        converged = false
                        if verbosity > 0
                            println("\tfound better: ", coordinate, " -> ", param_options[coordinate][ip])
                            println("\t\tKL: ", best_KLdiv)
                        end
                    end

                end
            end

            # to_try = VehicleBehaviorEM[]
            # ip_arr = Int[]
            # for ip in 1 : length(param_options[coordinate])

            #     newparams = copy(paraminds)
            #     newparams[coordinate] = ip

            #     if !in(newparams, params_tried)
            #         push!(params_tried, newparams)
            #         push!(ip_arr, ip)
            #         push!(to_try, inds_to_simparams(newparams))
            #     end
            # end

            # if !isempty(to_try)

            #     tic()
            #     KL_divs = pmap(to_try) do sim_params

            #         B = calc_histobin(road, em, target_histobin, sim_params, histobin_params, drivelog)
            #         B .+= 1.0
            #         KL_div_function(A,B)
            #     end
            #     toc()

            #     if verbosity > 1
            #         for (i, kl_div) in enumerate(KL_divs)
            #             ip = ip_arr[i]
            #             println("\t coordinate -> ", param_options[coordinate][ip], "  ", kl_div)
            #         end
            #     end

            #     KL_divergence, best_ind = findmin(KL_divs)
            #     if KL_divergence < best_KLdiv
            #         best_KLdiv = KL_divergence
            #         paraminds[coordinate] = ip_arr[best_ind]
            #         converged = false
            #         if verbosity > 0
            #             println("\tfound better: ", coordinate, " -> ", param_options[coordinate][ip_arr[best_ind]])
            #             println("\t\tKL: ", best_KLdiv)
            #         end
            #     end
            # end
        end
    end

    if verbosity > 0
        toc()
        println("optimal params: ", paraminds)
        println("KL-divergence: ", best_KLdiv)
    end


    (ModelSimParams(
        param_options[1][paraminds[1]],
        param_options[2][paraminds[2]][1],
        param_options[2][paraminds[2]][2]),
     ModelSimParams(
        param_options[3][paraminds[3]],
        param_options[4][paraminds[4]][1],
        param_options[4][paraminds[4]][2]),
     best_KLdiv)
end