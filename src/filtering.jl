# specification of particle filters for the bumper and lidar Roomba environments
# maintained by {jmorton2,kmenda}@stanford.edu

function ParticleFilters.predict!(pm, m::RoombaPOMDP, b::ParticleCollection, a, rng)
    for i in 1:n_particles(b)
        pm[i] = isterminal(m, b.particles[i]) ? initialstate(m, rng) : @gen(:sp)(m, b.particles[i], a, rng)
    end 
end

function ParticleFilters.reweight!(wm, m::RoombaPOMDP, b::ParticleCollection, a, pm, o)
    for i in 1:n_particles(b)
        wm[i] = isterminal(m, pm[i]) ? 0 : obs_weight(m, rand(b.particles), a, pm[i], o)
    end
end

struct BumperResampler
    # number of particles
    n::Int
    # roomba model
    room::Room
    # coefficient of noises in pos
    pos_noise_coeff::Float64
    # coefficient of noises in orientation
    ori_noise_coeff::Float64
end
function BumperResampler(n, m::RoombaModel, pos, ori)
    return BumperResampler(n, mdp(m).room, pos, ori)
end
BumperResampler(n, m) = BumperResampler(n, m, 0.3, 0.1)
BumperResampler(m) = BumperResampler(5000, m)

function ParticleFilters.resample(r::BumperResampler, b::WeightedParticleBelief{RoombaState}, rng::AbstractRNG)
    particles = RoombaState[]
    for i in 1:r.n
        state = rand(rng, b)
        contact = wall_contact(r.room, state)

        # add noise to position without changing wall_contact
        temp = [state.x, state.y]
        x, y = [temp...]
        temp[1] += (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
        if in_room(r.room, SVector(temp...)) && wall_contact(r.room, SVector(temp...)) == contact
            x = temp[1]
        else
            temp[1] = x
        end
        temp[2] += (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
        if in_room(r.room, SVector(temp...)) && wall_contact(r.room, SVector(temp...)) == contact
            y = temp[2]
        end

        # add noise to orientation
        theta = state.theta + (rand(rng) - 0.5) * 2.0 * r.ori_noise_coeff

        push!(particles, RoombaState(x, y, theta, state.status))
    end
    return ParticleCollection(particles)
end

struct LidarResampler
    # number of particles
    n::Int
    # roomba model
    room::Room
    # coefficient of noises in pos
    pos_noise_coeff::Float64
    # coefficient of noises in orientation
    ori_noise_coeff::Float64
end
function LidarResampler(n, m::RoombaModel, pos, ori)
    return LidarResampler(n, mdp(m).room, pos, ori)
end
LidarResampler(n, m) = LidarResampler(n, m, 0.3, 0.1)
LidarResampler(m) = LidarResampler(5000, m)

function ParticleFilters.resample(r::LidarResampler, b::WeightedParticleBelief{RoombaState}, rng::AbstractRNG)
    particles = RoombaState[]
    for i in 1:r.n
        state = rand(rng, b)

        # add noise to position without changing wall_contact
        temp = [Inf, Inf]
        while !in_room(r.room, SVector(temp...))
            temp = [state.x, state.y]
            temp[1] += (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
            temp[2] += (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
        end
        x, y = [temp...]

        # add noise to orientation
        theta = state.theta + (rand(rng) - 0.5) * 2.0 * r.ori_noise_coeff

        push!(particles, RoombaState(x, y, theta, state.status))
    end
    return ParticleCollection(particles)
end