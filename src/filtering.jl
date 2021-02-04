# specification of particle filters for the bumper and lidar Roomba environments
# maintained by {jmorton2,kmenda}@stanford.edu

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
        x = temp_x = state.x
        y = temp_y = state.y
        contact = room_contact(r.room, Pos(x,y))
        # add noise to position without changing wall_contact
        temp_x += (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
        temp = Pos(temp_x, temp_y)
        if in_room(r.room, temp) && room_contact(r.room, temp) == contact
            x = temp_x
        end
        temp_y += (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
        temp = Pos(temp_x, temp_y)
        if in_room(r.room, temp) && room_contact(r.room, temp) == contact
            y = temp_y
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
        x = Inf
        y = Inf
        while !in_room(r.room, Pos(x, y))
            x = state.x + (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
            y = state.y + (rand(rng) - 0.5) * 2.0 * r.pos_noise_coeff
        end

        # add noise to orientation
        theta = state.theta + (rand(rng) - 0.5) * 2.0 * r.ori_noise_coeff

        push!(particles, RoombaState(x, y, theta, state.status))
    end
    return ParticleCollection(particles)
end
