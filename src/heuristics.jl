struct Running{AS,SS} <: Policy
    m::RoombaMDP{SS,AS}
end
Running(p::RoombaModel) = Running(mdp(p))

function POMDPs.action(p::Running{AS}, s::RoombaState) where AS <: AbstractVector{RoombaAct}
    om = delegate_angle(p, s)
    m = p.m
    v_max = m.v_max
    o_max = m.om_max
    as = m.aspace
    best_act = first(as)
    best_loss = Inf
    for act in as
        new_loss = ((act.omega-om)/o_max)^2 + (act.v/v_max-1.0)^2
        if new_loss < best_loss
            best_act = act
            loss = new_loss
        end
    end
    return best_act
end

POMDPs.action(p::Running{AS}, s::RoombaState) where AS <: RoombaActions = RoombaAct(p.m.v_max, delegate_angle(p, s))

function delegate_angle(p::Running, s::RoombaState)
    x,y,th,_ = s
    if (p.m.room.goal_wall == 3 && y < -5) || (p.m.room.goal_wall == 6 && x > -15)
        goal_x, goal_y = -20, 0
    else
        goal_x, goal_y = get_goal_xy(p.m)
    end
    ang_to_goal = atan(goal_y - y, goal_x - x)
    del_angle = wrap_to_pi(ang_to_goal - th)
    # apply proportional control to compute the turn-rate
    return 1.0 * del_angle
end

POMDPs.action(p::Running, b::AbstractParticleBelief) = action(p, rand(b))

POMDPs.action(p::Running, b::Any) = action(p, rand(b))

struct RunningSolver <: Solver end
POMDPs.solve(solver::RunningSolver, p::RoombaModel) = Running(p)