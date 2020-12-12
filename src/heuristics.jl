struct Running <: Policy
    m::RoombaMDP
end
Running(p::RoombaModel) = Running(mdp(p))

function POMDPs.action(p::Running, s::RoombaState)
    goal_x, goal_y = get_goal_xy(p.m)
    x,y,th = s[1:3]
    ang_to_goal = atan(goal_y - y, goal_x - x)
    del_angle = wrap_to_pi(ang_to_goal - th)
    
    # apply proportional control to compute the turn-rate
    Kprop = 1.0
    om = Kprop * del_angle
    # find the closest option in action space
    if !(typeof(p.m.aspace) <: RoombaActions)
        _,ind = findmin(abs.(om .- p.m.aspace))
        om = p.m.aspace[ind]
    end
    # always travel at some fixed velocity
    v = p.m.v_max
    
    return RoombaAct(v, om)
end

POMDPs.action(p::Running, b::AbstractParticleBelief) = action(p, mode(b))

POMDPs.action(p::Running, b::Any) = action(p, rand(b))

struct RunningSolver <: Solver end
POMDPs.solve(solver::RunningSolver, p::RoombaModel) = Running(p)