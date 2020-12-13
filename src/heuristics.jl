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
    # always travel at some fixed velocity
    v = p.m.v_max
    # find the closest option in action space
    if typeof(p.m.aspace) <: RoombaActions
        return RoombaAct(v, om)
    else
        _, ind = findmin([((act.omega-om)/p.m.om_max)^2 + ((act.v-v)/p.m.v_max)^2  for act in p.m.aspace])
        return p.m.aspace[ind]
    end
end

POMDPs.action(p::Running, b::AbstractParticleBelief) = action(p, mode(b))

POMDPs.action(p::Running, b::Any) = action(p, rand(b))

struct RunningSolver <: Solver end
POMDPs.solve(solver::RunningSolver, p::RoombaModel) = Running(p)