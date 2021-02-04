# A L-shape room with 6 walls
# Define constants  -- all units in m
 # countrl the room width; the width of the corridor is twice as much
#as ROOM_W; should be in (0,25)
mutable struct ROBOT_W_struct
    val::Float64 # robot width
end
const DEFAULT_ROBOT_W = 1.0
const DEFAULT_R = DEFAULT_ROBOT_W / 2.
const ROBOT_W = ROBOT_W_struct(DEFAULT_ROBOT_W)
const ROOM_MARGIN = 1e-9
const SQRT_2 = sqrt(2)

function round_corners(sspace, corners)

    if sspace isa DiscreteRoombaStateSpace
        for i in 1:4
            xi = floor(Int, (corners[i,1] - sspace.XLIMS[1]) / sspace.x_step + 0.5) + 1
            yi = floor(Int, (corners[i,2] - sspace.YLIMS[1]) / sspace.y_step + 0.5) + 1
            corners[i,1] = sspace.XLIMS[1] + (xi-1) * sspace.x_step
            corners[i,2] = sspace.YLIMS[1] + (yi-1) * sspace.y_step
        end
    end
    return corners
end

mutable struct Room
    corners::Array{Float64, 2}
    poscorners::Array{Float64, 2}
    segments::Array{LineSegment, 1}
    possegments::Array{LineSegment, 1}
    goal_wall::Int
    stair_wall::Int
    contact_walls::Vector{Int}
    goal_segment::LineSegment  # Index of wall that leads to goal
    stair_segment::LineSegment # Index of wall that leads to stairs
    xl::Float64
    xu::Float64
    yl::Float64
    yu::Float64
    tpoint::Array{Float64, 1}#Turning point
# ROOM_W:countrl the room width; the width of the corridor is twice as
# much as ROOM_W; should in (2,10) or cloud have unknown mistakes
    function Room(sspace; configuration=1,ROOM_W::Float64 = 5.0)
        retval = new()
         # Define different configurations for stair and goal locations
         # Wall start from the laft and is clockwidth)
        if configuration > 4
            configuration = 1
        end
        if configuration == 1
            retval.goal_wall = 3
            retval.stair_wall = 4
        elseif configuration == 2
            retval.goal_wall = 6
            retval.stair_wall = 1
        elseif configuration == 3
            retval.goal_wall = 3
            retval.stair_wall = 1
        else #  configuration == 4
            retval.goal_wall = 6
            retval.stair_wall = 4
        end
        retval.contact_walls = Int[]
        retval.goal_segment = goal_wall_segment(retval.goal_wall,ROOM_W)
        retval.stair_segment = goal_wall_segment(retval.stair_wall,ROOM_W)
        #corners(start from the bottom left and is clockwidth)
        retval.corners = round_corners(sspace,[[-20-ROOM_W -20]; [-20-ROOM_W 0+ROOM_W];
        [10+ROOM_W 0+ROOM_W]; [10+ROOM_W 0-ROOM_W];
        [-20+ROOM_W 0-ROOM_W];[-20+ROOM_W -20]])
        retval.poscorners = round_corners(sspace,[[-20-ROOM_W+DEFAULT_R -20+DEFAULT_R];
        [-20-ROOM_W+DEFAULT_R 0+ROOM_W-DEFAULT_R];
        [10+ROOM_W-DEFAULT_R 0+ROOM_W-DEFAULT_R];
        [10+ROOM_W-DEFAULT_R 0-ROOM_W+DEFAULT_R];
        [-20+ROOM_W-DEFAULT_R 0-ROOM_W+DEFAULT_R];
        [-20+ROOM_W-DEFAULT_R -20+DEFAULT_R]])
        retval.xl = retval.poscorners[1,1]
        retval.xu = retval.poscorners[3,1]
        retval.yl = retval.poscorners[1,2]
        retval.yu = retval.poscorners[2,2]
        # a little diffrence in this corner
        retval.tpoint = SVector(retval.poscorners[5,1],retval.poscorners[5,2])
        retval.segments = [LineSegment(retval.corners[i, :], retval.corners[i+1, :]) for i =1:5]
        push!(retval.segments, LineSegment(retval.corners[1, :], retval.corners[6, :]))
        retval.possegments = [LineSegment(retval.poscorners[i, :], retval.poscorners[i+1, :]) for i =1:5]
        push!(retval.possegments, LineSegment(retval.poscorners[1, :], retval.poscorners[6, :]))
        return retval
    end
end

function init_pos(r::Room, rng)
    w = r.xu - r.xl
    h = r.yu - r.yl
    i=1
    while i<1000
        i=i+1
        init_pos = SVector(rand(rng)*w + r.xl, rand(rng)*h + r.yl)
        if init_pos[1]<r.tpoint[1]||init_pos[2]>r.tpoint[2]
            return init_pos
        end
    end
    @assert "can not init position, recheck the room"
end

function in_room(r::Room, pos::Pos)
    if r.xl-ROOM_MARGIN< pos[1] < r.xu+ROOM_MARGIN &&
        r.yl-ROOM_MARGIN < pos[2] < r.yu+ROOM_MARGIN &&
        (r.tpoint[1]+ROOM_MARGIN > pos[1]||r.tpoint[2]-ROOM_MARGIN < pos[2])
        return true
    end

    return false
end
# for determing
function room_contact(r::Room,pos::Pos)
    #if roomba contact any wall
    if r.xl > pos[1] - ROOM_MARGIN
        return true
    end
    if r.xu < pos[1] + ROOM_MARGIN
        return true
    end
    if r.yl > pos[2] - ROOM_MARGIN
        return true
    end
    if r.yu < pos[2] + ROOM_MARGIN
        return true
    end
    if r.tpoint[2] > pos[2] - ROOM_MARGIN && r.tpoint[1] < pos[1] + ROOM_MARGIN
        return true
    end
    return false
end
# possible contact walls
# pos1:start point
# pos2:end point
function collapse_detect(r::Room, pos1::Pos, pos2::Pos)
    contact_walls = r.contact_walls
    empty!(contact_walls)
    if r.xl > pos2[1] - ROOM_MARGIN
        push!(contact_walls,1)
    end
    if r.xu < pos2[1] + ROOM_MARGIN
        push!(contact_walls,3)
    end
    if r.yl > pos2[2] + ROOM_MARGIN
        push!(contact_walls,6)
    end
    if r.yu < pos2[2] + ROOM_MARGIN
        push!(contact_walls,2)
    end
    if r.tpoint[1] < pos2[1] + ROOM_MARGIN && r.tpoint[2] > pos2[2] - ROOM_MARGIN
        if r.tpoint[1] > pos1[1] - ROOM_MARGIN
            push!(contact_walls,5)
        end
        if r.tpoint[2] < pos1[2] + ROOM_MARGIN
            push!(contact_walls,4)
        end
    end
    if (pos1[1] < r.tpoint[1] < pos2[1]&&pos1[2] < r.tpoint[2] < pos2[2])||
        (pos1[1] > r.tpoint[1] > pos2[1]) && (pos1[2] > r.tpoint[2] > pos2[2])
        # not every conditions are considerd
        if pos1[1]<pos2[1]
            #keep p2<p1
            temp1=pos1
            temp2=pos2
        else
            temp1=pos2
            temp2=pos1
        end
        #
        tan_tp1_tp2=(temp2[2]-temp1[2])/(temp2[1]-temp1[1])
        tan_tp1_tpoint=(r.tpoint[2]-temp1[2])/(r.tpoint[1]-temp1[1])
        if tan_tp1_tp2 > tan_tp1_tpoint
            if pos2[1]>pos1[1]
                push!(contact_walls,5)
            else
                push!(contact_walls,4)
            end
        end
    end
#=
    if r.tpoint[1] < pos2[1]  && r.xu > pos2[1] # out of L, near by the Turning point
        if r.tpoint[1] < pos1[1]
            push!(contact_walls,4)
        elseif r.tpoint[2] > pos1[2]
            push!(contact_walls,5)
        else
            push!(contact_walls,4)
            push!(contact_walls,5)
        end
    end
=#
    return contact_walls

end

# determines if pos (center of robot) is intersecting with a segment;should
# replaced with calculating the distance between point and LineSegment
function segment_contact(seg::LineSegment,pos::Pos)
    # vertical line
    if abs(seg.p2[1]-seg.p1[1])<ROOM_MARGIN
        if abs(pos[1]-seg.p2[1])<ROOM_MARGIN+DEFAULT_R&&
            pos[2]<max(seg.p2[2],seg.p1[2])&&
            pos[2]>min(seg.p2[2],seg.p1[2])
            return true
        end
        return false
        # horizonal line
    elseif abs(pos[2]-seg.p2[2])<ROOM_MARGIN+DEFAULT_R&&
        pos[1]<max(seg.p2[1],seg.p1[1])&&
        pos[1]>min(seg.p2[1],seg.p1[1])
        return true
    end
    return false
end
    # pos1:start point
    # pos2:end point
function legal_translate(r::Room, pos::Pos, heading::Pos, des_step::Float64)
     pos1 = pos
     if des_step == 0
         return pos1
     end
     pos2 = pos1 + des_step*heading
     contact_walls = collapse_detect(r,pos1,pos2)
     if length(contact_walls) == 0
         return pos2
     end
    #calculate the shortest distance (which means the first wall roomba meets)
    length_min=minimum(ray_length(r.possegments[wall_index], pos1, heading) for wall_index in contact_walls)
#    if length_min > 20 || length_min < -0.1
#        @show contact_walls,pos1,heading
#    end
    length_min=min(des_step,length_min) #might a useless check ?
    pos2 = pos1 + length_min*heading
    return pos2
end

# Render room based on segments
function render(r::Room, ctx::CairoContext)
    for seg in r.segments
        render(seg, ctx)
    end
    render(r.goal_segment,ctx,goal=true)
    render(r.stair_segment,ctx,stair=true)
end
