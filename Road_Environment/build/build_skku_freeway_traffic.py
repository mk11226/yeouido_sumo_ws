"""Additional trips along open multilane roads, in both travel directions."""
import math


def routes(net, path, segment_vph=100):
    def is_open(edge):
        return edge.getLaneNumber() >= 2 and (edge.getType() in ('highway.trunk', 'highway.motorway')
                                              or edge.getSpeed() >= 19.4)

    def vector(edge):
        shape = edge.getShape()
        dx, dy = shape[-1][0]-shape[0][0], shape[-1][1]-shape[0][1]
        norm = math.hypot(dx, dy) or 1
        return dx/norm, dy/norm

    def opposite(edge):
        x,y = edge.getShape()[len(edge.getShape())//2]
        dx,dy = vector(edge)
        candidates=[]
        for e,distance in net.getNeighboringEdges(x,y,100):
            if e.getFunction() or not e.allows('passenger') or not is_open(e):continue
            ux,uy=vector(e)
            if dx*ux+dy*uy < -.7:candidates.append((distance,e.getID(),e))
        return min(candidates,default=(0,'',None))[2]

    sections=[];current=[]
    for edge in path:
        if is_open(edge):current.append(edge)
        elif current:sections.append(current);current=[]
    if current:sections.append(current)
    trips=[];seen=set()
    for section in sections:
        if sum(e.getLength() for e in section)<1500:continue
        start=0
        while start<len(section)-1:
            end=start+1;distance=section[start].getLength()
            while end<len(section) and distance<4000:
                distance+=section[end].getLength();end+=1
            part=section[start:end]
            if distance>=1200 and part[0].getID()!='507449308':
                key=tuple(e.getID() for e in part)
                if key not in seen:
                    seen.add(key);trips.append((f'freeway_extra_{len(trips):02d}',part,segment_vph*4.5,'freeway_extra'))
                a,b=opposite(part[-1]),opposite(part[0])
                if a is not None and b is not None:
                    reverse,_=net.getFastestPath(a,b,vClass='passenger')
                    if reverse and all(is_open(e) for e in reverse) and sum(e.getLength() for e in reverse)<6000:
                        key=tuple(e.getID() for e in reverse)
                        if key not in seen:
                            seen.add(key);trips.append((f'freeway_opposing_{len(trips):02d}',list(reverse),segment_vph*4.5,'freeway_opposing'))
            moved=0
            while start<len(section) and moved<2000:
                moved+=section[start].getLength();start+=1
    return trips
