"""User-confirmed signal sequencing corrections after netconvert generation."""
import xml.etree.ElementTree as ET


def protect_yeoui2_left(path):
    """Yeoui 2 south end: both through -> opposing clearance -> through+left.

    netconvert's opposites layout adds permissive left greens during opposing
    through traffic. These two left-only lanes must instead wait for a protected
    arrow. Keep the generated timing/other approaches and reuse the junction's
    geometry-adjusted clearance duration while our through traffic continues.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    approach, opposing = '1180030100', '1180031402'
    connections = [c for c in root.findall('connection') if c.get('tl')]
    ours = [c for c in connections if c.get('from') == approach]
    if not ours:
        return False
    tids = {c.get('tl') for c in ours}
    if len(tids) != 1:
        raise ValueError('여의2교 남단 진입로의 신호 제어기를 하나로 정할 수 없습니다.')
    tid = next(iter(tids))
    left = {int(c.get('linkIndex')) for c in ours if c.get('dir') in ('l', 'L')}
    straight = {int(c.get('linkIndex')) for c in ours if c.get('dir') == 's'}
    opposite = {int(c.get('linkIndex')) for c in connections
                if c.get('from') == opposing and c.get('tl') == tid and c.get('dir') == 's'}
    if not left or not straight or not opposite:
        raise ValueError('여의2교 남단 좌회전·직진·맞은편 직진 연결이 누락됐습니다.')
    programs = [t for t in root.findall('tlLogic') if t.get('id') == tid]
    for program in programs:
        phases = program.findall('phase')
        clearance = max((float(p.get('duration')) for p in phases
                         if set(p.get('state')) == {'r'}), default=3.0)
        protected = [p for p in phases if all(p.get('state')[i] == 'G' for i in left | straight)
                     and all(p.get('state')[i] == 'r' for i in opposite)]
        if len(protected) != 1 or any(p.get('next') for p in phases):
            raise ValueError('여의2교 남단 보호 좌회전 순서를 안전하게 수정할 수 없습니다.')
        for phase in phases:
            state = list(phase.get('state'))
            for i in left:
                if state[i] == 'g' or any(state[j] in 'Ggy' for j in opposite):
                    state[i] = 'r'
            phase.set('state', ''.join(state))
        target = protected[0]
        previous = phases[phases.index(target) - 1]
        state = previous.get('state')
        if any(state[i] == 'y' for i in opposite):
            # Clear opposing traffic without interrupting our through green.
            clear_state = ''.join('r' if c == 'y' or i in left else c
                                  for i, c in enumerate(state))
            gap = ET.Element('phase', duration=f'{clearance:g}', state=clear_state)
            gap.tail = previous.tail
            program.insert(list(program).index(target), gap)
        elif not (all(state[i] == 'r' for i in opposite | left)
                  and float(previous.get('duration')) >= clearance):
            raise ValueError('여의2교 남단 맞은편 직진 종료 후 비움 구간이 없습니다.')
        for phase in program.findall('phase'):
            state = phase.get('state')
            if any(state[i] in 'Ggy' for i in opposite) and any(state[i] in 'Gg' for i in left):
                raise ValueError('맞은편 직진과 좌회전 녹색이 겹칩니다.')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return True
