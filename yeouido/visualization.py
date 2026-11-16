"""Color underpass links (standard node-link ROAD_TYPE=002) light gray.

Build a display copy of the network; geometry, lanes, connections, signals,
routes, and the original network remain unchanged.
"""
import json
import xml.etree.ElementTree as ET

from yeouido.config import NET, OUT

VISUAL_NET = OUT / "yeouido.underpass.net.xml"


def prepare_underpass_view(settings_path):
    tree = ET.parse(NET)
    root = tree.getroot()
    edges = {edge.get("id"): edge for edge in root.findall("edge")}
    underpasses = {eid for eid, edge in edges.items()
                   if edge.get("type", "").split(".")[-1] == "002"}
    # Color internal connections only when both connecting roads are underpasses.
    internal = set()
    for connection in root.findall("connection"):
        if connection.get("from") in underpasses and connection.get("to") in underpasses:
            via = connection.get("via", "")
            if via:
                internal.add(via.rsplit("_", 1)[0])
    for eid, edge in edges.items():
        for old in list(edge.findall("param")):
            if old.get("key") == "visual_underpass":
                edge.remove(old)
        ET.SubElement(edge, "param", key="visual_underpass",
                      value="1" if eid in underpasses or eid in internal else "0")
    tree.write(VISUAL_NET, encoding="utf-8", xml_declaration=True)
    settings_path.write_text('''<viewsettings>
    <scheme name="underpass_gray">
        <background backgroundColor="242,239,233"/>
        <vehicles vehicleQuality="2" vehicle_minSize="1.00" vehicle_exaggeration="1.00" vehicle_constantSize="0"/>
        <edges laneEdgeMode="31" edgeParam="visual_underpass">
            <colorScheme name="by param (numerical, streetwise)" interpolated="false">
                <entry color="45,45,45" threshold="0"/>
                <entry color="170,170,170" threshold="1"/>
            </colorScheme>
        </edges>
        <pois poiType_show="1" poiType_size="60" poiType_color="255,0,0"/>
    </scheme>
</viewsettings>
''', encoding="utf-8")
    report = {"road_type": "002", "underpass_edges": sorted(underpasses),
              "count": len(underpasses), "gray_rgb": [170, 170, 170],
              "internal_connections": sorted(internal),
              "source_net": NET.name, "display_net": VISUAL_NET.name}
    (OUT / "underpass_visualization.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report
