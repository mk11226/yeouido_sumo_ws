from pathlib import Path
import sys,json,hashlib,math,xml.etree.ElementTree as E
sys.path.insert(0,"/home/mk/bmil_ws")
from run_skku_commute import prepare_runtime
prepare_runtime()
import traci,sumolib
base=Path("/home/mk/bmil_ws/env/skku_commute")
out=Path("/home/mk/bmil_ws/results/skku_commute/buildings_signals_final_gui_seed42")
out.mkdir(parents=True,exist_ok=True)
net=sumolib.net.readNet(str(base/"skku_commute.net.xml"),withInternal=True)
cmd=[sumolib.checkBinary("sumo-gui"),"-c",str(base/"skku_commute.sumocfg"),"--seed","42","--step-length","0.5","--start","--quit-on-end","true","--delay","0","--no-step-log","true","--no-warnings","true","--duration-log.disable","true","--log",str(out/"visual_check.sumo.log")]
traci.start(cmd,label="skku_building_signal_visual_check",doSwitch=False,verbose=False)
c=traci.getConnection("skku_building_signal_visual_check")
report={"network_sha256":hashlib.sha256((base/"skku_commute.net.xml").read_bytes()).hexdigest()}
try:
 ids=c.polygon.getIDList()
 report["polygon_count"]=len(ids)
 report["building_outer_count"]=sum(i.startswith("building_") for i in ids)
 report["courtyard_mask_count"]=sum(i.startswith("courtyard_") for i in ids)
 assert report["polygon_count"]==18781
 c.gui.setSchema("View #0","SKKU campus commute")
 for i in range(1200):
  c.simulationStep()
  if "ego" not in c.vehicle.getIDList(): continue
  c.vehicle.setColor("ego",(255,40,40,255))
  c.vehicle.setShapeClass("ego","passenger/sedan")
  nexttls=c.vehicle.getNextTLS("ego")
  if nexttls and nexttls[0][3].lower()=="r" and nexttls[0][2]<60 and c.vehicle.getSpeed("ego")<.1:
   tls,link,gap,state=nexttls[0]
   x,y=c.vehicle.getPosition("ego")
   lane=c.vehicle.getLaneID("ego")
   end=net.getLane(lane).getShape()[-1]
   cx,cy=(x+end[0])/2,(y+end[1])/2
   c.gui.setBoundary("View #0",cx-75,cy-75,cx+75,cy+75)
   c.gui.screenshot("View #0",str(out/"gui_signal_closeup.png"))
   report.update(simulation_time_s=c.simulation.getTime(),ego_speed_mps=c.vehicle.getSpeed("ego"),ego_shape=c.vehicle.getShapeClass("ego"),ego_road=c.vehicle.getRoadID("ego"),next_tls_id=tls,next_tls_state=state,distance_to_tls_m=gap,view_schema=c.gui.getSchema("View #0"),view_center_xy=[cx,cy])
   c.simulationStep()
   break
 else: raise RuntimeError("No red-light stop captured in first 600 simulated seconds")
finally:
 c.close()
report["screenshot_exists"]=(out/"gui_signal_closeup.png").is_file()
polys=E.parse(base/"buildings.poly.xml").getroot().findall("poly")
report["building_polygons_with_vertices_in_view"]=sum(any(abs(float(q.split(",")[0])-cx)<75 and abs(float(q.split(",")[1])-cy)<75 for q in p.get("shape").split()) for p in polys if p.get("id").startswith("building_"))
(out/"visual_closeup_check.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))
