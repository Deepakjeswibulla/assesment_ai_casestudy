import json, tempfile, unittest
import importlib.util
from pathlib import Path
from src.pipeline import load_capture, make_output, run
from src.evaluate import gate, evaluate
from src.calibration import calibrate
from src.geometry import polygon_area, wall
from src.stitching import place_rooms
from src.damage import concealed_flag
from src.schema import scope_item, measurement
from src.gates import assess, assess_repeat_wall, assess_openings
from src.reconstruction import _scale, process_lidar_sequence
from src.planes import extract_surface_candidates
from src.schema import measurement as measurement_record
from src.data.alignment import nearest_timestamp_index
from src.data.loader import inspect_capture
from src.data.sequence import select_representative_frames, read_odometry_by_frame
from src.room_geometry import reconstruct_room_geometry

class ContractTests(unittest.TestCase):
 def _synthetic_room_points(self):
  import numpy as np
  points=[]
  for x in np.linspace(0,4,41):
   for y in np.linspace(0,3,31):points.extend([[x,y,0],[x,y,2.5]])
  for y in np.linspace(0,3,31):
   for z in np.linspace(0,2.5,26):points.extend([[0,y,z],[4,y,z]])
  for x in np.linspace(0,4,41):
   for z in np.linspace(0,2.5,26):points.extend([[x,0,z],[x,3,z]])
  return np.asarray(points)

 def test_photo_requires_two_to_eight_per_room(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'capture.json';p.write_text(json.dumps({'tier':'photo','rooms':[{'id':'r','photos':['a.jpg']}]}))
   with self.assertRaises(ValueError): load_capture(p)
 def test_missing_room_identity_fails(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'capture.json';p.write_text(json.dumps({'tier':'video','rooms':[{'video':'v.mov'}]}))
   with self.assertRaises(ValueError): load_capture(p)
 def test_lidar_requires_conventions(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'capture.json';p.write_text(json.dumps({'tier':'lidar','rooms':[{'id':'r','depth':'d','poses':'p','intrinsics':'i'}]}))
   with self.assertRaises(ValueError): load_capture(p)
 def test_no_ground_truth_is_not_evaluable(self):
  self.assertEqual(gate(1.0,None,'length',.1,'m')['status'],'NOT_EVALUABLE')
 def test_missing_prediction_with_gt_fails(self):
  self.assertEqual(gate(None,2.0,'length',.1,'m')['status'],'FAIL')
 def test_uncalibrated_is_not_evaluable(self):
  self.assertEqual(evaluate({'calibration':{'status':'NOT_CALIBRATED'}},{'gates':{'x':{'value':1}}})['status'],'NOT_EVALUABLE')
 def test_calibration_separates_fit_and_holdout(self):
  r=calibrate([{'id':'a','raw':2,'measured':4,'unit':'m'},{'id':'b','raw':3,'measured':6,'unit':'m'},{'id':'c','raw':4,'measured':8,'unit':'m'}],[{'id':'d','raw':5,'measured':10,'unit':'m'},{'id':'e','raw':6,'measured':12,'unit':'m'}])
  self.assertEqual(r['status'],'CALIBRATED');self.assertEqual(r['scale'],2)
 def test_calibration_insufficient_is_closed(self):
  self.assertEqual(calibrate([],[])['status'],'NOT_CALIBRATED')
 def test_degenerate_footprint_rejected(self):
  with self.assertRaises(ValueError): polygon_area([[0,0],[1,1],[2,2]])
 def test_wall_has_metric_null_without_calibration(self):
  self.assertIsNone(wall([0,0],[1,0],room_id='r',wall_id='w',support=10,residual=0.01,confidence=None)['metric_length'])
 def test_adjacency_requires_explicit_correspondence(self):
  r=place_rooms([{'id':'a'},{'id':'b'}],[{'room_a':'a','room_b':'b'}]);self.assertFalse(r['adjacency']['a']);self.assertEqual(r['status'],'UNRESOLVED')
 def test_scope_requires_surface_and_anomaly(self):
  with self.assertRaises(ValueError): scope_item('', 'x')
 def test_interval_not_implied_by_score(self):
  m=measurement(1,'m',status='PROVISIONAL',evidence='depth');self.assertEqual(m['confidence_interval']['status'],'UNESTABLISHED')
 def test_photo_wall_threshold_is_eight_percent(self):
  r=assess('photo_wall_relative_error',0.07,0,unit='fraction',calibrated=True)
  self.assertEqual(r['status'],'PASS')
 def test_metric_gate_without_calibration_is_not_evaluable(self):
  self.assertEqual(assess('ceiling_error_m',0.001,0,unit='m')['status'],'NOT_EVALUABLE')
 def test_repeatability_uses_one_cm_or_half_percent(self):
  self.assertEqual(assess_repeat_wall(4.015,4.0,calibrated=True)['status'],'PASS')
 def test_opening_miss_and_phantom_fail(self):
  r=assess_openings([{'gt_id':'door','width':0.8,'unit':'m'},{'gt_id':'ghost','width':1,'unit':'m'}],[{'id':'door','width':0.8,'unit':'m'},{'id':'window','width':1.0,'unit':'m'}],calibrated=True)
  self.assertEqual(r['status'],'FAIL');self.assertEqual(r['missed'],1);self.assertEqual(r['phantom'],1)
 def test_depth_unit_conversion(self):
  self.assertEqual(_scale('mm'),0.001)
 def test_ambiguous_depth_unit_rejected(self):
  with self.assertRaises(ValueError): _scale('unknown')
 def test_plane_candidates_are_deterministic_and_provisional(self):
  pts=[[x/10,y/10,0] for x in range(10) for y in range(10)]+[[x/10,y/10,3] for x in range(10) for y in range(10)]
  r=extract_surface_candidates(pts,[0,0,1],min_support=10,max_planes=3)
  self.assertEqual(r['status'],'PROVISIONAL');self.assertTrue(all(x.get('validation_status','PROVISIONAL')=='PROVISIONAL' for x in r['surfaces']))
 def test_invalid_world_up_rejected(self):
  with self.assertRaises(ValueError): extract_surface_candidates([[0,0,0]]*20,'z')
 def test_concealed_flag_is_inspection_trigger_only(self):
  self.assertEqual(concealed_flag('R1','wall-a','missing data','trigger')['state'],'INSPECTION_TRIGGER_ONLY')
 def test_metric_value_and_interval_are_null(self):
  out=make_output({'tier':'photo','rooms':[{'id':'r'}]},[], 'run')
  m=out['rooms'][0]['floor']['area'];self.assertIsNone(m['value']);self.assertIsNone(m['confidence_interval']['lower'])
  self.assertEqual(out['rooms'][0]['surface_candidates'],[])
 def test_shared_measurement_contract_keeps_unvalidated_metric_null(self):
  m=measurement_record(12,'raw-unit',raw_value=12,raw_unit='raw-unit',metric_value=3.2,metric_unit='m',status='PROVISIONAL',evidence=['depth-frame-1'])
  self.assertEqual(m['raw'],{'value':12,'unit':'raw-unit'});self.assertIsNone(m['metric']['value']);self.assertIsNone(m['value']);self.assertIsNone(m['unit'])
 def test_validated_measurement_requires_complete_interval(self):
  with self.assertRaises(ValueError): measurement_record(1,'m',status='VALIDATED',lower=0.9)
  m=measurement_record(1,'m',status='VALIDATED',lower=0.9,upper=1.1,metric_value=1,metric_unit='m')
  self.assertEqual(m['confidence_interval']['status'],'ESTABLISHED')
 def test_timestamp_alignment_is_tolerance_gated_and_deterministic(self):
  self.assertEqual(nearest_timestamp_index([1.0,1.1,1.2],1.11,tolerance=0.02),(1,0.010000000000000009))
  self.assertEqual(nearest_timestamp_index([1.0],2.0,tolerance=0.1)[0],None)
 def test_inventory_aligns_frame_ids_without_positional_assumptions(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);(root/'depth').mkdir();(root/'confidence').mkdir()
   # Minimal PNG headers suffice for the read-only metadata inventory.
   import struct
   def png(path):
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+struct.pack('>I',13)+b'IHDR'+struct.pack('>IIBBBBB',4,3,16,0,0,0,0))
   png(root/'depth'/'000002.png');png(root/'confidence'/'000001.png');png(root/'confidence'/'000002.png')
   alignment=inspect_capture(root,hash_files=False)['frame_alignment']['value']
   self.assertEqual(alignment['matching_ids'],['000002'])
   self.assertEqual(alignment['confidence_without_depth'],['000001'])
 def test_frame_sampling_covers_trajectory_deterministically(self):
  self.assertEqual(select_representative_frames(['000000','000001','000002','000003','000004'],3),['000000','000002','000004'])
 def test_odometry_rows_are_keyed_by_frame_id(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'odometry.csv';p.write_text('timestamp, frame, x, y, z, qx, qy, qz, qw\n1.0, 000007, 0, 0, 0, 0, 0, 0, 1\n')
   rows=read_odometry_by_frame(p);self.assertEqual(rows['000007']['qw'],'1')
 @unittest.skipUnless(importlib.util.find_spec('cv2') is not None,'OpenCV is optional in the lightweight test runtime')
 def test_png_csv_lidar_adapter_is_deterministic_and_keeps_metric_null(self):
  import cv2, numpy as np
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);depth=root/'depth';depth.mkdir()
   for frame in range(3):cv2.imwrite(str(depth/f'{frame:06d}.png'),np.full((16,16),2000,dtype=np.uint16))
   (root/'odometry.csv').write_text('timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy\n'+'\n'.join(f'{i/30}, {i:06d}, 0, 0, 0, 0, 0, 0, 1, 20, 20, 7.5, 7.5' for i in range(3)))
   room={'depth_dir':'depth','odometry_csv':'odometry.csv','depth_unit':'mm','pose_unit':'m','quaternion_order':'xyzw','pose_convention':'camera_to_world','coordinate_handedness':'right-handed','world_up':[0,0,1],'intrinsics_grid':'depth','max_frames':2}
   result=process_lidar_sequence(room,root)
   self.assertEqual(result['selected_frame_ids'],['000000','000002']);self.assertEqual(result['point_count'],512)
   self.assertTrue(all(s.get('observed_coverage_area',{}).get('metric_value') is None for s in result['surface_candidates']))
 def test_room_geometry_builds_footprint_from_supported_wall_intersections(self):
  result=reconstruct_room_geometry(self._synthetic_room_points(),[0,0,1],plane_tolerance=.005,min_plane_support=100,min_wall_support=100,seed=1)
  self.assertEqual(result['status'],'PROVISIONAL');self.assertEqual(len(result['walls']),4)
  self.assertAlmostEqual(result['footprint']['area_raw'],12.0,places=5)
  self.assertIsNone(result['footprint']['area_metric']);self.assertTrue(all(w['metric_length'] is None for w in result['walls']))
 def test_room_geometry_respects_tilted_world_up(self):
  import numpy as np
  angle=np.deg2rad(17);rotation=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
  points=self._synthetic_room_points()@rotation.T;up=rotation@np.array([0,0,1])
  result=reconstruct_room_geometry(points,up,plane_tolerance=.005,min_plane_support=100,min_wall_support=100,seed=1)
  self.assertEqual(result['status'],'PROVISIONAL');self.assertAlmostEqual(result['footprint']['area_raw'],12.0,places=4)
 def test_room_geometry_stays_unresolved_with_sparse_support(self):
  result=reconstruct_room_geometry(self._synthetic_room_points()[:15],[0,0,1],plane_tolerance=.005,min_plane_support=10,min_wall_support=10,seed=1)
  self.assertEqual(result['status'],'UNRESOLVED');self.assertIsNone(result['footprint'])
 def test_one_command_pipeline_emits_contract_and_rendered_plan(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d)/'capture';root.mkdir();out=Path(d)/'output'
   (root/'depth.json').write_text(json.dumps({'data':[[2.0]*8 for _ in range(8)],'unit':'m','timestamp_seconds':1.0}))
   (root/'poses.json').write_text(json.dumps([{'quaternion':[0,0,0,1],'translation':[0,0,0],'timestamp_seconds':1.0,'unit':'m'}]))
   (root/'intrinsics.json').write_text(json.dumps({'fx':10,'fy':10,'cx':3.5,'cy':3.5,'reference_width':8,'reference_height':8}))
   (root/'capture.json').write_text(json.dumps({'capture_id':'synthetic','tier':'lidar','rooms':[{'id':'room-test','depth':'depth.json','poses':'poses.json','intrinsics':'intrinsics.json','depth_unit':'m','pose_unit':'m','quaternion_order':'xyzw','pose_convention':'camera_to_world','rgb_depth_registration':None,'coordinate_handedness':'right-handed','world_up':[0,0,1],'intrinsics_grid':'depth','max_pose_delta_s':0.01}]}))
   run(root,out)
   result=json.loads((out/'result.json').read_text());room=result['rooms'][0]
   self.assertIsNone(room['floor']['area']['value']);self.assertTrue((out/'floorplan.svg').is_file());self.assertTrue((out/'run_manifest.json').is_file())

if __name__=='__main__': unittest.main()
