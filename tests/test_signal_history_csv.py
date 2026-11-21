import csv
import datetime as dt
import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path
from zoneinfo import ZoneInfo

from yeouido.tdata_signals import FIELDS, extract


class SignalHistoryCSVTests(unittest.TestCase):
    def inputs(self, folder, samples, zipped=False):
        folder = Path(folder)
        catalog = folder / 'crossroads.json'
        catalog.write_text(json.dumps({'intersections': [
            {'itst_id': '148', 'name': 'test', 'lat': 37.52, 'lon': 126.92}]}))
        header = ['DATA_ID', 'TRSM_UTC_TIME', 'TRSM_YEAR', 'TRSM_MT', 'TRSM_DY',
                  'TRSM_HR', 'TRSM_TM', 'TRSM_MS', 'ITST_ID', 'EQMN_ID', *FIELDS]
        stream = io.StringIO(newline='')
        writer = csv.writer(stream)
        writer.writerow(header)
        start = dt.datetime(2026, 11, 16, 8, tzinfo=ZoneInfo('Asia/Seoul')).timestamp()
        for second, state, hour, itst in samples:
            writer.writerow(['sample', int((start + second) * 1000), '2026', '11', '16',
                             hour, '080000', '000', itst, 'CIB', state, *[''] * (len(FIELDS) - 1)])
        payload = ('\ufeff' + stream.getvalue()).encode()
        history = folder / ('history.zip' if zipped else 'history.csv')
        if zipped:
            with zipfile.ZipFile(history, 'w') as archive:
                archive.writestr('signalPhase.csv', payload)
        else:
            history.write_bytes(payload)
        return history, catalog, folder / 'observations.json'

    def test_out_of_order_packets_are_sorted_before_compression(self):
        with tempfile.TemporaryDirectory() as folder:
            args = self.inputs(folder, [(2, 'protected-Movement-Allowed', 8, '148'),
                                       (0, 'protected-Movement-Allowed', 8, '148'),
                                       (1, 'protected-Movement-Allowed', 8, '148')])
            data = extract(*args)
        runs = data['intersections']['148']['levels']['rush']
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0][1] - runs[0][0], 2)
        self.assertEqual(data['out_of_order_packets'], 2)

    def test_csv_and_zip_have_identical_observed_runs(self):
        results = []
        for zipped in (False, True):
            with tempfile.TemporaryDirectory() as folder:
                args = self.inputs(folder, [(0, 'stop-And-Remain', 8, '148'),
                                           (10, 'stop-And-Remain', 8, '148')], zipped)
                data = extract(*args)
                results.append(data['intersections'])
        self.assertEqual(results[0], results[1])
        self.assertEqual(len(results[0]['148']['levels']['rush']), 2)

    def test_conflicting_duplicate_is_unknown_and_not_silently_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            args = self.inputs(folder, [(0, 'protected-Movement-Allowed', 8, '148'),
                                       (0, 'stop-And-Remain', 8, '148')])
            data = extract(*args)
        self.assertEqual(data['conflicting_timestamps'], 1)
        self.assertEqual(data['intersections']['148']['levels']['rush'][0][2], '?' * len(FIELDS))

    def test_other_locations_and_unselected_hours_are_excluded(self):
        with tempfile.TemporaryDirectory() as folder:
            args = self.inputs(folder, [(0, 'stop-And-Remain', 9, '148'),
                                       (0, 'stop-And-Remain', 8, '999'),
                                       (0, 'stop-And-Remain', 8, '148')])
            data = extract(*args)
        self.assertEqual(data['selected_rows'], 1)
        self.assertEqual(set(data['intersections']), {'148'})


if __name__ == '__main__':
    unittest.main()
