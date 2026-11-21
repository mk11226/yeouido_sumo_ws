import io
import json
import unittest
import urllib.error
from unittest.mock import patch

from yeouido.tdata_current import CurrentAPIError, CurrentClient, ENDPOINT, collect, summarize


class CurrentClientTests(unittest.TestCase):
    def test_current_endpoint_and_documented_key_parameter(self):
        row = {'itstId': '148', 'ntStsgStatNm': 'stop-And-Remain'}
        response = io.BytesIO(json.dumps([row]).encode())
        with patch('urllib.request.urlopen', return_value=response) as call:
            rows = CurrentClient(key='test-key').request(intersection='148', rows=1)
        self.assertEqual(rows, [row])
        url = call.call_args.args[0]
        self.assertTrue(url.startswith(ENDPOINT + '?'))
        self.assertIn('apikey=test-key', url)
        self.assertIn('itstId=148', url)
        self.assertNotIn('v2xSignalPhaseInformation', url)

    def test_server_error_does_not_leak_key_or_fall_back(self):
        error = urllib.error.HTTPError('https://example.invalid/?apikey=secret-key', 403,
                                       'denied', {}, io.BytesIO(b'{"error":"secret-key"}'))
        with patch('urllib.request.urlopen', side_effect=error) as call:
            with self.assertRaises(CurrentAPIError) as raised:
                CurrentClient(key='secret-key').request(district='11190')
        self.assertEqual(call.call_count, 1)
        self.assertNotIn('secret-key', str(raised.exception))
        self.assertEqual(raised.exception.status, 403)

    def test_rate_limit_is_reported_without_repeated_requests(self):
        error = urllib.error.HTTPError('https://example.invalid/', 429, 'limited', {},
                                       io.BytesIO(b'{"retryAfterSeconds":30}'))
        with patch('urllib.request.urlopen', side_effect=error) as call:
            with self.assertRaises(CurrentAPIError) as raised:
                CurrentClient(key='test').request(district='11190')
        self.assertEqual(call.call_count, 1)
        self.assertEqual(raised.exception.retry_after, 30)

    def test_missing_filter_or_excessive_page_size_never_sends_request(self):
        with patch('urllib.request.urlopen') as call:
            with self.assertRaises(ValueError):
                CurrentClient(key='test').request()
            with self.assertRaises(ValueError):
                CurrentClient(key='test').request(district='11190', rows=5001)
        call.assert_not_called()

    def test_error_object_is_not_accepted_as_signal_data(self):
        response = io.BytesIO(b'{"status":200,"message":"not authorized"}')
        with patch('urllib.request.urlopen', return_value=response):
            with self.assertRaises(CurrentAPIError):
                CurrentClient(key='test').request(district='11190')

    def test_old_and_missing_timestamps_are_not_current(self):
        rows = [{'itstId': 'fresh', 'trsmUtcTime': 99000},
                {'itstId': 'old', 'trsmUtcTime': 80000}, {'itstId': 'no_time'},
                {'itstId': 'outside', 'trsmUtcTime': 99000}]
        report = summarize(rows, {'fresh', 'old', 'no_time', 'absent'}, 100)
        self.assertEqual(report['received_intersections'], 3)
        self.assertEqual(report['fresh_intersection_ids'], ['fresh'])
        self.assertEqual(report['stale_intersection_ids'], ['no_time', 'old'])
        self.assertEqual(report['missing_intersection_ids'], ['absent'])

    def test_gateway_quota_payload_is_preserved_without_retry(self):
        payload = b'{"headers":{"X-RateLimit-Limit":"1000","X-RateLimit-Remaining":"-1","X-RateLimit-Reset":"5661"}}'
        error = urllib.error.HTTPError('https://example.invalid/', 429, 'limited', {}, io.BytesIO(payload))
        client = CurrentClient(key='test')
        with patch('urllib.request.urlopen', side_effect=error) as call:
            with self.assertRaises(CurrentAPIError) as raised:
                client.request(district='11190')
            with self.assertRaises(CurrentAPIError):
                client.request(district='11190')
        self.assertEqual(call.call_count, 1)
        self.assertEqual(raised.exception.quota['X-RateLimit-Limit'], '1000')

    def test_request_budget_is_enforced_before_network(self):
        client = CurrentClient(key='test', max_requests=0)
        with patch('urllib.request.urlopen') as call:
            with self.assertRaises(CurrentAPIError):
                client.request(district='11190')
        call.assert_not_called()

    def test_unaffordable_recording_is_rejected_before_loading_key_or_sending_requests(self):
        from pathlib import Path
        with patch('urllib.request.urlopen') as call, patch('yeouido.tdata_current.local_key') as key:
            with self.assertRaises(ValueError):
                collect(480, 1, Path('/unused'), max_requests=900)
        call.assert_not_called()
        key.assert_not_called()


if __name__ == '__main__':
    unittest.main()
