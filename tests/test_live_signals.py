"""Time-of-day switches must finish clearance for overlapping signal plans."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from yeouido.live import Live


class SignalSwitchTests(unittest.TestCase):
    def setUp(self):
        self.state = 'Grg'
        self.writes = []
        def set_state(tid, state):
            self.state = state
            self.writes.append(state)
        tls = SimpleNamespace(
            getRedYellowGreenState=lambda tid: self.state,
            getNextSwitch=lambda tid: 99,
            getProgram=lambda tid: 'overlapping',
            getAllProgramLogics=lambda tid: [SimpleNamespace(
                programID='overlapping', phases=[SimpleNamespace(state='Grg', duration=30),
                                                 SimpleNamespace(state='yry', duration=3)])],
            setRedYellowGreenState=set_state)
        self.live = Live.__new__(Live)
        self.live.traci = SimpleNamespace(trafficlight=tls)
        self.live.pending_signals = {'tls'}
        self.live.signal_transitions = {}
        self.live.level = 'day'

    @patch('yeouido.live.activate_program')
    def test_overlapping_plan_waits_for_yellow_then_red(self, activate):
        self.live._switch_signals(0)
        self.assertEqual(self.writes, ['yry'])
        self.live._switch_signals(2.5)
        self.assertFalse(activate.called)
        self.live._switch_signals(3)
        self.assertEqual(self.writes, ['yry', 'rrr'])
        self.live._switch_signals(10.5)
        self.assertFalse(activate.called)
        self.live._switch_signals(11)
        activate.assert_called_once_with(self.live.traci, 'tls', 'day', 11)
        self.assertFalse(self.live.pending_signals)
        self.assertFalse(self.live.signal_transitions)

    @patch('yeouido.live.activate_program')
    def test_new_request_during_clearance_uses_latest_target(self, activate):
        self.live._switch_signals(0)
        self.live.level = 'night'
        self.live._switch_signals(3)
        self.live._switch_signals(11)
        self.assertEqual(activate.call_args.args[2], 'night')

    @patch('yeouido.live.activate_program')
    def test_natural_all_red_keeps_existing_clearance(self, activate):
        tls = self.live.traci.trafficlight
        tls.getAllProgramLogics = lambda tid: [SimpleNamespace(
            programID='overlapping', phases=[SimpleNamespace(state='rrr', duration=2)])]
        self.state = 'rrr'
        tls.getNextSwitch = lambda tid: 2
        self.live._switch_signals(0)
        self.assertFalse(activate.called)
        self.live._switch_signals(1.5)
        self.assertTrue(activate.called)
        self.assertFalse(self.writes)


if __name__ == '__main__':
    unittest.main()
