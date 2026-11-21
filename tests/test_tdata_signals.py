"""Measured cycles must never bridge telemetry gaps or invent missing arrows."""
import unittest

from yeouido.tdata_signals import FIELDS, FIELD_INDEX, complete_cycle, translated, validate_cycle, check_conflicts


def observation(char):
    state = ['-'] * len(FIELDS)
    state[FIELD_INDEX['NT_STSG_STAT_NM']] = char
    return ''.join(state)


def observed_cycle(start):
    return [[start, start + 29, observation('G')],
            [start + 30, start + 32, observation('y')],
            [start + 33, start + 34, observation('r')]]


class MeasuredSignalsTests(unittest.TestCase):
    mapping = {0: ('NT', 'STSG', 's', None)}

    def test_directional_clearance_without_junction_wide_red(self):
        validate_cycle([(30, 'Gr'), (3, 'yr'), (30, 'rG'), (3, 'ry')], {0: (), 1: ()})

    def test_missing_observation_outside_repeated_cycles_is_ignored(self):
        runs = [[-10, -9, '?' * len(FIELDS)]]
        for start in (0, 35, 70, 105):
            runs.extend(observed_cycle(start))
        cycle, _, _ = complete_cycle(runs, self.mapping)
        self.assertEqual(cycle, 35)

    def test_protected_green_conflicting_with_yellow_is_rejected(self):
        class Junction:
            def areFoes(self, a, b):
                return a != b
        junction = Junction()
        class Connection:
            def __init__(self, index):
                self.index = index
            def getJunction(self):
                return junction
            def getJunctionIndex(self):
                return self.index
        mapping = {i: (None, None, None, Connection(i)) for i in range(2)}
        with self.assertRaisesRegex(ValueError, '황색'):
            check_conflicts([(3, 'Gy')], mapping)

    def test_repeated_complete_cycles_can_be_separated_by_data_gap(self):
        runs = observed_cycle(0) + observed_cycle(35) + observed_cycle(350) + observed_cycle(385)
        cycle, _, phases = complete_cycle(runs, self.mapping)
        self.assertEqual(cycle, 35)
        self.assertEqual({state: duration for duration, state in phases}, {'G': 30, 'y': 3, 'r': 2})
        states = [state for _, state in phases]
        self.assertIn(''.join(states), ('Gyr', 'yrG', 'rGy'))

    def test_gap_is_not_treated_as_a_long_signal_phase(self):
        runs = []
        for start in (0, 350, 700):
            # Missing observations between green and yellow invalidate the cycle.
            runs += [[start, start + 10, observation('G')],
                     [start + 30, start + 32, observation('y')],
                     [start + 33, start + 34, observation('r')]]
        with self.assertRaises(ValueError):
            complete_cycle(runs, self.mapping)

    def test_single_observed_cycle_is_not_enough(self):
        with self.assertRaises(ValueError):
            complete_cycle(observed_cycle(0) + [[35, 36, observation('G')]], self.mapping)

    def test_missing_arrow_rejects_the_mapping(self):
        with self.assertRaisesRegex(ValueError, '관측 없음'):
            translated(observation('G'), {0: ('NT', 'LTSG', 'l', None)})

    def test_right_turn_yields_and_obeys_red(self):
        right = {0: ('NT', 'STSG', 'r', None)}
        self.assertEqual(translated(observation('G'), right), 'g')
        self.assertEqual(translated(observation('r'), right), 'r')

    def test_missing_yellow_is_rejected(self):
        runs = []
        for start in (0, 35, 70, 105):
            runs += [[start, start + 31, observation('G')],
                     [start + 32, start + 34, observation('r')]]
        with self.assertRaisesRegex(ValueError, '황색'):
            complete_cycle(runs, self.mapping)


if __name__ == '__main__':
    unittest.main()
