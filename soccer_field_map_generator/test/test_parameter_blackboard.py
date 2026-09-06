"""Verify field geometry export units and parameter names."""

from soccer_field_map_generator.generator import generate_parameter_blackboard


def test_parameter_blackboard():
    result = generate_parameter_blackboard(
        {
            'field_length': 1400,
            'field_width': 900,
            'border_strip_width': 100,
            'penalty_area_length': 300,
            'center_circle_diameter': 300,
            'goal_width': 260,
            'goal_depth': 75,
        }
    )
    field = result['parameter_blackboard']['ros__parameters']['field']
    assert field == {
        'size': {'x': 14.0, 'y': 9.0, 'padding': 1.0},
        'markings': {
            'penalty_area': {'size': {'x': 3.0}},
            'center_circle': {'diameter': 3.0},
        },
        'goal': {'width': 2.6, 'depth': 0.75},
    }
