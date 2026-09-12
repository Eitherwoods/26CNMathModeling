"""Attachment timing example only; not a search or localization algorithm."""


def solve(context):
    responses = [context.measure(300, 400, 1), context.measure(300, 400, 2),
                 context.clear(300, 0, 3), context.measure(300, 0, 2)]
    return {'demo_only': True,
            'action_virtual_times_s': [r['virtual_time_s'] for r in responses]}
