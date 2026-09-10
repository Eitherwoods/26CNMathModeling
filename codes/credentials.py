"""Read the local team identifier without exposing stored credentials."""
from pathlib import Path

from .protocol import identifier


TEAM_LABELS = frozenset({'参赛队号', '队号', 'team_no', 'team number', 'robot_id'})


def _label(text):
    return text.strip().strip(':：= ').casefold()


def load_robot_id(path='tester/username-and-password.txt'):
    """Return only the team number; never return or log the adjacent password."""
    source = Path(path)
    try:
        handle = source.open(encoding='utf-8-sig')
    except OSError as exc:
        raise ValueError(f'Cannot read team identifier file: {source}') from exc

    expecting_value = False
    with handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            if expecting_value:
                return identifier(text, 64)
            for separator in ('：', ':', '='):
                if separator in text:
                    key, value = text.split(separator, 1)
                    if _label(key) in TEAM_LABELS and value.strip():
                        return identifier(value.strip(), 64)
            expecting_value = _label(text) in TEAM_LABELS
    raise ValueError(f'No team identifier labelled as 参赛队号 in: {source}')
