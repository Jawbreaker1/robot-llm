"""Small, hardware-scoped vocabulary for model-selected social expressions."""

REPLY_FACES = ("idle", "neutral", "happy", "frustrated", "curious", "surprised", "angry")
REPLY_GESTURES = ("none", "claw_snap", "arm_wave", "claw_flourish")
EV3_GESTURES = ("none", "saw_flourish", "saw_celebrate")
ALL_GESTURES = tuple(dict.fromkeys(REPLY_GESTURES + EV3_GESTURES))


def valid_social_expression(value, *, gestures=REPLY_GESTURES, faces=REPLY_FACES):
    return (
        isinstance(value, dict)
        and set(value) == {"face", "gesture"}
        and value["face"] in faces
        and value["gesture"] in gestures
    )


def social_expression_schema(*, gestures=REPLY_GESTURES, faces=REPLY_FACES):
    return {"oneOf": [
        {"type": "null"},
        {"type": "object", "properties": {
            "face": {"type": "string", "enum": list(faces)},
            "gesture": {"type": "string", "enum": list(gestures)},
        }, "required": ["face", "gesture"], "additionalProperties": False},
    ]}
