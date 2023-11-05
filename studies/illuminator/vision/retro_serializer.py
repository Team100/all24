# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring

import msgpack

def serialize(result) -> bytes:
    tapes = {}
    tapes["tapes"] = []

    for result_item in result:
        tapes["tapes"].append(
            {
                "pose_t": result_item.pose_t.tolist(),
            }
        )

    return msgpack.packb(tapes)
