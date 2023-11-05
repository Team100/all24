# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring

import msgpack

def serialize(result) -> bytes:
    tags = {}
    tags["tags"] = []

    for result_item in result:
        if result_item.hamming > 0:
            continue

        tags["tags"].append(
            {
                "id": result_item.tag_id,
                "pose_t": result_item.pose_t.tolist(),
                "pose_R": result_item.pose_R.tolist(),
            }
        )

    return msgpack.packb(tags)
