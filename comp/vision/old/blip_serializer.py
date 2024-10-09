# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring

import msgpack # type: ignore

def serialize(result) -> bytes: # type: ignore
    tags = {}
    tags["tags"] = []

    for result_item in result: # type: ignore
        if result_item.hamming > 0: # type: ignore
            continue

        tags["tags"].append( # type: ignore
            {
                "id": result_item.tag_id, # type: ignore
                "pose_t": result_item.pose_t.tolist(), # type: ignore
                "pose_R": result_item.pose_R.tolist(), # type: ignore
            }
        )

    return msgpack.packb(tags) # type: ignore
