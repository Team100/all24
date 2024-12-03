"""Resolve my confusion about struct array listeners."""

# pylint: disable=W0212
import dataclasses
import unittest
from typing import cast

import ntcore
from wpiutil import wpistruct


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Datum:
    field0: int
    field1: int


class StructArrayTest(unittest.TestCase):
    def test_struct(self) -> None:
        """A simple struct pub/sub case."""
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructTopic("foo/1", Datum).subscribe(Datum(0, 0))
        pub = inst.getStructTopic("foo/1", Datum).publish()
        pub.set(Datum(1, 2), ntcore._now())
        queue = sub.readQueue()
        self.assertEqual(1, len(queue))
        item = queue.pop()
        # the Struct subscriber knows the type of the item
        self.assertEqual(Datum, type(item.value))
        self.assertEqual(1, item.value.field0)
        self.assertEqual(2, item.value.field1)

    def test_subscriber_keys_match_exactly(self) -> None:
        """The subscriber key is not a prefix."""
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructTopic("foo", Datum).subscribe(Datum(0, 0))
        pub = inst.getStructTopic("foo/1", Datum).publish()
        pub.set(Datum(1, 2), ntcore._now())
        queue = sub.readQueue()
        self.assertEqual(0, len(queue))

    def test_struct_array(self) -> None:
        """A simple struct array pub/sub case."""
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructArrayTopic("foo/1", Datum).subscribe([])
        pub = inst.getStructArrayTopic("foo/1", Datum).publish()
        pub.set([Datum(1, 2), Datum(3, 4)], ntcore._now())
        queue = sub.readQueue()
        self.assertEqual(1, len(queue))
        item = queue.pop()
        # the StructArray subscriber knows the type of the item
        self.assertEqual(list, type(item.value))
        self.assertEqual(Datum, type(item.value[0]))
        self.assertEqual(2, len(item.value))
        value: Datum = item.value[0]
        self.assertEqual(1, value.field0)
        self.assertEqual(2, value.field1)
        value: Datum = item.value[1]
        self.assertEqual(3, value.field0)
        self.assertEqual(4, value.field1)

    def test_struct_listener(self) -> None:
        """A simple struct pub/listener case."""
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        poller = ntcore.NetworkTableListenerPoller(inst)
        poller.addListener(["foo"], ntcore.EventFlags.kValueAll)
        pub = inst.getStructTopic("foo/1", Datum).publish()
        pub.set(Datum(1, 2), ntcore._now())
        queue = poller.readQueue()
        self.assertEqual(1, len(queue))
        event = queue.pop()
        item = cast(ntcore.ValueEventData, event.data)
        # the value type is not known here
        self.assertEqual(ntcore.Value, type(item.value))
        value = wpistruct.unpack(Datum, item.value.getRaw())
        self.assertEqual(1, value.field0)
        self.assertEqual(2, value.field1)

    def test_struct_array_listener(self) -> None:
        """A simple struct array pub/listener case."""
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        poller = ntcore.NetworkTableListenerPoller(inst)
        poller.addListener(["foo"], ntcore.EventFlags.kValueAll)
        pub = inst.getStructArrayTopic("foo/1", Datum).publish()
        pub.set([Datum(1, 2), Datum(3, 4)], ntcore._now())
        queue = poller.readQueue()
        self.assertEqual(1, len(queue))
        event = queue.pop()
        # item =  event.data
        item = cast(ntcore.ValueEventData, event.data)
        self.assertEqual(ntcore.Value, type(item.value))
        item_size = wpistruct.getSize(Datum)
        self.assertEqual(8, item_size)
        # raw_array: bytes =  item.value.getRaw()
        raw_array: bytes = cast(bytes, item.value.getRaw())
        self.assertEqual(16, len(raw_array))
        # step through the byte array one Datum at a time.
        raw_item_array = [
            raw_array[i : i + item_size] for i in range(0, len(raw_array), item_size)
        ]
        self.assertEqual(2, len(raw_item_array))
        value: Datum = wpistruct.unpack(Datum, raw_item_array[0])
        self.assertEqual(1, value.field0)
        self.assertEqual(2, value.field1)
        value: Datum = wpistruct.unpack(Datum, raw_item_array[1])
        self.assertEqual(3, value.field0)
        self.assertEqual(4, value.field1)
