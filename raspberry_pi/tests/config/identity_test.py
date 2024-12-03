import platform
import unittest

from app.config.identity import Identity


class IdentityTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.get()
        if platform.machine() == "x86_64":
            self.assertEqual("unknown", identity.value)
