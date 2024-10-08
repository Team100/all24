import unittest
from app.config.identity import Identity


class IdentityTest(unittest.TestCase):
    def test_identity(self) -> None:
        identity: Identity = Identity.get()
        self.assertEqual("unknown", identity.value)
        