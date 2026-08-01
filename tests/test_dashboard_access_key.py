import os
from pathlib import Path
import stat
import tempfile
import unittest

from robot_agent.dashboard_access_key import (
    DashboardAccessKeyError,
    load_or_create_dashboard_access_key,
)


class DashboardAccessKeyTests(unittest.TestCase):
    def test_first_launch_creates_owner_only_key_and_reuses_it(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "private" / "console-access-key"
            first = load_or_create_dashboard_access_key(
                path,
                token_factory=lambda: "a" * 64,
            )

            def must_not_generate_again():
                raise AssertionError("existing access key must be reused")

            second = load_or_create_dashboard_access_key(
                path,
                token_factory=must_not_generate_again,
            )

            self.assertEqual(first, "a" * 64)
            self.assertEqual(second, first)
            self.assertEqual(
                stat.S_IMODE(path.stat().st_mode),
                stat.S_IRUSR | stat.S_IWUSR,
            )

    def test_existing_insecure_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console-access-key"
            path.write_text("a" * 64 + "\n", encoding="ascii")
            path.chmod(0o644)

            with self.assertRaisesRegex(
                DashboardAccessKeyError,
                "owner-only",
            ):
                load_or_create_dashboard_access_key(path)

    def test_symlink_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "target"
            target.write_text("a" * 64 + "\n", encoding="ascii")
            target.chmod(0o600)
            link = Path(directory) / "console-access-key"
            link.symlink_to(target)

            with self.assertRaises(DashboardAccessKeyError):
                load_or_create_dashboard_access_key(link)

    def test_invalid_factory_value_does_not_create_a_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console-access-key"

            with self.assertRaises(DashboardAccessKeyError):
                load_or_create_dashboard_access_key(
                    path,
                    token_factory=lambda: "too short",
                )

            self.assertFalse(path.exists())

    def test_control_char_in_existing_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "console-access-key"
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT, 0o600)
            try:
                os.write(descriptor, ("a" * 40 + "\tbad\n").encode("ascii"))
            finally:
                os.close(descriptor)

            with self.assertRaises(DashboardAccessKeyError):
                load_or_create_dashboard_access_key(path)


if __name__ == "__main__":
    unittest.main()
