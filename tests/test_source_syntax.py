from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = (
    PROJECT_ROOT / "src",
    PROJECT_ROOT / "ev3",
)


class SourceSyntaxTests(unittest.TestCase):
    def test_every_python_source_compiles_without_writing_bytecode(self):
        failures = []
        for root in SOURCE_ROOTS:
            for path in sorted(root.rglob("*.py")):
                try:
                    compile(
                        path.read_text(encoding="utf-8"),
                        str(path),
                        "exec",
                    )
                except (SyntaxError, ValueError) as exc:
                    failures.append("{}: {}".format(path, exc))

        self.assertEqual(
            failures,
            [],
            "Python source syntax failures:\n{}".format("\n".join(failures)),
        )
