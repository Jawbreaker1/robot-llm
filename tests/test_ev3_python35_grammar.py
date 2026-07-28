import ast
from pathlib import Path
import unittest


EV3_ROOT = Path(__file__).resolve().parents[1] / "ev3"


class EV3Python35GrammarTests(unittest.TestCase):
    def test_every_ev3_module_uses_python35_grammar(self):
        failures = []
        for path in sorted(EV3_ROOT.glob("*.py")):
            try:
                ast.parse(
                    path.read_text(encoding="utf-8"),
                    filename=str(path),
                    feature_version=5,
                )
            except SyntaxError as exc:
                failures.append("{}: {}".format(path.name, exc))

        self.assertEqual(
            failures,
            [],
            "EV3 modules must remain parseable by Python 3.5:\n{}".format(
                "\n".join(failures)
            ),
        )
