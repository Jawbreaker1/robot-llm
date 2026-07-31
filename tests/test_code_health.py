import ast
from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOTS = (
    PROJECT_ROOT / "src",
    PROJECT_ROOT / "ev3",
)
JAVASCRIPT_ROOT = PROJECT_ROOT / "src" / "robot_agent" / "dashboard_web"

# New production modules must stay comfortably below the size of the
# historical supervisor.  Its exact current size is a debt ceiling: it may be
# split into smaller modules, but it may not silently grow into a larger
# monolith.
MAX_PYTHON_MODULE_LINES = 1_800
LEGACY_PYTHON_LINE_BUDGETS = {
    "ev3/supervisor.py": 3_638,
}
MAX_JAVASCRIPT_MODULE_LINES = 1_900
MAX_FUNCTION_LINES = 500


def _relative(path):
    return path.relative_to(PROJECT_ROOT).as_posix()


class CodeHealthTests(unittest.TestCase):
    def test_production_modules_stay_within_explicit_size_budgets(self):
        failures = []
        for root in PYTHON_ROOTS:
            for path in sorted(root.rglob("*.py")):
                relative = _relative(path)
                line_count = len(
                    path.read_text(encoding="utf-8").splitlines()
                )
                budget = LEGACY_PYTHON_LINE_BUDGETS.get(
                    relative,
                    MAX_PYTHON_MODULE_LINES,
                )
                if line_count > budget:
                    failures.append(
                        "{} has {} lines (budget {})".format(
                            relative,
                            line_count,
                            budget,
                        )
                    )

        for path in sorted(JAVASCRIPT_ROOT.glob("*.js")):
            line_count = len(
                path.read_text(encoding="utf-8").splitlines()
            )
            if line_count > MAX_JAVASCRIPT_MODULE_LINES:
                failures.append(
                    "{} has {} lines (budget {})".format(
                        _relative(path),
                        line_count,
                        MAX_JAVASCRIPT_MODULE_LINES,
                    )
                )

        self.assertEqual(
            failures,
            [],
            "Production module size budgets exceeded:\n{}".format(
                "\n".join(failures)
            ),
        )

    def test_python_functions_stay_bounded(self):
        failures = []
        for root in PYTHON_ROOTS:
            for path in sorted(root.rglob("*.py")):
                tree = ast.parse(
                    path.read_text(encoding="utf-8"),
                    filename=str(path),
                )
                for node in ast.walk(tree):
                    if not isinstance(
                        node,
                        (ast.FunctionDef, ast.AsyncFunctionDef),
                    ):
                        continue
                    end_line = getattr(node, "end_lineno", None)
                    if end_line is None:
                        continue
                    line_count = end_line - node.lineno + 1
                    if line_count > MAX_FUNCTION_LINES:
                        failures.append(
                            "{}:{} {} has {} lines (budget {})".format(
                                _relative(path),
                                node.lineno,
                                node.name,
                                line_count,
                                MAX_FUNCTION_LINES,
                            )
                        )

        self.assertEqual(
            failures,
            [],
            "Python function size budgets exceeded:\n{}".format(
                "\n".join(failures)
            ),
        )


if __name__ == "__main__":
    unittest.main()
